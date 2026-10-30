"""Transactional, ROS-independent SQLite storage for named map poses."""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import threading
import zlib


def validate_pose(pose):
    if len(pose) != 7 or not all(math.isfinite(v) for v in pose):
        raise ValueError('Pose must contain seven finite translation/quaternion values')
    if abs(sum(v * v for v in pose[3:]) - 1.0) > 1e-3:
        raise ValueError('Pose quaternion must be normalized')


def validate_name(name):
    if not name or name != name.strip() or '\x00' in name or len(name) > 255:
        raise ValueError('Name must be 1–255 characters, without surrounding whitespace or NUL')


@dataclass(frozen=True)
class MapSnapshot:
    frame_id: str
    width: int
    height: int
    resolution: float
    origin: tuple
    data: tuple

    def __post_init__(self):
        object.__setattr__(self, 'origin', tuple(float(v) for v in self.origin))
        object.__setattr__(self, 'data', tuple(self.data))
        if not self.frame_id or self.frame_id != self.frame_id.strip():
            raise ValueError('Map frame must be nonempty without surrounding whitespace')
        if (self.width <= 0 or self.height <= 0
                or not math.isfinite(self.resolution) or self.resolution <= 0):
            raise ValueError('Map dimensions and resolution must be positive and finite')
        if len(self.data) != self.width * self.height:
            raise ValueError('Map occupancy length does not match dimensions')
        if any(not isinstance(v, int) or v < -1 or v > 100 for v in self.data):
            raise ValueError('Map occupancy values must be integers from -1 to 100')
        validate_pose(self.origin)

    @property
    def metadata(self):
        return {'frame_id': self.frame_id, 'width': self.width, 'height': self.height,
                'resolution': float(self.resolution), 'origin': self.origin}

    @property
    def occupancy_bytes(self):
        return bytes(value & 255 for value in self.data)

    @property
    def map_id(self):
        metadata = json.dumps(self.metadata, sort_keys=True, separators=(',', ':'),
                              allow_nan=False).encode()
        return hashlib.sha256(metadata + b'\x00' + self.occupancy_bytes).hexdigest()


@dataclass(frozen=True)
class Location:
    name: str
    map_id: str
    frame_id: str
    pose: tuple
    captured_ns: int


class WaypointStore:
    """One connection protected across executor threads; writes are atomic."""

    def __init__(self, database_path):
        path = Path(database_path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(str(path), timeout=5., check_same_thread=False)
        try:
            version = self._db.execute('PRAGMA user_version').fetchone()[0]
            if version not in (0, 1):
                raise ValueError(f'Unsupported waypoint database version: {version}')
            self._db.execute('PRAGMA foreign_keys = ON')
            with self._db:
                self._db.execute('CREATE TABLE IF NOT EXISTS maps ('
                                 'map_id TEXT PRIMARY KEY, metadata TEXT NOT NULL, '
                                 'occupancy BLOB NOT NULL)')
                self._db.execute('CREATE TABLE IF NOT EXISTS locations ('
                                 'map_id TEXT NOT NULL REFERENCES maps(map_id), '
                                 'name TEXT NOT NULL, pose TEXT NOT NULL, '
                                 'captured_ns INTEGER NOT NULL, PRIMARY KEY(map_id, name))')
                self._db.execute('PRAGMA user_version = 1')
        except Exception:
            self._db.close()
            raise

    def close(self):
        with self._lock:
            self._db.close()

    def save(self, name, grid, pose, captured_ns, overwrite=False):
        validate_name(name)
        validate_pose(pose)
        if not isinstance(captured_ns, int) or captured_ns < 0:
            raise ValueError('Capture time must be nonnegative integer nanoseconds')
        map_id = grid.map_id
        metadata = json.dumps(grid.metadata, allow_nan=False)
        occupancy = zlib.compress(grid.occupancy_bytes)
        encoded_pose = json.dumps(tuple(float(v) for v in pose), allow_nan=False)
        with self._lock, self._db:
            self._db.execute('INSERT OR IGNORE INTO maps VALUES (?, ?, ?)',
                             (map_id, metadata, occupancy))
            exists = self._db.execute(
                'SELECT 1 FROM locations WHERE map_id=? AND name=?', (map_id, name)
            ).fetchone()
            if exists and not overwrite:
                raise ValueError(f'Location {name!r} already exists on this map')
            self._db.execute(
                'INSERT INTO locations VALUES (?, ?, ?, ?) '
                'ON CONFLICT(map_id, name) DO UPDATE SET '
                'pose=excluded.pose, captured_ns=excluded.captured_ns',
                (map_id, name, encoded_pose, captured_ns))
        return Location(name, map_id, grid.frame_id, tuple(pose), captured_ns)

    def get(self, name, map_id):
        validate_name(name)
        with self._lock:
            row = self._db.execute(
                'SELECT l.pose, l.captured_ns, m.metadata FROM locations l '
                'JOIN maps m USING(map_id) WHERE l.map_id=? AND l.name=?',
                (map_id, name)).fetchone()
        if row is None:
            raise KeyError(f'Location {name!r} not found on map {map_id}')
        return Location(name, map_id, json.loads(row[2])['frame_id'],
                        tuple(json.loads(row[0])), row[1])

    def list(self, map_id):
        with self._lock:
            rows = self._db.execute(
                'SELECT l.name, l.pose, l.captured_ns, m.metadata FROM locations l '
                'JOIN maps m USING(map_id) WHERE l.map_id=? ORDER BY l.name',
                (map_id,)).fetchall()
        return [Location(row[0], map_id, json.loads(row[3])['frame_id'],
                         tuple(json.loads(row[1])), row[2]) for row in rows]

    def delete(self, name, map_id):
        validate_name(name)
        with self._lock, self._db:
            result = self._db.execute('DELETE FROM locations WHERE map_id=? AND name=?',
                                      (map_id, name))
            if result.rowcount == 0:
                raise KeyError(f'Location {name!r} not found on map {map_id}')

    def get_map(self, map_id):
        with self._lock:
            row = self._db.execute('SELECT metadata, occupancy FROM maps WHERE map_id=?',
                                   (map_id,)).fetchone()
        if row is None:
            raise KeyError(f'Map {map_id} not found')
        metadata = json.loads(row[0])
        data = tuple(value if value < 128 else value - 256
                     for value in zlib.decompress(row[1]))
        grid = MapSnapshot(**metadata, data=data)
        if grid.map_id != map_id:
            raise ValueError('Stored map content does not match its map ID')
        return grid
