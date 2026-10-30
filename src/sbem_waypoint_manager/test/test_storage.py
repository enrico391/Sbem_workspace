"""Persistence contracts: real SQLite databases, no database mocks."""
import sqlite3
from dataclasses import replace

import pytest

from sbem_waypoint_manager.storage import MapSnapshot, WaypointStore


@pytest.fixture
def grid():
    return MapSnapshot('map', 2, 2, 0.05, (1., 2., 0., 0., 0., 0., 1.),
                       (-1, 0, 100, 50))


@pytest.fixture
def store(tmp_path):
    instance = WaypointStore(str(tmp_path / 'locations.db'))
    yield instance
    instance.close()


def test_map_and_pose_survive_reopen(tmp_path, grid):
    path = str(tmp_path / 'persistent.db')
    first = WaypointStore(path)
    first.save('kitchen', grid, (3., 4., 0., 0., 0., 0., 1.), 123456789)
    first.close()
    reopened = WaypointStore(path)
    try:
        location = reopened.get('kitchen', grid.map_id)
        assert location.pose == (3., 4., 0., 0., 0., 0., 1.)
        assert location.captured_ns == 123456789
        assert reopened.get_map(grid.map_id) == grid
    finally:
        reopened.close()


def test_maps_deduplicate_and_names_are_scoped_per_map(store, grid):
    other = replace(grid, data=(0, 0, 100, 50))
    store.save('home', grid, grid.origin, 1)
    store.save('desk', grid, grid.origin, 2)
    store.save('home', other, (5., 6., 0., 0., 0., 0., 1.), 3)
    assert [p.name for p in store.list(grid.map_id)] == ['desk', 'home']
    assert store.get('home', other.map_id).pose[:2] == (5., 6.)
    assert grid.map_id != other.map_id
    assert replace(grid).map_id == grid.map_id
    assert len(store.list(other.map_id)) == 1


def test_duplicate_requires_explicit_overwrite(store, grid):
    store.save('home', grid, grid.origin, 1)
    with pytest.raises(ValueError, match='already exists'):
        store.save('home', grid, (8., 9., 0., 0., 0., 0., 1.), 2)
    assert store.get('home', grid.map_id).captured_ns == 1
    store.save('home', grid, (8., 9., 0., 0., 0., 0., 1.), 2, overwrite=True)
    assert store.get('home', grid.map_id).pose[:2] == (8., 9.)
    store.delete('home', grid.map_id)
    assert store.list(grid.map_id) == []
    with pytest.raises(KeyError):
        store.get('home', grid.map_id)
    with pytest.raises(KeyError):
        store.delete('home', grid.map_id)


def test_transaction_rolls_back_map_when_location_write_fails(tmp_path, grid):
    path = str(tmp_path / 'rollback.db')
    store = WaypointStore(path)
    with sqlite3.connect(path) as connection:
        connection.execute("CREATE TRIGGER reject_location BEFORE INSERT ON locations "
                           "BEGIN SELECT RAISE(ABORT, 'disk write failed'); END")
    try:
        with pytest.raises(sqlite3.DatabaseError):
            store.save('home', grid, grid.origin, 1)
        with pytest.raises(KeyError):
            store.get_map(grid.map_id)
    finally:
        store.close()


@pytest.mark.parametrize('name', ['', '   ', ' home ', 'a\x00b'])
def test_rejects_invalid_names(store, grid, name):
    with pytest.raises(ValueError):
        store.save(name, grid, grid.origin, 1)


@pytest.mark.parametrize('pose', [
    (float('nan'), 0., 0., 0., 0., 0., 1.),
    (0., 0., 0., 0., 0., 0., 0.),
    (0., 0., 0.),
])
def test_rejects_invalid_poses(store, grid, pose):
    with pytest.raises(ValueError):
        store.save('home', grid, pose, 1)


@pytest.mark.parametrize('changes', [
    {'width': 3}, {'resolution': 0.}, {'resolution': float('inf')},
    {'frame_id': ''}, {'data': (-2, 0, 100, 50)}, {'data': (101, 0, 100, 50)},
    {'origin': (0., 0., 0., 0., 0., 0., 0.)},
])
def test_rejects_invalid_maps(grid, changes):
    with pytest.raises(ValueError):
        replace(grid, **changes)


def test_unknown_database_version_is_not_modified(tmp_path):
    path = str(tmp_path / 'future.db')
    with sqlite3.connect(path) as connection:
        connection.execute('PRAGMA user_version = 99')
    with pytest.raises(ValueError, match='version'):
        WaypointStore(path)
    with sqlite3.connect(path) as connection:
        assert connection.execute('PRAGMA user_version').fetchone()[0] == 99
