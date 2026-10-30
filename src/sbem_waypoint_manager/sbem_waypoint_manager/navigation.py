"""Track owned Nav2 goals through acceptance, timeout and cancellation.

The action client is supplied by the ROS node. No ROS imports are required here.
"""
from dataclasses import dataclass, field
import threading


@dataclass
class GoalTicket:
    map_id: str
    event: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    message: str = ''
    goal_id: str = ''
    handle: object = None
    invalidated: bool = False
    cancel_sent: bool = False

    def respond(self, success, message):
        if not self.event.is_set():
            self.success = success
            self.message = message
            self.event.set()


class GoalDispatcher:
    def __init__(self, client, log=lambda message: None):
        self._client = client
        self._log = log
        self._lock = threading.RLock()
        self._ticket = None

    @property
    def busy(self):
        with self._lock:
            return self._ticket is not None

    def start(self, goal, map_id):
        with self._lock:
            if self._ticket is not None:
                raise ValueError('Navigation is busy with a pending or active goal')
            ticket = GoalTicket(map_id)
            self._ticket = ticket
            try:
                future = self._client.send_goal_async(goal)
                future.add_done_callback(lambda result: self._accepted(ticket, result))
            except Exception as error:
                self._ticket = None
                ticket.respond(False, f'Could not send navigation goal: {error}')
            return ticket

    def timeout(self, ticket):
        with self._lock:
            if not ticket.event.is_set():
                ticket.invalidated = True
                ticket.respond(False, 'Nav2 acceptance timed out; acceptance unknown. '
                               'A late-accepted goal will be canceled.')
                self._log(ticket.message)
                self._cancel(ticket)

    def map_changed(self, map_id):
        with self._lock:
            ticket = self._ticket
            if ticket is not None and ticket.map_id != map_id:
                ticket.invalidated = True
                ticket.respond(False, 'Map changed while navigation was pending')
                self._log('Map changed; canceling owned navigation goal')
                self._cancel(ticket)

    def _accepted(self, ticket, future):
        with self._lock:
            try:
                handle = future.result()
            except Exception as error:
                ticket.respond(False, f'Nav2 goal acceptance failed: {error}')
                self._release(ticket)
                return
            if not handle.accepted:
                ticket.respond(False, 'Nav2 rejected the navigation goal')
                self._release(ticket)
                return
            ticket.handle = handle
            ticket.goal_id = bytes(handle.goal_id.uuid).hex()
            # Register result tracking even when acceptance is too late to use.
            try:
                handle.get_result_async().add_done_callback(
                    lambda result: self._finished(ticket, result))
            except Exception as error:
                ticket.invalidated = True
                ticket.respond(False, f'Could not track Nav2 result: {error}')
                self._log('Cannot track goal result; retaining busy state until restart')
            if ticket.invalidated:
                self._cancel(ticket)
            else:
                ticket.respond(True, 'Nav2 accepted the navigation goal; arrival is pending')

    def _cancel(self, ticket):
        if ticket.handle is None or ticket.cancel_sent:
            return
        ticket.cancel_sent = True
        try:
            ticket.handle.cancel_goal_async().add_done_callback(self._canceled)
        except Exception as error:
            self._log(f'Could not request Nav2 cancellation: {error}; retaining goal ownership')

    def _canceled(self, future):
        try:
            response = future.result()
            if not response.goals_canceling:
                self._log('Nav2 did not accept cancellation; retaining goal until terminal result')
        except Exception as error:
            self._log(f'Nav2 cancellation failed: {error}; retaining goal ownership')

    def _finished(self, ticket, future):
        with self._lock:
            try:
                result = future.result()
                self._log(f'Navigation goal {ticket.goal_id} finished with status {result.status}')
            except Exception as error:
                self._log(f'Could not obtain navigation result: {error}; retaining goal ownership')
                return
            self._release(ticket)

    def _release(self, ticket):
        if self._ticket is ticket:
            self._ticket = None
