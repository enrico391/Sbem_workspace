"""Goal lifecycle tests using controllable external action futures."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from sbem_waypoint_manager.navigation import GoalDispatcher


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.goal_id = SimpleNamespace(uuid=list(range(16)))
        self.result = Future()
        self.cancel = Future()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        return self.cancel


class Client:
    def __init__(self):
        self.acceptance = Future()
        self.sent_goal = None

    def send_goal_async(self, goal):
        self.sent_goal = goal
        return self.acceptance


def test_acceptance_returns_uuid_and_keeps_busy_until_result():
    client = Client()
    dispatcher = GoalDispatcher(client)
    ticket = dispatcher.start('pose-goal', 'map-a')
    handle = Handle()
    client.acceptance.set_result(handle)
    assert ticket.event.is_set()
    assert ticket.success
    assert ticket.goal_id == '000102030405060708090a0b0c0d0e0f'
    assert client.sent_goal == 'pose-goal'
    with pytest.raises(ValueError, match='busy'):
        dispatcher.start('another', 'map-a')
    handle.result.set_result(SimpleNamespace(status=4))
    assert not dispatcher.busy


def test_rejected_goal_releases_busy_state():
    client = Client()
    dispatcher = GoalDispatcher(client)
    ticket = dispatcher.start('goal', 'map-a')
    client.acceptance.set_result(Handle(accepted=False))
    assert not ticket.success
    assert 'rejected' in ticket.message
    assert not dispatcher.busy


def test_timeout_tracks_and_cancels_late_acceptance():
    client = Client()
    dispatcher = GoalDispatcher(client)
    ticket = dispatcher.start('goal', 'map-a')
    dispatcher.timeout(ticket)
    assert not ticket.success
    assert 'unknown' in ticket.message
    assert dispatcher.busy
    handle = Handle()
    client.acceptance.set_result(handle)
    assert handle.cancel_count == 1
    assert not ticket.success
    handle.cancel.set_result(SimpleNamespace(goals_canceling=[handle.goal_id]))
    assert dispatcher.busy
    handle.result.set_result(SimpleNamespace(status=5))
    assert not dispatcher.busy


@pytest.mark.parametrize('accept_first', [False, True])
def test_map_change_cancels_pending_or_active_goal(accept_first):
    client = Client()
    dispatcher = GoalDispatcher(client)
    ticket = dispatcher.start('goal', 'map-a')
    handle = Handle()
    if accept_first:
        client.acceptance.set_result(handle)
    dispatcher.map_changed('map-b')
    if not accept_first:
        assert not ticket.success
        client.acceptance.set_result(handle)
    assert handle.cancel_count == 1
    assert dispatcher.busy
    dispatcher.map_changed('map-c')
    assert handle.cancel_count == 1
    handle.result.set_result(SimpleNamespace(status=5))
    assert not dispatcher.busy


def test_identical_map_does_not_cancel_and_acceptance_wins_timeout_race():
    client = Client()
    dispatcher = GoalDispatcher(client)
    ticket = dispatcher.start('goal', 'map-a')
    handle = Handle()
    client.acceptance.set_result(handle)
    dispatcher.map_changed('map-a')
    dispatcher.timeout(ticket)
    assert ticket.success
    assert handle.cancel_count == 0


def test_failed_send_or_acceptance_releases_busy_state():
    client = Client()
    dispatcher = GoalDispatcher(client)
    ticket = dispatcher.start('goal', 'map-a')
    client.acceptance.set_exception(RuntimeError('connection lost'))
    assert not ticket.success
    assert 'connection lost' in ticket.message
    assert not dispatcher.busy


def test_cancel_rejection_retains_goal_ownership_until_result():
    messages = []
    client = Client()
    dispatcher = GoalDispatcher(client, messages.append)
    dispatcher.start('goal', 'map-a')
    handle = Handle()
    client.acceptance.set_result(handle)
    dispatcher.map_changed('map-b')
    handle.cancel.set_result(SimpleNamespace(goals_canceling=[]))
    assert dispatcher.busy
    assert any('cancel' in message.lower() for message in messages)
    handle.result.set_result(SimpleNamespace(status=4))
    assert not dispatcher.busy
