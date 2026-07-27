# 本地传输测试：锁定同步交付、类型稳定性、生命周期与线程安全计数。
from __future__ import annotations

from dataclasses import FrozenInstanceError
import inspect
import math
from threading import Barrier, Event, Thread
import time
from typing import get_args

import pytest

from slope_sim.interfaces.transport import (
    LocalTransport,
    Transport,
    TransportCallback,
    TransportSnapshot,
)


UINT64_MAX = (1 << 64) - 1


def test_transport_callback_contract_accepts_bool_or_none_results():
    """公共回调允许显式接受、显式拒绝及旧式无返回值。"""
    _parameters, return_type = get_args(TransportCallback)

    assert set(get_args(return_type)) == {bool, type(None)}


@pytest.mark.parametrize(
    ("callback_result", "expected_publish_result"),
    ((True, True), (None, True), (False, False)),
)
def test_local_transport_honors_explicit_callback_rejection(
    callback_result,
    expected_publish_result,
):
    """None 兼容旧回调，False 在本地与 eCAL 中都表示明确拒绝。"""
    delivered: list[bytes] = []
    transport = LocalTransport()

    def callback(payload: bytes, _received_at: float) -> bool | None:
        delivered.append(payload)
        return callback_result

    transport.subscribe("topic", "TypeA", callback)

    assert (
        transport.publish("topic", b"payload", "TypeA", 1, wall_time=1.0)
        is expected_publish_result
    )
    assert delivered == [b"payload"]
    assert transport.snapshot().error_count == 0


def test_local_publish_delivers_at_explicit_wall_time_and_counts_then_closes_idempotently():
    clock_calls: list[None] = []

    def clock() -> float:
        clock_calls.append(None)
        return 99.0

    received: list[tuple[bytes, float]] = []
    transport = LocalTransport(monotonic=clock)
    subscription = transport.subscribe(
        "/sim/wheel/command",
        "WheelCommand",
        lambda payload, received_at: received.append((payload, received_at)),
    )

    source = bytearray(b"command")
    assert transport.publish(
        "/sim/wheel/command",
        source,
        "WheelCommand",
        123,
        wall_time=2.5,
    )
    source[:] = b"changed"

    assert received == [(b"command", 2.5)]
    assert clock_calls == []
    assert isinstance(transport, Transport)
    snapshot = transport.snapshot()
    assert snapshot == TransportSnapshot(
        mode="local",
        ecal_connected=False,
        published_count=1,
        received_count=1,
        error_count=0,
        dropped_count=0,
    )
    with pytest.raises(FrozenInstanceError):
        snapshot.published_count = 2

    subscription.close()
    subscription.close()
    assert transport.publish("/sim/wheel/command", b"ignored", "WheelCommand", 124, wall_time=2.6)
    assert transport.snapshot().published_count == 2
    assert transport.snapshot().received_count == 1

    transport.close()
    transport.close()
    with pytest.raises(RuntimeError, match="closed"):
        transport.publish("/sim/wheel/command", b"late", "WheelCommand", 125, wall_time=2.7)
    with pytest.raises(RuntimeError, match="closed"):
        transport.subscribe(
            "/sim/wheel/command",
            "WheelCommand",
            lambda payload, received_at: received.append((payload, received_at)),
        )


def test_publish_uses_injected_monotonic_when_wall_time_is_omitted():
    clock_values = iter((4.25,))
    delivered: list[tuple[bytes, float]] = []
    transport = LocalTransport(monotonic=lambda: next(clock_values))
    transport.subscribe(
        "topic",
        "TypeA",
        lambda payload, received_at: delivered.append((payload, received_at)),
    )

    assert transport.publish("topic", memoryview(b"payload"), "TypeA", UINT64_MAX)

    assert delivered == [(b"payload", 4.25)]


def test_topic_type_conflicts_are_rejected_without_partial_publish_state():
    delivered: list[tuple[bytes, float]] = []
    transport = LocalTransport()
    transport.subscribe(
        "topic",
        "TypeA",
        lambda payload, received_at: delivered.append((payload, received_at)),
    )

    with pytest.raises(ValueError, match="type"):
        transport.subscribe(
            "topic",
            "TypeB",
            lambda payload, received_at: delivered.append((payload, received_at)),
        )
    with pytest.raises(ValueError, match="type"):
        transport.publish("topic", b"wrong", "TypeB", 1, wall_time=1.0)

    assert transport.snapshot().published_count == 0
    assert transport.publish("topic", b"right", "TypeA", 2, wall_time=1.1)
    assert delivered == [(b"right", 1.1)]


def test_first_publish_also_binds_topic_type():
    transport = LocalTransport()

    assert transport.publish("topic", b"first", "TypeA", 1, wall_time=1.0)
    with pytest.raises(ValueError, match="type"):
        transport.subscribe("topic", "TypeB", lambda _payload, _received_at: None)
    with pytest.raises(ValueError, match="type"):
        transport.publish("topic", b"second", "TypeB", 2, wall_time=2.0)

    assert transport.snapshot().published_count == 1


def test_callback_may_close_its_subscription_without_corrupting_delivery_snapshot():
    transport = LocalTransport()
    calls: list[tuple[str, bytes]] = []
    self_subscription = []

    def close_self(payload: bytes, _received_at: float) -> None:
        calls.append(("self", payload))
        self_subscription[0].close()

    self_subscription.append(transport.subscribe("topic", "TypeA", close_self))
    transport.subscribe(
        "topic",
        "TypeA",
        lambda payload, _received_at: calls.append(("other", payload)),
    )

    assert transport.publish("topic", b"one", "TypeA", 1, wall_time=1.0)
    assert transport.publish("topic", b"two", "TypeA", 2, wall_time=2.0)

    assert calls == [("self", b"one"), ("other", b"one"), ("other", b"two")]
    assert transport.snapshot().received_count == 3


def test_callback_failure_is_counted_and_does_not_skip_later_subscribers():
    transport = LocalTransport()
    calls: list[str] = []

    def failing_callback(_payload: bytes, _received_at: float) -> None:
        calls.append("failing")
        raise RuntimeError("callback failed")

    transport.subscribe("topic", "TypeA", failing_callback)
    transport.subscribe("topic", "TypeA", lambda _payload, _received_at: calls.append("healthy"))

    assert not transport.publish("topic", b"payload", "TypeA", 1, wall_time=1.0)

    assert calls == ["failing", "healthy"]
    assert transport.snapshot() == TransportSnapshot("local", False, 1, 2, 1, 0)


@pytest.mark.parametrize("topic", ("", None, 1))
def test_subscribe_and_publish_reject_invalid_topics_without_reserving_a_type(topic):
    transport = LocalTransport()

    with pytest.raises(ValueError, match="topic"):
        transport.subscribe(topic, "TypeA", lambda _payload, _received_at: None)
    with pytest.raises(ValueError, match="topic"):
        transport.publish(topic, b"payload", "TypeA", 1, wall_time=1.0)

    assert transport.snapshot().published_count == 0


@pytest.mark.parametrize("type_name", ("", None, 1))
def test_subscribe_and_publish_reject_invalid_type_names(type_name):
    transport = LocalTransport()

    with pytest.raises(ValueError, match="type_name"):
        transport.subscribe("topic", type_name, lambda _payload, _received_at: None)
    with pytest.raises(ValueError, match="type_name"):
        transport.publish("topic", b"payload", type_name, 1, wall_time=1.0)


def test_subscribe_requires_a_callable_before_binding_topic_type():
    transport = LocalTransport()

    with pytest.raises(ValueError, match="callback"):
        transport.subscribe("topic", "TypeA", None)

    transport.subscribe("topic", "TypeB", lambda _payload, _received_at: None)


@pytest.mark.parametrize("payload", ("payload", 4, [1, 2]))
def test_publish_rejects_non_bytes_like_payload_without_binding_topic(payload):
    transport = LocalTransport()

    with pytest.raises(ValueError, match="payload"):
        transport.publish("topic", payload, "TypeA", 1, wall_time=1.0)

    transport.subscribe("topic", "TypeB", lambda _payload, _received_at: None)
    assert transport.snapshot().published_count == 0


@pytest.mark.parametrize("sim_time_ns", (True, -1, 1.0, UINT64_MAX + 1))
def test_publish_rejects_non_uint64_sim_time_without_partial_state(sim_time_ns):
    transport = LocalTransport()

    with pytest.raises(ValueError, match="sim_time_ns"):
        transport.publish("topic", b"payload", "TypeA", sim_time_ns, wall_time=1.0)

    assert transport.snapshot().published_count == 0


@pytest.mark.parametrize("wall_time", (True, -1.0, math.nan, math.inf, -math.inf, "1"))
def test_publish_rejects_invalid_wall_time_without_partial_state(wall_time):
    transport = LocalTransport()

    with pytest.raises(ValueError, match="wall_time"):
        transport.publish("topic", b"payload", "TypeA", 1, wall_time=wall_time)

    transport.subscribe("topic", "TypeB", lambda _payload, _received_at: None)
    assert transport.snapshot().published_count == 0


def test_local_transport_constructor_exposes_exact_monotonic_parameter():
    signature = inspect.signature(LocalTransport)

    assert tuple(signature.parameters) == ("monotonic",)
    assert signature.parameters["monotonic"].default is None


def test_local_transport_requires_a_callable_monotonic():
    with pytest.raises(ValueError, match="monotonic"):
        LocalTransport(monotonic=1)


def test_concurrent_first_subscriptions_atomically_choose_one_topic_type():
    transport = LocalTransport()
    barrier = Barrier(3)
    successes: list[tuple[str, object]] = []
    errors: list[tuple[str, BaseException]] = []
    delivered: list[str] = []

    def subscribe(type_name: str) -> None:
        barrier.wait(timeout=5.0)
        try:
            subscription = transport.subscribe(
                "topic",
                type_name,
                lambda _payload, _received_at: delivered.append(type_name),
            )
            successes.append((type_name, subscription))
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            errors.append((type_name, exc))

    threads = (Thread(target=subscribe, args=("TypeA",)), Thread(target=subscribe, args=("TypeB",)))
    for thread in threads:
        thread.start()
    barrier.wait(timeout=5.0)
    for thread in threads:
        thread.join(timeout=5.0)

    assert not any(thread.is_alive() for thread in threads)
    assert len(successes) == 1
    assert len(errors) == 1
    assert isinstance(errors[0][1], ValueError)
    winner = successes[0][0]
    assert transport.publish("topic", b"payload", winner, 1, wall_time=1.0)
    assert delivered == [winner]


def test_callbacks_on_two_publish_threads_may_close_each_other_without_deadlock():
    transport = LocalTransport()
    callbacks_ready = Barrier(2)
    subscriptions: dict[str, object] = {}
    publish_results: list[bool] = []

    def callback_a(_payload: bytes, _received_at: float) -> None:
        callbacks_ready.wait(timeout=2.0)
        subscriptions["b"].close()

    def callback_b(_payload: bytes, _received_at: float) -> None:
        callbacks_ready.wait(timeout=2.0)
        subscriptions["a"].close()

    subscriptions["a"] = transport.subscribe("topic/a", "TypeA", callback_a)
    subscriptions["b"] = transport.subscribe("topic/b", "TypeB", callback_b)
    publishers = (
        Thread(
            target=lambda: publish_results.append(
                transport.publish("topic/a", b"a", "TypeA", 1, wall_time=1.0)
            ),
            daemon=True,
        ),
        Thread(
            target=lambda: publish_results.append(
                transport.publish("topic/b", b"b", "TypeB", 2, wall_time=1.0)
            ),
            daemon=True,
        ),
    )

    for publisher in publishers:
        publisher.start()
    for publisher in publishers:
        publisher.join(timeout=2.0)

    assert not any(publisher.is_alive() for publisher in publishers)
    assert publish_results == [True, True]
    assert transport.snapshot().received_count == 2


def test_callback_transport_close_does_not_wait_on_other_callback_or_start_late_snapshot_delivery():
    transport = LocalTransport()
    blocking_started = Event()
    release_blocking = Event()
    callback_close_returned = Event()
    late_callback_started = Event()
    publish_results: list[bool] = []

    def blocking_callback(_payload: bytes, _received_at: float) -> None:
        blocking_started.set()
        assert release_blocking.wait(timeout=5.0)

    def closing_callback(_payload: bytes, _received_at: float) -> None:
        transport.close()
        callback_close_returned.set()

    transport.subscribe("blocking", "Blocking", blocking_callback)
    transport.subscribe("closing", "Closing", closing_callback)
    transport.subscribe(
        "closing",
        "Closing",
        lambda _payload, _received_at: late_callback_started.set(),
    )
    blocking_publish = Thread(
        target=lambda: publish_results.append(
            transport.publish("blocking", b"block", "Blocking", 1, wall_time=1.0)
        ),
        daemon=True,
    )
    closing_publish = Thread(
        target=lambda: publish_results.append(
            transport.publish("closing", b"close", "Closing", 2, wall_time=1.0)
        ),
        daemon=True,
    )

    blocking_publish.start()
    assert blocking_started.wait(timeout=2.0)
    closing_publish.start()
    close_returned_without_waiting = callback_close_returned.wait(timeout=1.0)
    try:
        assert close_returned_without_waiting
        closing_publish.join(timeout=1.0)
        assert not closing_publish.is_alive()
        assert not late_callback_started.is_set()
    finally:
        release_blocking.set()
        blocking_publish.join(timeout=2.0)
        closing_publish.join(timeout=2.0)

    assert not blocking_publish.is_alive()
    assert not closing_publish.is_alive()
    assert sorted(publish_results) == [True, True]
    assert transport.snapshot().received_count == 2


def test_concurrent_external_close_waits_for_the_same_lifecycle_barrier():
    transport = LocalTransport()
    callback_started = Event()
    release_callback = Event()
    first_close_returned = Event()
    second_close_entered = Event()
    second_close_returned = Event()

    def blocking_callback(_payload: bytes, _received_at: float) -> None:
        callback_started.set()
        assert release_callback.wait(timeout=5.0)

    def first_close() -> None:
        transport.close()
        first_close_returned.set()

    def second_close() -> None:
        second_close_entered.set()
        transport.close()
        second_close_returned.set()

    transport.subscribe("topic", "TypeA", blocking_callback)
    publisher = Thread(
        target=lambda: transport.publish("topic", b"payload", "TypeA", 1, wall_time=1.0),
        daemon=True,
    )
    first_closer = Thread(target=first_close, daemon=True)
    second_closer = Thread(target=second_close, daemon=True)

    publisher.start()
    assert callback_started.wait(timeout=2.0)
    first_closer.start()

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            probe = transport.subscribe("probe", "Probe", lambda _payload, _received_at: None)
        except RuntimeError:
            break
        probe.close()
    else:
        pytest.fail("first close did not close the transport")

    second_closer.start()
    assert second_close_entered.wait(timeout=2.0)
    try:
        assert not first_close_returned.is_set()
        assert not second_close_returned.wait(timeout=0.2)
    finally:
        release_callback.set()
        publisher.join(timeout=2.0)
        first_closer.join(timeout=2.0)
        second_closer.join(timeout=2.0)

    assert not publisher.is_alive()
    assert not first_closer.is_alive()
    assert not second_closer.is_alive()
    assert first_close_returned.is_set()
    assert second_close_returned.is_set()


def test_external_quiesce_waits_for_started_callback_and_is_idempotent():
    transport = LocalTransport()
    callback_started = Event()
    release_callback = Event()
    quiesce_returned = Event()
    snapshots: list[TransportSnapshot] = []

    def blocking_callback(_payload: bytes, _received_at: float) -> None:
        callback_started.set()
        assert release_callback.wait(timeout=5.0)

    transport.subscribe("topic", "TypeA", blocking_callback)
    publisher = Thread(
        target=lambda: transport.publish(
            "topic", b"payload", "TypeA", 1, wall_time=1.0
        ),
        daemon=True,
    )

    def quiesce_transport() -> None:
        snapshots.append(transport.quiesce())
        quiesce_returned.set()

    quiescer = Thread(target=quiesce_transport, daemon=True)
    publisher.start()
    assert callback_started.wait(timeout=2.0)
    quiescer.start()
    try:
        assert not quiesce_returned.wait(timeout=0.2)
        with pytest.raises(RuntimeError, match="closed"):
            transport.publish("topic", b"late", "TypeA", 2, wall_time=2.0)
        with pytest.raises(RuntimeError, match="closed"):
            transport.subscribe("late", "TypeB", lambda *_args: None)
    finally:
        release_callback.set()
        publisher.join(timeout=2.0)
        quiescer.join(timeout=2.0)

    assert not publisher.is_alive() and not quiescer.is_alive()
    assert quiesce_returned.is_set()
    assert snapshots == [transport.snapshot()]
    assert transport.quiesce() == snapshots[0]
    transport.close()
    transport.close()


def test_callback_on_other_transport_quiesce_waits_for_its_started_callback():
    transport_a = LocalTransport()
    transport_b = LocalTransport()
    callback_b_started = Event()
    release_callback_b = Event()
    cross_quiesce_started = Event()
    cross_quiesce_returned = Event()
    publish_errors: list[BaseException] = []

    def callback_b(_payload: bytes, _received_at: float) -> None:
        callback_b_started.set()
        if not release_callback_b.wait(timeout=5.0):
            raise TimeoutError("transport B callback was not released")

    def callback_a(_payload: bytes, _received_at: float) -> None:
        cross_quiesce_started.set()
        transport_b.quiesce()
        cross_quiesce_returned.set()

    transport_b.subscribe("topic/b", "TypeB", callback_b)
    transport_a.subscribe("topic/a", "TypeA", callback_a)

    def publish(transport, topic: str, type_name: str) -> None:
        try:
            transport.publish(topic, b"payload", type_name, 1, wall_time=1.0)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            publish_errors.append(exc)

    publisher_b = Thread(
        target=publish,
        args=(transport_b, "topic/b", "TypeB"),
        daemon=True,
    )
    publisher_a = Thread(
        target=publish,
        args=(transport_a, "topic/a", "TypeA"),
        daemon=True,
    )
    publisher_b.start()
    assert callback_b_started.wait(timeout=2.0)
    publisher_a.start()
    assert cross_quiesce_started.wait(timeout=2.0)
    returned_while_b_was_blocked = cross_quiesce_returned.wait(timeout=0.2)

    release_callback_b.set()
    publisher_b.join(timeout=2.0)
    publisher_a.join(timeout=2.0)
    transport_a.close()
    transport_b.close()

    assert not returned_while_b_was_blocked
    assert cross_quiesce_returned.is_set()
    assert not publisher_a.is_alive() and not publisher_b.is_alive()
    assert publish_errors == []


def test_callback_quiesce_returns_without_waiting_and_skips_late_snapshot_delivery():
    transport = LocalTransport()
    callback_quiesced = Event()
    late_callback_started = Event()
    snapshots: list[TransportSnapshot] = []

    def quiesce_in_callback(_payload: bytes, _received_at: float) -> None:
        snapshots.append(transport.quiesce())
        callback_quiesced.set()

    transport.subscribe("topic", "TypeA", quiesce_in_callback)
    transport.subscribe(
        "topic",
        "TypeA",
        lambda _payload, _received_at: late_callback_started.set(),
    )

    publisher = Thread(
        target=lambda: transport.publish(
            "topic", b"payload", "TypeA", 1, wall_time=1.0
        ),
        daemon=True,
    )
    publisher.start()
    publisher.join(timeout=1.0)

    assert not publisher.is_alive()
    assert callback_quiesced.is_set()
    assert not late_callback_started.is_set()
    assert len(snapshots) == 1
    assert transport.quiesce() == transport.snapshot()
    transport.close()
