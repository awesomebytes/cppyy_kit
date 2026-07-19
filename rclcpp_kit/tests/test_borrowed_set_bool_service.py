import inspect

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.borrowed_set_bool_service import (
    BorrowedSetBoolRequestRef,
    BorrowedSetBoolResponseRef,
    _BorrowedScope,
    _make_dispatch,
    create_borrowed_set_bool_service,
)


class _FakeInvocation:
    def __init__(self):
        self.success = False
        self.message = ""

    def request_data(self):
        return True

    def response_success(self):
        return self.success

    def set_response_success(self, value):
        self.success = value

    def response_message(self):
        return self.message

    def set_response_message(self, value):
        self.message = value


def test_noncallable_callback_is_rejected_before_bringup():
    with pytest.raises(TypeError, match="callable"):
        create_borrowed_set_bool_service(None, None, "service", None)


def test_coroutine_callback_is_rejected_before_bringup():
    async def callback(_request, _response):
        return None

    with pytest.raises(TypeError, match="synchronous"):
        create_borrowed_set_bool_service(None, None, "service", callback)


def test_generated_bridge_has_no_owning_message_copy_or_response_commit():
    source = inspect.getsource(create_borrowed_set_bool_service)
    assert "invocation(*request, *response)" in source
    assert "request_cpp_copies() const override { return 0; }" in source
    assert "response_cpp_copies() const override { return 0; }" in source
    assert "owning_request" not in source
    assert "owning_response" not in source
    assert "commit_response" not in source
    assert "*response =" not in source


def test_guarded_views_mutate_in_place_and_expire():
    expired = [0]
    invocation = _FakeInvocation()
    scope = _BorrowedScope(invocation, expired)
    request = BorrowedSetBoolRequestRef(scope)
    response = BorrowedSetBoolResponseRef(scope)

    assert request.valid and response.valid
    assert request.data is True
    response.success = True
    response.message = "in-place"
    assert invocation.success is True
    assert invocation.message == "in-place"
    with pytest.raises(AttributeError):
        request.data = False
    with pytest.raises(TypeError, match="bool"):
        response.success = 1
    with pytest.raises(TypeError, match="str"):
        response.message = b"bytes"

    scope.expire()
    assert not request.valid and not response.valid
    with pytest.raises(RuntimeError, match="expired"):
        _ = request.data
    with pytest.raises(RuntimeError, match="expired"):
        response.message = "too-late"
    assert expired == [2]


def test_dispatch_rejects_replacement_response_and_expires_retained_views():
    retained = []
    rejected = [0]
    expired = [0]

    def callback(request, response):
        retained.extend((request, response))
        response.success = request.data
        return response

    dispatch = _make_dispatch(callback, rejected, expired)
    with pytest.raises(TypeError, match="return None"):
        dispatch(_FakeInvocation())
    assert rejected == [1]
    assert not retained[0].valid and not retained[1].valid
    with pytest.raises(RuntimeError, match="expired"):
        _ = retained[0].data
    assert expired == [1]


def test_dispatch_rejects_runtime_awaitable_and_closes_it():
    rejected = [0]
    expired = [0]

    async def result():
        return None

    awaitable = result()

    def callback(_request, _response):
        return awaitable

    with pytest.raises(TypeError, match="awaitables"):
        _make_dispatch(callback, rejected, expired)(_FakeInvocation())
    assert rejected == [1]
    assert inspect.getcoroutinestate(awaitable) == inspect.CORO_CLOSED


def test_borrowed_set_bool_service_interoperates_with_stock_client():
    proc = run_helper("_borrowed_set_bool_service_helper.py", timeout=300)
    assert proc.returncode == 0, format_output(proc)
    assert "BORROWED_SET_BOOL_STOCK_OK" in proc.stdout
    assert "BORROWED_SET_BOOL_MISUSE_OK" in proc.stdout
    assert "BORROWED_SET_BOOL_TEARDOWN_OK" in proc.stdout
