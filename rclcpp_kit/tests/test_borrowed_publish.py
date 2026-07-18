"""Contract tests for publishing through an authoritative stock handle."""

from concurrent.futures import ThreadPoolExecutor

from _run_helper import format_output, run_helper

from rclcpp_kit import borrowed_publish


def test_prepare_caches_cppyy_binding_but_returns_independent_routes(monkeypatch):
    message_type = type("Message", (), {})
    calls = []

    class Binding:
        cpp_type_name = "example::msg::Message"
        cpp_message_type = object()
        scratch_type = object()

        def __init__(self, requested_type):
            calls.append(requested_type)

    monkeypatch.setattr(borrowed_publish, "_PublisherBinding", Binding)
    monkeypatch.setattr(borrowed_publish, "_BINDINGS", {})

    with ThreadPoolExecutor(max_workers=8) as executor:
        routes = list(executor.map(borrowed_publish.prepare, [message_type] * 32))

    assert calls == [message_type]
    assert len({id(route) for route in routes}) == len(routes)
    assert all(route.message_type is message_type for route in routes)
    assert all(route._scratch_type is routes[0]._scratch_type for route in routes)
    assert len({id(route._scratch_local) for route in routes}) == len(routes)


def test_failed_binding_is_not_cached(monkeypatch):
    message_type = type("Message", (), {})
    calls = []

    class Binding:
        def __init__(self, requested_type):
            calls.append(requested_type)
            if len(calls) == 1:
                raise RuntimeError("binding failed")
            self.cpp_type_name = "example::msg::Message"
            self.cpp_message_type = object()
            self.scratch_type = object()

    monkeypatch.setattr(borrowed_publish, "_PublisherBinding", Binding)
    monkeypatch.setattr(borrowed_publish, "_BINDINGS", {})

    try:
        borrowed_publish.prepare(message_type)
    except RuntimeError as exception:
        assert str(exception) == "binding failed"
    else:
        raise AssertionError("failed binding was accepted")

    route = borrowed_publish.prepare(message_type)
    assert route.message_type is message_type
    assert calls == [message_type, message_type]


def test_publish_glue_checks_rcl_return_and_resets_error():
    source = borrowed_publish._PUBLISH_GLUE
    assert "class PublishScratch" in source
    assert "rclcpp::Serialization<MessageT> serializer_;" in source
    assert "rclcpp::SerializedMessage serialized_;" in source
    assert "rcl_publish_serialized_message(" in source
    serialization = source.index("serializer_.serialize_message(")
    publish = source.index("rcl_publish_serialized_message(")
    assert "catch (...)" in source[serialization:publish]
    assert "raw.buffer_length = 0;" in source[serialization:publish]
    assert source[serialization:publish].count("rcl_reset_error();") >= 2
    assert "result != RCL_RET_OK" in source
    assert "rcl_get_error_string" in source
    assert "rcl_reset_error" in source
    failure = source.index("result != RCL_RET_OK")
    assert "raw.buffer_length = 0;" in source[failure:]


def test_default_retained_capacity_ceiling_is_eight_mib():
    assert borrowed_publish._MAX_RETAINED_SERIALIZED_CAPACITY == 8 * 1024 * 1024


def test_same_handle_roundtrip_identity_and_teardown():
    proc = run_helper("_borrowed_publish_helper.py")
    details = format_output(proc)
    assert "BORROWED_PUBLISH_ROUNDTRIP_OK" in proc.stdout, details
    assert "BORROWED_PUBLISH_TEARDOWN_OK" in proc.stdout, details
    assert proc.returncode == 0, details


def test_reusable_publish_scratch_contracts():
    proc = run_helper("_borrowed_publish_scratch_helper.py")
    details = format_output(proc)
    assert "BORROWED_PUBLISH_SCRATCH_REUSE_OK" in proc.stdout, details
    assert "BORROWED_PUBLISH_SCRATCH_EVICTION_OK" in proc.stdout, details
    assert "BORROWED_PUBLISH_SCRATCH_RECOVERY_OK" in proc.stdout, details
    assert "BORROWED_PUBLISH_SCRATCH_THREADS_OK" in proc.stdout, details
    assert proc.returncode == 0, details


def test_fastdds_success_does_not_leave_stale_rcl_error(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    proc = run_helper("_borrowed_publish_error_hygiene_helper.py")
    details = format_output(proc)
    assert proc.returncode == 0, details
    assert "BORROWED_PUBLISH_ERROR_HYGIENE_OK" in proc.stdout, details
    stderr = proc.stderr.lower()
    assert "rcutils_set_error_state" not in stderr, details
    assert "error state is being overwritten" not in stderr, details
    assert "typesupport identifier" not in stderr, details
