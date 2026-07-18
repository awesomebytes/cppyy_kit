#!/usr/bin/env python3
"""Isolated semantic and native-retention proof for message sequences."""

import ctypes
import gc
import json
from pathlib import Path

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue

from rclcpp_kit.bringup_rclcpp import _resolve_message_type
from rclcpp_kit.bringup_rclcpp import convert_python_msg_to_cpp


WARMUP_CONVERSIONS = 1_000
MEASURED_CONVERSIONS = 10_000
MAX_RETAINED_GROWTH_BYTES = 512 * 1024


class Mallinfo2(ctypes.Structure):
    _fields_ = [
        (name, ctypes.c_size_t)
        for name in (
            "arena",
            "ordblks",
            "smblks",
            "hblks",
            "hblkhd",
            "usmblks",
            "fsmblks",
            "uordblks",
            "fordblks",
            "keepcost",
        )
    ]


def parameter(name, parameter_type, **values):
    return Parameter(
        name=name,
        value=ParameterValue(type=parameter_type, **values),
    )


def convert(message, cpp_type):
    return convert_python_msg_to_cpp(message, cpp_type())


def uint8_value(value):
    return ord(value) if isinstance(value, str) else int(value)


def assert_semantics(cpp_type):
    empty = convert(ParameterEvent(node="/empty"), cpp_type)
    assert str(empty.node) == "/empty"
    assert len(empty.new_parameters) == 0
    assert len(empty.changed_parameters) == 0
    assert len(empty.deleted_parameters) == 0

    single = convert(
        ParameterEvent(
            node="/single",
            new_parameters=[parameter(
                "enabled",
                ParameterType.PARAMETER_BOOL,
                bool_value=True,
            )],
        ),
        cpp_type,
    )
    assert len(single.new_parameters) == 1
    assert str(single.new_parameters[0].name) == "enabled"
    assert uint8_value(
        single.new_parameters[0].value.type
    ) == ParameterType.PARAMETER_BOOL
    assert bool(single.new_parameters[0].value.bool_value) is True

    multiple = convert(
        ParameterEvent(
            node="/multiple",
            new_parameters=[
                parameter(
                    "count",
                    ParameterType.PARAMETER_INTEGER,
                    integer_value=42,
                ),
                parameter(
                    "label",
                    ParameterType.PARAMETER_STRING,
                    string_value="converted",
                ),
            ],
            changed_parameters=[parameter(
                "gain",
                ParameterType.PARAMETER_DOUBLE,
                double_value=1.25,
            )],
            deleted_parameters=[parameter(
                "obsolete",
                ParameterType.PARAMETER_NOT_SET,
            )],
        ),
        cpp_type,
    )
    assert [str(item.name) for item in multiple.new_parameters] == [
        "count", "label",
    ]
    assert int(multiple.new_parameters[0].value.integer_value) == 42
    assert str(multiple.new_parameters[1].value.string_value) == "converted"
    assert len(multiple.changed_parameters) == 1
    assert float(multiple.changed_parameters[0].value.double_value) == 1.25
    assert len(multiple.deleted_parameters) == 1
    assert str(multiple.deleted_parameters[0].name) == "obsolete"


def resident_kib():
    for line in Path("/proc/self/status").read_text(encoding="utf-8").splitlines():
        if line.startswith("VmRSS:"):
            return int(line.split()[1])
    raise RuntimeError("/proc/self/status did not report VmRSS")


def retained_bytes(libc):
    info = libc.mallinfo2()
    return int(info.uordblks + info.hblkhd)


def assert_retention(cpp_type):
    libc = ctypes.CDLL(None)
    try:
        mallinfo2 = libc.mallinfo2
        malloc_trim = libc.malloc_trim
    except AttributeError as exc:
        raise RuntimeError("glibc mallinfo2 and malloc_trim are required") from exc
    mallinfo2.restype = Mallinfo2
    malloc_trim.argtypes = [ctypes.c_size_t]
    malloc_trim.restype = ctypes.c_int

    message = ParameterEvent(new_parameters=[parameter(
        "use_sim_time",
        ParameterType.PARAMETER_BOOL,
        bool_value=False,
    )])
    for _ in range(WARMUP_CONVERSIONS):
        convert(message, cpp_type)
    gc.collect()
    malloc_trim(0)
    before = retained_bytes(libc)
    rss_before = resident_kib()

    for _ in range(MEASURED_CONVERSIONS):
        convert(message, cpp_type)
    gc.collect()
    malloc_trim(0)
    after = retained_bytes(libc)
    rss_after = resident_kib()
    growth = max(0, after - before)
    assert growth <= MAX_RETAINED_GROWTH_BYTES, (growth, before, after)
    print("MESSAGE_SEQUENCE_RETENTION " + json.dumps({
        "allocator_growth_bytes": growth,
        "max_allocator_growth_bytes": MAX_RETAINED_GROWTH_BYTES,
        "measured_conversions": MEASURED_CONVERSIONS,
        "rss_growth_kib": max(0, rss_after - rss_before),
        "warmup_conversions": WARMUP_CONVERSIONS,
    }, sort_keys=True), flush=True)


def main():
    _, cpp_type = _resolve_message_type(ParameterEvent)
    assert_semantics(cpp_type)
    print("MESSAGE_SEQUENCE_SEMANTICS_OK", flush=True)
    assert_retention(cpp_type)
    print("MESSAGE_SEQUENCE_RETENTION_OK", flush=True)


if __name__ == "__main__":
    main()
