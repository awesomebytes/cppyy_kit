"""Run isolated characterizations of explicit native ``rclcpp`` choices."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from typing import Any

from rclcpp_kit.benchmarks._native_choices_protocol import (
    CASES,
    build_document,
    dumps,
    validate_sample,
    write,
)
from rclcpp_kit.benchmarks._native_choices_worker import SAMPLE_PREFIX


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _defaults(mode: str) -> tuple[int, int, int]:
    return (16, 4, 1) if mode == "smoke" else (2000, 100, 5)


def _domains(base: int, count: int) -> list[int]:
    if not 0 <= base <= 232:
        raise ValueError("domain id must be between 0 and 232")
    if count > 233:
        raise ValueError("matrix requires more unique domains than ROS supports")
    return [(base + offset) % 233 for offset in range(count)]


def _communicate(command: list[str], env: dict[str, str], timeout: float):
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        stdout, stderr = process.communicate(timeout=5)
        raise RuntimeError(
            "worker timed out after %.1fs\nstdout:\n%s\nstderr:\n%s"
            % (timeout, stdout, stderr)
        )
    return process.returncode, stdout, stderr, process.pid


def _extract_sample(stdout: str) -> dict[str, Any]:
    payloads = [
        line[len(SAMPLE_PREFIX):]
        for line in stdout.splitlines()
        if line.startswith(SAMPLE_PREFIX)
    ]
    if len(payloads) != 1:
        raise ValueError("worker must emit exactly one sample payload")
    value = json.loads(payloads[0])
    if not isinstance(value, dict):
        raise ValueError("worker sample payload must be an object")
    return value


def _failure(
    case: dict[str, Any], repetition: int, error: str,
    *, ros_domain_id: int, returncode: int | None = None,
    stdout: str = "", stderr: str = "",
) -> dict[str, Any]:
    return {
        "case_id": case["case_id"],
        "repetition": repetition,
        "ros_domain_id": ros_domain_id,
        "error": str(error),
        "returncode": returncode,
        "stdout": stdout[-12000:],
        "stderr": stderr[-12000:],
    }


def run_matrix(
    *, mode: str, messages: int, warmup_messages: int, repetitions: int,
    base_domain_id: int, worker_timeout: float,
) -> tuple[list[dict], list[dict], list[int]]:
    sample_count = len(CASES) * repetitions
    domains = _domains(base_domain_id, sample_count)
    results = []
    failures = []
    sample_index = 0
    for repetition in range(repetitions):
        for case in CASES:
            domain_id = domains[sample_index]
            sample_index += 1
            command = [
                sys.executable,
                "-m", "rclcpp_kit.benchmarks._native_choices_worker",
                "--case-id", case["case_id"],
                "--repetition", str(repetition),
                "--messages", str(messages),
                "--warmup-messages", str(warmup_messages),
            ]
            environment = os.environ.copy()
            environment["RMW_IMPLEMENTATION"] = case["rmw"]
            environment["ROS_DOMAIN_ID"] = str(domain_id)
            returncode = None
            stdout = ""
            stderr = ""
            worker_pid = None
            try:
                returncode, stdout, stderr, worker_pid = _communicate(
                    command, environment, worker_timeout)
                if returncode != 0:
                    raise RuntimeError("worker exited with code %d" % returncode)
                sample = _extract_sample(stdout)
                validate_sample(sample, messages=messages)
                if sample["repetition"] != repetition:
                    raise ValueError("worker repetition evidence is invalid")
                if sample["ros_domain_id"] != domain_id:
                    raise ValueError("worker ROS domain evidence is invalid")
                if sample["pid"] != worker_pid:
                    raise ValueError("worker process identity evidence is invalid")
                results.append(sample)
            except Exception as exception:
                failures.append(_failure(
                    case,
                    repetition,
                    str(exception),
                    ros_domain_id=domain_id,
                    returncode=returncode,
                    stdout=stdout,
                    stderr=stderr,
                ))
    return results, failures, domains


def parse_args(arguments: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Characterize explicit native rclcpp choices without making "
            "performance claims."
        ))
    parser.add_argument("--mode", choices=("smoke", "measurement"), default="smoke")
    parser.add_argument("--messages", type=int)
    parser.add_argument("--warmup-messages", type=int)
    parser.add_argument("--repetitions", type=int)
    parser.add_argument("--domain-id", type=int)
    parser.add_argument("--worker-timeout", type=float, default=180.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args(arguments)


def main(arguments: list[str] | None = None) -> int:
    args = parse_args(arguments)
    defaults = _defaults(args.mode)
    messages = args.messages if args.messages is not None else defaults[0]
    warmup = (
        args.warmup_messages
        if args.warmup_messages is not None else defaults[1]
    )
    repetitions = args.repetitions if args.repetitions is not None else defaults[2]
    if messages <= 0 or warmup < 0 or repetitions <= 0:
        raise ValueError("messages/repetitions must be positive and warmup non-negative")
    if args.worker_timeout <= 0:
        raise ValueError("worker timeout must be positive")
    domain_value = args.domain_id
    if domain_value is None:
        configured = os.environ.get("ROS_DOMAIN_ID")
        domain_value = int(configured) if configured is not None else os.getpid() % 233
    results, failures, domains = run_matrix(
        mode=args.mode,
        messages=messages,
        warmup_messages=warmup,
        repetitions=repetitions,
        base_domain_id=domain_value,
        worker_timeout=args.worker_timeout,
    )
    document = build_document(
        repo_root=_repo_root(),
        mode=args.mode,
        messages=messages,
        warmup_messages=warmup,
        repetitions=repetitions,
        results=results,
        failures=failures,
        domains=domains,
        command=[sys.executable, "-m", __spec__.name, *(arguments or sys.argv[1:])],
    )
    if args.output is None:
        sys.stdout.write(dumps(document))
    else:
        write(document, args.output)
        print("NATIVE_CHOICES_RESULT=%s" % args.output.resolve())
    if failures:
        print("NATIVE_CHOICES_FAILURES=%d" % len(failures), file=sys.stderr)
        return 1
    print("NATIVE_CHOICES_CASES=%d" % len(results), file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
