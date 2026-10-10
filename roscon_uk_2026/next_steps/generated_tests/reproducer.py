"""Replay the saved witness without importing or calling Hypothesis."""

import argparse
import json
from pathlib import Path

import numpy as np
from roscon_uk_2026.next_steps.reverse_core import PoseFilter
from faulty_fixture import MissingResetFilter


def replay(factory, witness):
    instance = factory(witness["config"])
    output = None
    try:
        for operation in witness["operations"]:
            if operation["method"] == "reset":
                instance.reset()
            else:
                output = instance.process(np.array(operation["timestamps_ns"], dtype=np.int64),
                                          np.array(operation["positions_m"], dtype=np.float64))
        return output
    finally:
        instance.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assert-correct", action="store_true",
                        help="raise AssertionError on the known fault (expected exit 1)")
    args = parser.parse_args()
    witness = json.loads(Path(__file__).with_name("minimized_fault.json").read_text())
    expected = np.array(witness["expected_last_output_m"], dtype=np.float64)
    correct = replay(PoseFilter, witness)
    faulty = replay(MissingResetFilter, witness)
    np.testing.assert_allclose(correct, expected, rtol=1e-12, atol=1e-9)
    if args.assert_correct:
        np.testing.assert_allclose(faulty, expected, rtol=1e-12, atol=1e-9)
    else:
        assert not np.allclose(faulty, expected, rtol=1e-12, atol=1e-9)
        print(json.dumps({"correct_native_passes": True, "known_fault_reproduced": True,
                          "expected_m": expected.tolist(), "faulty_m": faulty.tolist()}))


if __name__ == "__main__":
    main()
