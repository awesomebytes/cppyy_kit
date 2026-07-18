"""Keep third-party workflow execution bound to reviewed commits."""

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]
USES_RE = re.compile(r"^\s*uses:\s+([^@\s]+)@([^\s#]+)", re.MULTILINE)
COMMIT_RE = re.compile(r"[0-9a-f]{40}")


def test_all_external_actions_use_immutable_commit_pins():
    observed = []
    for workflow in sorted((ROOT / ".github/workflows").glob("*.yml")):
        for action, revision in USES_RE.findall(workflow.read_text()):
            observed.append((workflow.name, action, revision))
            assert COMMIT_RE.fullmatch(revision), (
                workflow.name, action, revision)

    assert observed
