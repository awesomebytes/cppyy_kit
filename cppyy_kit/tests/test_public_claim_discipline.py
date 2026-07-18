"""Keep shared-host rclcpp measurements bounded as characterization."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _section(document: str, heading: str) -> str:
    start = document.index(heading)
    end = document.find("\n## ", start + len(heading))
    return document[start:] if end < 0 else document[start:end]


def _normalized(document: str) -> str:
    return " ".join(document.split())


def test_rclcpp_public_docs_do_not_promote_shared_host_tf_results():
    report = (ROOT / "rclcpp_kit" / "REPORT.md").read_text(encoding="utf-8")
    why = (ROOT / "rclcpp_kit" / "WHY.md").read_text(encoding="utf-8")
    root_readme = (ROOT / "README.md").read_text(encoding="utf-8")
    benchmark = (ROOT / "docs" / "benchmarks.md").read_text(encoding="utf-8")
    patterns = (ROOT / "docs" / "COMMON_PATTERNS.md").read_text(encoding="utf-8")
    tf_section = _section(
        benchmark, "## TF ingest — C++ tf2 listener vs Python callback")

    assert "do not establish a portable performance claim" in _normalized(report)
    assert "not a portable performance" in _normalized(why)
    assert "no portable claim" in _normalized(root_readme)
    assert "does not choose a winner" in _normalized(tf_section)
    assert "not a portable performance claim or winner" in _normalized(patterns)

    promoted_phrases = (
        "verdict: confirmed",
        "less ingest cpu",
        "lookups are cheaper",
        "wins decisively",
    )
    public_tf_text = _normalized(
        "\n".join((report, why, tf_section, patterns))).lower()
    assert not any(phrase in public_tf_text for phrase in promoted_phrases)


def test_rclcpp_pch_numbers_are_labeled_single_host_characterization():
    root_readme = (ROOT / "README.md").read_text(encoding="utf-8")
    benchmark = (ROOT / "docs" / "benchmarks.md").read_text(encoding="utf-8")
    freeze = (ROOT / "docs" / "FREEZE.md").read_text(encoding="utf-8")
    patterns = (ROOT / "docs" / "COMMON_PATTERNS.md").read_text(encoding="utf-8")
    pch_section = _section(
        benchmark, "## Auto-PCH — zero-config cold vs warm bringup")

    assert "not a portable startup claim" in _normalized(root_readme)
    assert "not a portable startup" in _normalized(pch_section)
    assert "single-host cache characterization" in _normalized(freeze)
    assert "not a portable startup claim or threshold" in _normalized(patterns)
