"""Explicit local guide discovery. No native loading or installation."""
from pathlib import Path

print(Path(__file__).with_name("README.md").read_text(encoding="utf-8"))
