"""Checkout-only proof that the package recipe includes readable agent guides."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile


def main():
    repo = Path(__file__).resolve().parents[2]
    build = repo / "build" / "packaged-guide-proof"
    build.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="run-", dir=build))
    source = run / "source"
    source.mkdir()
    shutil.copytree(repo / "cppyy_kit", source / "cppyy_kit",
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    target = run / "installed"
    # Use the actual recipe installer, but direct pip into this isolated directory.
    # The active Pixi environment is not modified.
    python_shim = run / "python-target"
    python_shim.write_text(
        "#!/bin/bash\nexec %s \"$@\" --target %s\n" %
        (shlex.quote(sys.executable), shlex.quote(str(target))))
    python_shim.chmod(0o700)
    env = dict(os.environ)
    env["CPPYY_KIT_NO_AUTOPCH"] = "1"
    env.update(SRC_DIR=str(source), PYTHON=str(python_shim), PKG_NAME="cppyy-kit",
               PKG_IMPORT="cppyy_kit", PKG_WHERE=".", PKG_VERSION="0.4.0")
    result = subprocess.run(["bash", str(repo / "recipe" / "_build_kit.sh")],
                            env=env, text=True, capture_output=True, timeout=180)
    (run / "install.txt").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise RuntimeError("package installation failed; inspect " + str(run / "install.txt"))
    env["PYTHONPATH"] = str(target)
    check = subprocess.run(
        [sys.executable, "-c",
         "import json,cppyy_kit; from cppyy_kit import guides; "
         "print(json.dumps({'package':cppyy_kit.__file__,"
         "'guides':{k:len(guides.read_guide(k)) for k in guides.TOPICS}}))"],
        cwd=run, env=env, text=True, capture_output=True, timeout=60, check=True)
    data = json.loads(check.stdout)
    assert Path(data["package"]).is_relative_to(target), data
    for topic in data["guides"]:
        cli = subprocess.run([sys.executable, "-m", "cppyy_kit", "guide", topic],
                             cwd=run, env=env, text=True, capture_output=True,
                             timeout=60, check=True)
        assert cli.stdout.startswith("# ") and "Agent prompt:" in cli.stdout
    data["result"] = "installed resources and CLI passed outside checkout"
    (run / "result.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(data, indent=2))


if __name__ == "__main__":
    main()
