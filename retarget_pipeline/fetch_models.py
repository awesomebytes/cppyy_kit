"""
fetch_models -- download the MediaPipe Tasks ``.task`` model bundles once.

MediaPipe 0.10.x dropped the legacy ``mp.solutions`` API (whose models shipped in
the wheel); the Tasks API needs the model bundle downloaded separately. This grabs
the official Google-hosted bundles into a gitignored cache under ``build/pipeline/
models/`` (same lifecycle as the dataset downloaders in ``scripts/datasets/``).
Idempotent: an already-present, non-empty file is left alone unless ``--force``.

The perception demo calls :func:`model_path` and falls back to the synthetic scene
if a model is absent and cannot be fetched (offline), so this is a convenience, not
a hard dependency.
"""
import argparse
import os
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
MODELS_DIR = os.path.join(REPO, "build", "pipeline", "models")

# Official MediaPipe model bundles (storage.googleapis.com). float16 variants.
_BASE = "https://storage.googleapis.com/mediapipe-models"
MODELS = {
    "holistic": (_BASE + "/holistic_landmarker/holistic_landmarker/float16/latest/"
                 "holistic_landmarker.task", "holistic_landmarker.task"),
    "pose": (_BASE + "/pose_landmarker/pose_landmarker_full/float16/latest/"
             "pose_landmarker_full.task", "pose_landmarker_full.task"),
    "hand": (_BASE + "/hand_landmarker/hand_landmarker/float16/latest/"
             "hand_landmarker.task", "hand_landmarker.task"),
}


def model_path(name):
    """Absolute path where model ``name`` lives (whether or not it is present)."""
    if name not in MODELS:
        raise KeyError("unknown model %r (have: %s)" % (name, ", ".join(MODELS)))
    return os.path.join(MODELS_DIR, MODELS[name][1])


def have_model(name):
    p = model_path(name)
    return os.path.isfile(p) and os.path.getsize(p) > 0


def fetch(name, force=False, quiet=False):
    """Download model ``name`` to the cache if absent. Returns its path. Raises on a
    download failure (caller may catch and fall back to synthetic)."""
    url, fname = MODELS[name]
    dest = os.path.join(MODELS_DIR, fname)
    if not force and have_model(name):
        return dest
    os.makedirs(MODELS_DIR, exist_ok=True)
    tmp = dest + ".part"
    if not quiet:
        print("[fetch_models] downloading %s -> %s" % (name, dest), flush=True)
    try:
        urllib.request.urlretrieve(url, tmp)
        if os.path.getsize(tmp) <= 0:
            raise IOError("downloaded 0 bytes")
        os.replace(tmp, dest)
    except Exception:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
    if not quiet:
        print("[fetch_models] %s: %d bytes" % (name, os.path.getsize(dest)), flush=True)
    return dest


def ensure(name, quiet=False):
    """Return the model path, fetching it if absent; ``None`` if it cannot be
    obtained (offline and not cached) -- the demo then uses the synthetic scene."""
    try:
        return fetch(name, quiet=quiet)
    except Exception as exc:  # offline / URL change -> synthetic fallback
        if not quiet:
            print("[fetch_models] could not obtain %r (%s); synthetic fallback."
                  % (name, exc), flush=True)
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("models", nargs="*", default=["holistic"],
                    help="which models to fetch (default: holistic)")
    ap.add_argument("--force", action="store_true", help="re-download even if cached")
    ap.add_argument("--all", action="store_true", help="fetch all known models")
    args = ap.parse_args(argv)
    names = list(MODELS) if args.all else (args.models or ["holistic"])
    rc = 0
    for name in names:
        try:
            fetch(name, force=args.force)
        except Exception as exc:
            print("[fetch_models] FAILED %s: %s" % (name, exc), file=sys.stderr)
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
