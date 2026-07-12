"""
fetch_models -- download the MediaPipe Tasks ``.task`` model bundles once, pinned.

MediaPipe 0.10.x dropped the legacy ``mp.solutions`` API (whose models shipped in
the wheel); the Tasks API needs the model bundle downloaded separately. This grabs
the official Google-hosted bundles into a gitignored cache under ``build/pipeline/
models/`` (same lifecycle as the dataset downloaders in ``scripts/datasets/``).

**Pinned + checksum-verified.** Each bundle's URL and its SHA-256 are pinned below
(the exact bytes fetched on 2026-07-12); a download whose hash does not match is
rejected and the partial file removed -- supply-chain hygiene for anything fetched
at runtime. Idempotent: an already-present bundle whose hash matches is left alone.

Caveat: the URLs point at Google's ``.../latest/`` path, so if Google rotates a
bundle its hash will change and the check will (correctly) refuse it. Re-pin the new
SHA-256 here, or pass ``--allow-hash-mismatch`` / ``M6F_ALLOW_HASH_MISMATCH=1`` to
knowingly accept the new bundle.

The perception demo calls :func:`ensure` and falls back to the synthetic scene if a
model is absent and cannot be fetched (offline), so this is a convenience, not a hard
dependency.
"""
import argparse
import hashlib
import os
import sys
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
MODELS_DIR = os.path.join(REPO, "build", "pipeline", "models")

# Official MediaPipe model bundles (storage.googleapis.com), float16 "latest".
# (url, filename, sha256) -- sha256 verified after download; mismatch -> refuse.
_BASE = "https://storage.googleapis.com/mediapipe-models"
MODELS = {
    "holistic": (
        _BASE + "/holistic_landmarker/holistic_landmarker/float16/latest/"
        "holistic_landmarker.task", "holistic_landmarker.task",
        "e2dab61191e2dcd0a15f943d8e3ed1dce13c82dfa597b9dd39f562975a50c3f8"),
    "pose": (
        _BASE + "/pose_landmarker/pose_landmarker_full/float16/latest/"
        "pose_landmarker_full.task", "pose_landmarker_full.task",
        "4eaa5eb7a98365221087693fcc286334cf0858e2eb6e15b506aa4a7ecdcec4ad"),
    "hand": (
        _BASE + "/hand_landmarker/hand_landmarker/float16/latest/"
        "hand_landmarker.task", "hand_landmarker.task",
        "fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1"),
}


def model_path(name):
    """Absolute path where model ``name`` lives (whether or not it is present)."""
    if name not in MODELS:
        raise KeyError("unknown model %r (have: %s)" % (name, ", ".join(MODELS)))
    return os.path.join(MODELS_DIR, MODELS[name][1])


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _allow_mismatch(flag):
    return flag or os.environ.get("M6F_ALLOW_HASH_MISMATCH") == "1"


def have_model(name):
    """True iff the bundle is present AND its SHA-256 matches the pin (a stale/
    corrupt/rotated cached file is treated as absent so it gets re-fetched)."""
    p = model_path(name)
    if not (os.path.isfile(p) and os.path.getsize(p) > 0):
        return False
    return _sha256(p) == MODELS[name][2]


def fetch(name, force=False, quiet=False, allow_mismatch=False):
    """Download model ``name`` to the cache if absent, verifying its SHA-256.
    Returns its path. Raises on a download failure or (unless allowed) a hash
    mismatch -- callers may catch and fall back to synthetic."""
    url, fname, want = MODELS[name]
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
        got = _sha256(tmp)
        if got != want and not _allow_mismatch(allow_mismatch):
            raise IOError(
                "SHA-256 mismatch for %s:\n  expected %s\n  got      %s\n"
                "The pinned bundle may have been rotated at the 'latest' URL; re-pin "
                "in fetch_models.py or pass --allow-hash-mismatch." % (name, want, got))
        os.replace(tmp, dest)
        if not quiet:
            note = "" if got == want else " (HASH MISMATCH ALLOWED)"
            print("[fetch_models] %s: %d bytes, sha256 %s%s"
                  % (name, os.path.getsize(dest), got[:16] + "...", note), flush=True)
    except Exception:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise
    return dest


def ensure(name, quiet=False):
    """Return the model path, fetching+verifying it if absent; ``None`` if it cannot
    be obtained (offline / hash mismatch) -- the demo then uses the synthetic scene."""
    try:
        return fetch(name, quiet=quiet)
    except Exception as exc:  # offline / URL change / hash mismatch -> synthetic
        if not quiet:
            print("[fetch_models] could not obtain %r (%s); synthetic fallback."
                  % (name, exc), flush=True)
        return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("models", nargs="*", default=["holistic"],
                    help="which models to fetch (default: holistic)")
    ap.add_argument("--force", action="store_true", help="re-download even if cached")
    ap.add_argument("--all", action="store_true", help="fetch all known models")
    ap.add_argument("--allow-hash-mismatch", action="store_true",
                    help="accept a bundle whose SHA-256 differs from the pin "
                         "(the 'latest' URL was rotated -- re-pin afterwards)")
    args = ap.parse_args(argv)
    names = list(MODELS) if args.all else (args.models or ["holistic"])
    rc = 0
    for name in names:
        try:
            fetch(name, force=args.force, allow_mismatch=args.allow_hash_mismatch)
        except Exception as exc:
            print("[fetch_models] FAILED %s: %s" % (name, exc), file=sys.stderr)
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
