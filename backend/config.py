from pathlib import Path

# Absolute path to the repository root (parent of this backend/ directory).
# All scripts expect to be invoked with CWD = REPO_ROOT.
REPO_ROOT: Path = Path(__file__).resolve().parent.parent
