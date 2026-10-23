"""Source-repository wrapper for the installed control_self_test command."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from husky_drivable_area.control_self_test import main


if __name__ == "__main__":
    main()
