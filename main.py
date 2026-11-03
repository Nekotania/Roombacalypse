#!/usr/bin/env python3
"""ROOMBACALYPSE launcher.

Equivalent to ``python -m roombacalypse`` or the installed ``roombacalypse``
command. Run ``python main.py --help`` for options.
"""

from roombacalypse.app import main

if __name__ == "__main__":
    raise SystemExit(main())
