import os
import sys

# Ensure src on path when running from repo root
ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, 'src')
if SRC not in sys.path:
    sys.path.insert(0, SRC)

from gravtest.cli import main

if __name__ == '__main__':
    raise SystemExit(main())
