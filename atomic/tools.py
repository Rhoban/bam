import numpy as np
import sys


def filename_argument() -> str:
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} filename.json")
        sys.exit(1)

    return sys.argv[1]


def linear_regression(x, y) -> float:
    A = np.array([x]).T
    return np.linalg.lstsq(A, y, rcond=None)[0]
