"""Run the same scalar reduction in Python."""
from measure import run


def sum_sq(x):
    total = 0.0
    for value in x:
        total += value * value
    return total


if __name__ == '__main__':
    run(sum_sq)
