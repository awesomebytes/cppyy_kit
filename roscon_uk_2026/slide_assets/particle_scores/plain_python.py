"""Range scores using Python lists and the standard library only."""
from math import hypot


def score_particles(positions, landmarks, ranges):
    observations = tuple(zip(landmarks, ranges))
    scores = []
    for x, y in positions:
        error = 0.0
        for (lx, ly), distance in observations:
            residual = hypot(x - lx, y - ly) - distance
            error += residual * residual
        scores.append(-error)
    return scores
