import numpy as np
from index import Index

points = np.array([[0., 0., 0.], [0.1, 0., 0.], [0.2, 0., 0.], [0.3, 0., 0.]])
with Index(points, [0, 1, 20, 30], [1., .9, .8, .1]) as index:
    result = index.query([[0., 0., 0.]], [0], k=2, frame_gap=2,
                         min_confidence=.5, max_distance=.5)
    print("indices:", result["ids"].tolist())
    print("squared distances (m^2):", result["distances_squared"].tolist())
    print("distances (m):", np.sqrt(result["distances_squared"]).tolist())
    print("selected counts:", result["counts"].tolist())
    print("selected centroid (m):", result["centroids"].tolist())
