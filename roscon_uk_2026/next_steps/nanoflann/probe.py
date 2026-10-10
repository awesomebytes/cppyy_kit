"""Run uncertain header/template declaration and first call in this subprocess."""
from index import COMPILE, LIBRARY, Index

with Index([[0., 0., 0.], [1., 0., 0.]], [0, 2], [1., 1.]) as index:
    result = index.query([[0., 0., 0.]], [0], k=1, frame_gap=0)
    assert result["ids"].tolist() == [[1]]
    assert result["distances_squared"].tolist() == [[1.]]
    assert index._native.library_version() == 0x163
    print("native template and first call: PASS", LIBRARY, COMPILE)
