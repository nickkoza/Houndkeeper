"""The close look measures the whole toy's box, not a part's (look-closer [c1X])."""
import sys
from pathlib import Path
from types import SimpleNamespace as D

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_a_part_gives_way_to_the_toy_it_is_part_of():
    import wrist_refine as wr
    piece = D(bbox=(100, 100, 140, 130), confidence=0.4)                     # a puzzle's piece, chosen near the flag
    board = D(bbox=(60, 70, 300, 260), confidence=0.2)                       # the board around it
    other = D(bbox=(400, 300, 500, 400), confidence=0.5)                     # something else in view
    assert wr.whole_box(piece, [piece, board, other]) is board


def test_a_box_barely_bigger_or_beside_it_is_not_the_whole():
    import wrist_refine as wr
    shoe = D(bbox=(100, 100, 300, 180), confidence=0.5)
    loose = D(bbox=(95, 95, 310, 190), confidence=0.3)                       # the same shoe, boxed a little loosely
    beside = D(bbox=(250, 100, 600, 400), confidence=0.3)                    # overlaps a corner of it only
    assert wr.whole_box(shoe, [shoe, loose, beside]) is shoe
