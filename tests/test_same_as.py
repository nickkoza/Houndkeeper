"""[XfI] the same object is never handled twice in a night, and an object is never "the same as" itself."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import pick


def test_second_flag_on_a_handled_object_is_the_same():
    assert pick.same_as([1.0, 1.05], [(3, np.array([1.0, 1.0]))]) == 3


def test_far_object_is_new():
    assert pick.same_as([1.0, 1.3], [(3, np.array([1.0, 1.0]))]) is None


def test_a_relook_at_the_same_item_is_not_a_repeat():   # re-parked after "out of reach": item 5 seen again (25 Sep night)
    assert pick.same_as([1.0, 1.02], [(5, np.array([1.0, 1.0]))], this=5) is None
