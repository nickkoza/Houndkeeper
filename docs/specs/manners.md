# The dog's manners

**Type:** Capability spec · **Status:** Draft · **Source:** a robot living with people · **ID:** [YF0]

## Purpose

The robot lives in a house with people, so how it carries itself is part of what it is, not a side effect of the
task: it has to be lived with. It is given a few deliberate, dog-like gestures that show what it
just did, so the people around it can read it at a glance and enjoy it. They are designed here, together, so they stay
brief, safe and in one character, rather than piling up in the task specs.

## Acceptance criteria

1. [ZjA] Every gesture is short (about a second, never more than two), comes only once a step is finished (never
   while gripping or carrying), moves nothing it shouldn't (an item in the bin stays in; the arm reaches no further
   than the step already did), leaves the robot as it found it, and can be switched off.
2. [N2T] When a look, sitting or standing, finds nothing to pick where the flag was, the arm shakes the gripper side
   to side ("nope, that's not it"), in about a second, before it stows, and comes back to the pose it shook from.
   The gripper moves sideways at the look's height, not twisting in place; it never goes lower than the look did.
3. [Yju] When an item lands in the bin, once the robot is standing again, the whole dog twists side to side a couple
   of times, like a particularly happy dog, and ends facing the way it was.

## Decisions

1. [M7r] **The happy wiggle is a turn in place, commanded through the walking controller.** The body's yaw is
   swung by alternating turn-rate commands, half a swing at each end so the swings cancel; it asks the controller for
   no more than 1.2 rad/s (est.: under what the Go1 walking policy follows; the sim's turns use 0.6). How far the body
   actually swings is the controller's (est. ±10-15°). The bin rides on the robot's back; whether a swing this size
   leaves its contents in is checked on the live nights (binned items are scored from where they end up).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [ZjA] (short, safe, switchable) | `tests/test_nope.py` (the nope: only the shoulder, back where it started, about a second, off switch); `tests/test_happy.py` (the wiggle: swings cancel, within the rate and time limits, off switch) | Partial: rule tests; "nothing leaves the bin" is a live-night check, not yet run |
| [N2T] (nope shake after an empty look) | `tests/test_nope.py` | Partial: sim; watched on a live night, 28 Sep 2026, and read as intended; not covered (test audit, 3 Oct 2026): the trigger (an empty look), before it stows, and the look's height are unchecked |
| [Yju] (happy wiggle after a binned item) | `tests/test_happy.py`; live nights | Partial: rule test only; not yet watched |
