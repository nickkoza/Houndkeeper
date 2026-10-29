# [Oav] — The simulated arm holds position like a servo

**Status:** Superseded by [z7P] (27 Sep 2026: the real controller sends no gravity torque; the sim now compensates 80%, with printed-frame flex and backlash) · **Date:** 2026-09-25 · **Source:** measured in sim overnight 24–25 Sep 2026 · **Builds on:** [33f] · **ID:** [Oav]

## Context

The Maker Arm in every sim (standalone and DimOS) was a bare PD position controller per joint (kp 150 at the shoulder).
Holding its own 3.7 kg against gravity, a bare PD settles below its target: measured 0.05 rad at the shoulder and
0.04 rad at the elbow with the arm reaching down, about 5 cm at the fingertips. IK aimed the fingers correctly and the
arm put them 5 cm off, which looked like a perception or IK error. The real arm's RobStride servos hold their
commanded position.

## Decisions

1. [9rM] **Arm bodies are gravity-compensated through their actuators** (MuJoCo body gravcomp with actuator routing,
   MuJoCo 3.10.0), so the joints hold their targets (measured sag after: under 0.0001 rad) while the motors' torque
   limits still apply. Payload is not compensated.
2. [2eq] **Grasp, power and tip results measured before 25 Sep 2026 were with the sagging arm** and are re-measured
   before being relied on.
