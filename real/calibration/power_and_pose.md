# Walking power, crouch height, mass

Turns `go2_air.walk_power_W` (estimate), `crouch_height_m` (unknown) and checks `mass_kg` (vendor) ([4hm]).

## 1. Walking power (`walk_power_W`, direct)

DimOS's Go2 connection keeps the robot's low state, with the battery's charge (%), voltage and current
(`bms_state.soc`, `bms_state.current` in mA, `power_v`). Houndkeeper's battery module (`houndkeeper.battery`,
`src/houndkeeper/battery_module.py`) reads it once a second through the connection's `lowstate` RPC (a one-method
local addition to DimOS) and publishes the `battery` stream (time, charge %, volts, amps with the pack's sign, watts
drawn), so a recorded run keeps it beside odometry. Add it to every real run:
`dimos --record sqlite run unitree-go2-agentic houndkeeper.battery ...`.

1. Charge full; let it rest 10 minutes.
2. Record (`--record sqlite`, with `houndkeeper.battery`): stand still for 5 minutes, then walk the room for 20
   minutes on its own route (a night's sweep, no items).
3. `measure.py power recordings/<run>/memory.db`: mean power walking (odometry moving) and standing (odometry still),
   from the battery stream; and the check against the charge, (start % - end %) x 236.8 Wh / hours against the mean
   power over the same time, which should agree within ~10% (it says so when they don't). The charge comes in whole
   percent: over 25 minutes that is a few percent of the answer.

`--write` puts `walk_power_W` in the file and adds `stand_power_W` beside it. Then re-run the battery coverage ([7Xn]).

## 2. Crouch height (`crouch_height_m`, direct)

Command the crouch (DimOS or the app in the online window); measure the top of the trunk above the floor at the
front and back hip motors (mm), and the same standing. Write the crouch's mean height; the arm's reach and tip
limits re-run.

## 3. Mass (`mass_kg`, check)

Bathroom scale: weigh yourself, then yourself holding the robot (powered off, lying), twice; the difference. Then
with the arm when it arrives. If it differs from Unitree's 15 kg by more than 0.5 kg, write it as measured; the
payload and tip sims re-run.
