# The lidar: range spread, what DimOS passes on, and what it misses

Turns `go2_air.lidar` (vendor figures) and `go2_air.lidar_dropouts` (estimates) into measurements ([4hm]).

## 1. Range spread on a flat wall (`lidar.range_sd`, direct)

1. Stand the robot 1.5 m from a plain, matte, flat wall (painted plaster), square to it; record a minute standing.
2. In the recording's map frame, a box round a patch of the wall away from corners and the floor (x0 x1 y0 y1 z0 z1,
   m): `measure.py lidar <memory.db> --box x0 x1 y0 y1 z0 z1 --write` fits each scan's plane and reports the points'
   spread about it. Unitree quotes ~2 cm.

## 2. What DimOS passes on (rate, matched)

`measure.py lidar <real.db> --sim <sim.db>`: messages a second, points a second and points a message as DimOS
records them (the sim recorded 1.55 messages, ~9,000 points a second of its 21,600 drawn: DimOS registers and
thins them). If the real robot's differ, the sim's points per second is tuned until DimOS passes on the same.

## 3. Dark and shiny surfaces (`lidar_dropouts`, matched)

1. Stand the robot 1.5 m from the black sofa (or dark fabric), then from the TV (off) and a mirror, a minute each,
   recording; and once more from the plain wall beside each.
2. `measure.py lidar <db> --box ...` round the dark surface, then round the wall: the returns a scan puts in each
   box. A dark surface's dropout = 1 - (its returns per square metre / the wall's).
3. The same boxes on a sim recording of a like scene give the sim's; tune `dark_drop`, `shiny_drop` (and the albedo
   under which a surface counts as dark) until they agree.
