#!/usr/bin/env python3
"""Feed the saved house map into DimOS's live mapper (its `lidar` input, world frame), then report how big
the mapper's global_map is afterwards. Usage: inject_map.py <map.npy>"""
import sys, time, numpy as np
from dimos.core.transport_factory import make_transport, transport_topic
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
P = np.load(sys.argv[1])
got = {}
bus = Zenoh(); bus.start()
bus.subscribe(Topic(f"{transport_topic('/global_map')}/**"), lambda m, _t: got.__setitem__("n", len(np.asarray(m.as_numpy()[0] if isinstance(m.as_numpy(), tuple) else m.as_numpy()))))
time.sleep(2); before = got.get("n", 0)
tx = make_transport("/lidar", PointCloud2)
for chunk in np.array_split(P, max(1, len(P) // 20000)):          # lidar-frame-sized pieces
    msg = PointCloud2.from_numpy(chunk.astype(np.float32), frame_id="world", timestamp=time.time())
    (getattr(tx, "publish", None) or getattr(tx, "broadcast"))(msg); time.sleep(0.2)
time.sleep(6); after = got.get("n", 0); bus.stop()
print(f"injected {len(P)} points; mapper global_map {before} -> {after} points")
