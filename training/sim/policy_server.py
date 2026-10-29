#!/usr/bin/env python3
"""Serve a LeRobot checkpoint over stdin/stdout (pickle) to the sim evaluator, predicting action chunks the way DimOS's
LeRobotPolicyModule runtime does (predict_action_chunk, pre/post processors from the checkpoint).
Request: {"image": uint8 HxWx3, "state": float32[7]} or {"reset": True}. Reply: float32[chunk, 7]."""
import pickle, sys
import numpy as np, torch
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.utils import prepare_observation_for_inference
path = sys.argv[1]; cfg = PreTrainedConfig.from_pretrained(path); cfg.device = "cuda" if torch.cuda.is_available() else "cpu"
policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg); policy.eval()
pre, post = make_pre_post_processors(policy_cfg=cfg, pretrained_path=path, preprocessor_overrides={"device_processor": {"device": cfg.device}})
inp, out = sys.stdin.buffer, sys.stdout.buffer
pickle.dump({"ready": True, "chunk": cfg.chunk_size, "n_action_steps": cfg.n_action_steps}, out); out.flush()
while True:
    try: req = pickle.load(inp)
    except EOFError: break
    if req.get("reset"): policy.reset(); pickle.dump({"ok": True}, out); out.flush(); continue
    with torch.inference_mode():
        obs = prepare_observation_for_inference({"observation.images.wrist": req["image"], "observation.state": req["state"]}, torch.device(cfg.device),
                                                task="pick up the item on the floor", robot_type="go2_maker_arm_sim")
        chunk = post(policy.predict_action_chunk(pre(obs)))
    pickle.dump(np.asarray(chunk.to("cpu").numpy(), np.float32)[0], out); out.flush()
