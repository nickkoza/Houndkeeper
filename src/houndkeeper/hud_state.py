"""The HUD's hand-off ([5AP]): the tidy module writes its state and latest step here, and DimOS's viewer (sim_hook.viewer,
another process) draws it. Its own module, so the robot's side names the file without importing the sim's hook."""
HUD_FILE = "/dev/shm/houndkeeper_hud.json"   # the tidy module's state for the HUD (another process)
