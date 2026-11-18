# Predictions (written before the parquet analysis ran; November 13)

Data: LeRobot v3.0 parquet, 3,500 episodes, 35 tasks, 14-D joint state/action at 25 Hz (known from #27, G3). Per-task HDF5 holds the same joints plus end-effector poses and JPEG frames (one episode inspected).

| Quantity | Prediction (band) | Decision it changes |
|---|---|---|
| Episodes longer than 700 frames | 0 to 5% of 3,500 (point: 1%); longest 700 to 1500 | If many exceed 700, the demonstrations teach behaviour the evaluation clock would cut off |
| Episodes with an arm joint outside ARX's real range (arx5-sdk X5 table, #11) at any frame | 20 to 80% of episodes, joint 2 (index 1) or joint 4 most often | If near 0, #11's loose limits never matter for demos; if high, policies learn poses the real arm cannot reach |
| Frames with arm-joint step above 5 rad/s (0.2 rad per frame) | under 0.5% of frames; some episodes (0 to 100) with a step above 1 rad (teleport) | A step above 1 rad = not physical |
| First-frame state differs from the task's median start pose | most episodes start at one home pose within 0.05 rad | Odd starts hint at stitched or mis-reset episodes |
| Exact duplicate episodes / episodes under 50 frames | 0 to 5 duplicate pairs; 0 to 10 short | Sanity |
| Gripper values outside [0, 1] | none | Clamp artefact check |
| Gripper quantisation | closing ramps fall by exactly 0.1 per frame (seen in #27) in most closures | Control artefact in the command path |
| Success labels / object states in either format | absent in both | Report as such |
