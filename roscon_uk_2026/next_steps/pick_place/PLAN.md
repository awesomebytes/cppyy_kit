# TIAGo perception and whole-body pick and place

## Intended demonstration

Detect colored tabletop objects from RGB and depth. Reach with the arm, recruit
other joints as required, grasp, lift, and place. Attempt arm-to-arm transfer after
the basic cycle works. Use physical contact first; label any attachment fallback.

## Implementation order

1. Start an owned Jazzy/Harmonic simulator. Capture joint states, calibration,
   transforms, images, and clouds. Check the gripper before building the sequence.
2. Add a small task controller to `control_kit`. Support independent position and
   orientation goals, secondary elbow goals, gaze, and primitive obstacles. Use
   weighted motion costs for arm, torso, and base preference. Report the distinction
   between weighted preference and strict task hierarchy.
3. Filter point clouds with PCL, remove the table, and cluster objects. Detect image
   colors and shapes with OpenCV. Keep numerical processing native and expose
   concise Python configuration.
4. Connect observations to a bounded pick/place sequence. Check measured state at
   each transition. Stop on stale state, failed reach, or missing observations.
5. Run the same perception stages as separate processes and in one process.
   Preserve algorithms and inputs; measure startup separately from warm execution.
6. Document commands, expected output, observed results, limitations, and recovery.

## Validation and scope

Use executable demonstrations, recorded frames, and saved measurements. Do not
add test files. Keep the existing playground launch available. Gazebo remains a
separate process in both application deployments. Do not infer zero-copy or
real-time guarantees from composition. Report successful runs and failures with
their actual conditions. Keep robot motion limits explicit.

The existing slides directory is user material and is outside this change.

## Additional Placo backend and controller cleanup

1. Preserve the custom backend and its validated sequence. Extract TIAGo joint,
   frame and motion-cost configuration; split the solve and integration steps
   into readable methods. Run the project linter and existing route probe.
2. Add Placo in a separate locked environment. Use its native task solver,
   joint/velocity constraints, URDF collision geometry and SRDF exclusions.
   Keep the same task API and add a command-line backend selector.
3. Check measured-state FK, task convergence, primitive obstacles, collision
   clearances and step timing. Validate the solver in the live ROS application.
4. Record the results and distinguish the external Python task servo from the
   separate Python `ControllerInterface` adapter with mock hardware.
