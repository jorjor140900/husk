"""Prepare separate no-joystick profiles without inventing field measurements."""
import argparse
import copy
import json
from pathlib import Path
import tempfile

from husky_link_core.operator_permission import PERMISSION_TOPIC, operator_settings
from .robot_profile import load_robot_profile


def prepare_operator_profile(source, output, robot_output, motion=False, supervised_trial=False, supervised_goal=False):
    from husky_link_core.supervised_trial import MODE, LIMITS, GOAL_LIMITS
    if supervised_goal and supervised_trial:
        raise ValueError("choose supervised trial or supervised goal")
    goal_session=supervised_goal
    supervised_trial=supervised_trial or supervised_goal
    limits=GOAL_LIMITS if goal_session else LIMITS
    if motion and supervised_trial:
        raise ValueError("choose normal motion or supervised trial")
    source, output, robot_output = [Path(path).expanduser().resolve() for path in (source, output, robot_output)]
    if output == robot_output or output.exists() or robot_output.exists():
        raise ValueError("choose two distinct NEW output files; existing profiles are never overwritten")
    profile = copy.deepcopy(json.loads(source.read_text()))
    robot, bridge = profile["robot"], profile["bridge"]
    # A previously checked joystick is not evidence for this new Bool console.
    # Preserve the original file, but invalidate checks whose wiring changes.
    invalidated = []
    if (robot.get("ros1_permission_topic") != PERMISSION_TOPIC or robot.get("joy_topic") or
            robot.get("joy_deadman_button") is not None or
            robot.get("external_permission_topic") != "/husky_link/external_permission"):
        profile["field_measurements"]["independent_permission_gate_checked"] = False
        invalidated.append("independent_permission_gate_checked")
    if bridge.get("expected_output_topic") != "/cmd_vel":
        profile["field_measurements"]["mux_and_base_timeout_checked"] = False
        invalidated.append("mux_and_base_timeout_checked")
    robot.update(ros1_permission_topic=PERMISSION_TOPIC, joy_topic=None, joy_deadman_button=None,
                 external_permission_topic="/husky_link/external_permission")
    profile.pop("operation_mode", None)
    profile.pop("trial_limits", None)
    if supervised_trial:
        profile.update(operation_mode=MODE, trial_limits=copy.deepcopy(limits))
    bridge.update(forward_commands=motion or supervised_trial, expected_output_topic="/cmd_vel")
    operator_settings(profile, motion or supervised_trial, supervised_trial)
    # Validate geometry/files/field measurements before creating a motion file.
    # Preview preparation deliberately preserves incomplete physical checks.
    if motion or supervised_trial:
        with tempfile.TemporaryDirectory() as folder:
            temporary = Path(folder)/"profile.json"
            temporary.write_text(json.dumps(profile))
            load_robot_profile(temporary, allow_supervised_trial=supervised_trial)
    robot_profile = {key: copy.deepcopy(profile[key]) for key in
                     ("schema_version", "bridge", "robot", "field_measurements")}
    if supervised_trial:
        robot_profile.update(operation_mode=MODE, trial_limits=copy.deepcopy(limits))
    created = []
    try:
        for path, value in ((output, profile), (robot_output, robot_profile)):
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("x") as destination:
                created.append(path)
                destination.write(json.dumps(value, indent=2, allow_nan=False)+"\n")
    except Exception:
        for path in created:
            path.unlink()
        raise
    return dict(mode=MODE if supervised_trial else ("motion" if motion else "preview"), profile=str(output), robot_profile=str(robot_output),
                permission_topic=PERMISSION_TOPIC, expected_output_topic="/cmd_vel",
                invalidated_checks=invalidated, trial_limits=copy.deepcopy(limits) if supervised_trial else None,
                field_measurements=profile["field_measurements"], ready_for_autonomous_control=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--robot-output", required=True)
    parser.add_argument("--motion", action="store_true", help="requires previously recorded field checks; does not set them true")
    parser.add_argument("--supervised-trial", action="store_true", help="explicit bounded attended trial; LiDAR NOT checked, field checks remain unchanged")
    parser.add_argument("--supervised-goal", action="store_true", help="attended goal session: 0.05 m/s, backup 3 m/120 s; LiDAR NOT checked")
    args = parser.parse_args(argv)
    try:
        report = prepare_operator_profile(args.profile, args.output, args.robot_output, args.motion, args.supervised_trial, args.supervised_goal)
    except (ValueError, KeyError, OSError, TypeError) as exc:
        parser.exit(2, str(exc)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
