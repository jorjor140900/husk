"""Prepare separate no-joystick profiles without inventing field measurements."""
import argparse
import copy
import json
from pathlib import Path
import tempfile

from husky_link_core.operator_permission import PERMISSION_TOPIC, operator_settings
from .robot_profile import load_robot_profile


def prepare_operator_profile(source, output, robot_output, motion=False):
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
    bridge.update(forward_commands=motion, expected_output_topic="/cmd_vel")
    operator_settings(profile, motion)
    # Validate geometry/files/field measurements before creating a motion file.
    # Preview preparation deliberately preserves incomplete physical checks.
    if motion:
        with tempfile.TemporaryDirectory() as folder:
            temporary = Path(folder)/"profile.json"
            temporary.write_text(json.dumps(profile))
            load_robot_profile(temporary)
    robot_profile = {key: copy.deepcopy(profile[key]) for key in
                     ("schema_version", "bridge", "robot", "field_measurements")}
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
    return dict(mode="motion" if motion else "preview", profile=str(output), robot_profile=str(robot_output),
                permission_topic=PERMISSION_TOPIC, expected_output_topic="/cmd_vel",
                invalidated_checks=invalidated,
                field_measurements=profile["field_measurements"], ready_for_autonomous_control=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--robot-output", required=True)
    parser.add_argument("--motion", action="store_true", help="requires previously recorded field checks; does not set them true")
    args = parser.parse_args(argv)
    try:
        report = prepare_operator_profile(args.profile, args.output, args.robot_output, args.motion)
    except (ValueError, KeyError, OSError, TypeError) as exc:
        parser.exit(2, str(exc)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
