"""Fit a draft mount from four measured floor points, using a local browser."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import html
import math
from pathlib import Path
import secrets
import webbrowser

import cv2
import numpy as np

from .ground_geometry import CameraCalibration, CameraMount, ground_to_image


def floor_rectangle(near_x, far_x, left_y, right_y):
    if not all(math.isfinite(v) for v in (near_x, far_x, left_y, right_y)):
        raise ValueError("floor coordinates must be finite")
    if far_x <= near_x or left_y <= right_y:
        raise ValueError("require far_x > near_x and left_y > right_y")
    # Click order: A far-left, B far-right, C near-left, D near-right.
    return np.array([[far_x, left_y, 0], [far_x, right_y, 0],
                     [near_x, left_y, 0], [near_x, right_y, 0]], dtype=float)


def fit_mount(calibration, old_mount, world, pixels, *, fix_origin=False):
    world, pixels = np.asarray(world, dtype=float), np.asarray(pixels, dtype=float)
    if world.shape != (4, 3) or pixels.shape != (4, 2) or not np.isfinite(world).all() or not np.isfinite(pixels).all():
        raise ValueError("need four finite world XYZ / raw pixel XY correspondences")
    if np.any(world[:, 2] != 0) or np.linalg.matrix_rank(world[:, :2]-world[0, :2]) != 2:
        raise ValueError("need noncollinear floor points at Z=0")
    if (pixels < 0).any() or (pixels[:, 0] > calibration.width-1).any() or (pixels[:, 1] > calibration.height-1).any():
        raise ValueError("clicked point outside calibrated raw image")
    polygon = pixels[[0, 1, 3, 2]].astype(np.float32)
    if not cv2.isContourConvex(polygon) or abs(cv2.contourArea(polygon)) < 100:
        raise ValueError("points must form a visible rectangle: click A, B, C, D in order")
    ok, rotations, translations, _ = cv2.solvePnPGeneric(
        world, pixels, calibration.matrix, calibration.distortion, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise ValueError("pose fit failed")
    optical = CameraMount.from_mapping({**asdict(old_mount), "roll_deg": 0., "pitch_down_deg": 0., "yaw_deg": 0.}).rotation
    candidates = []
    for rvec, tvec in zip(rotations, translations):
        rotation, _ = cv2.Rodrigues(rvec)
        origin = (-rotation.T @ tvec).reshape(3)
        camera_points = world @ rotation.T + tvec.reshape(3)
        if origin[2] <= 0 or (camera_points[:, 2] <= 0).any():
            continue
        level = rotation.T @ optical.T
        pitch = math.asin(float(np.clip(-level[2, 0], -1, 1)))
        if abs(math.cos(pitch)) < 1e-6:
            continue
        roll = math.atan2(level[2, 1], level[2, 2])
        yaw = math.atan2(level[1, 0], level[0, 0])
        mapping = {**asdict(old_mount), "height_m": float(origin[2]),
                   "camera_x_m": float(origin[0]), "camera_y_m": float(origin[1]),
                   **dict(zip(("roll_deg", "pitch_down_deg", "yaw_deg"), map(math.degrees, (roll, pitch, yaw))))}
        mount = CameraMount.from_mapping(mapping)
        if not np.allclose(mount.rotation, rotation.T, atol=1e-7):
            raise ValueError("camera pose could not be represented by mount angles")
        projected, _ = cv2.projectPoints(world, rvec, tvec, calibration.matrix, calibration.distortion)
        residuals = np.linalg.norm(projected.reshape(4, 2)-pixels, axis=1)
        candidates.append({"mount": mapping, "rmse_px": float(np.sqrt(np.mean(residuals**2))),
                           "residuals_px": residuals.tolist()})
    if not candidates:
        raise ValueError("no valid above-floor camera pose; check point order and measurements")
    candidates.sort(key=lambda item: item["rmse_px"])
    if fix_origin:
        # Retain independently measured height and offsets. Fit only angles.
        keys = ("pitch_down_deg", "roll_deg", "yaw_deg")
        angles = np.array([candidates[0]["mount"][key] for key in keys])
        def mapping(a):
            return {**asdict(old_mount), **dict(zip(keys, map(float, a)))}
        def residual(a):
            projected, _ = ground_to_image(world[:, :2], calibration, CameraMount.from_mapping(mapping(a)))
            return (projected-pixels).reshape(-1)
        damping = .001
        for _ in range(80):
            r = residual(angles)
            jacobian = np.column_stack([(residual(angles+np.eye(3)[i]*1e-4)-
                                        residual(angles-np.eye(3)[i]*1e-4))/2e-4 for i in range(3)])
            if not np.isfinite(r).all() or not np.isfinite(jacobian).all():
                raise ValueError("fixed-origin angle fit produced invalid rays")
            step = -np.linalg.solve(jacobian.T@jacobian+damping*np.eye(3), jacobian.T@r)
            trial = angles+step
            try:
                trial_r = residual(trial)
            except ValueError:
                trial_r = np.full(8, np.inf)
            if np.isfinite(trial_r).all() and np.linalg.norm(trial_r) < np.linalg.norm(r):
                angles, damping = trial, max(1e-9, damping/3)
            else:
                damping = min(1e12, damping*10)
            if np.linalg.norm(step) < 1e-9:
                break
        errors = np.linalg.norm(residual(angles).reshape(4, 2), axis=1)
        candidates = [{"mount": mapping(angles), "rmse_px": float(np.sqrt(np.mean(errors**2))),
                       "residuals_px": errors.tolist()}]
    return {"candidates": candidates, "selected_candidate": 0,
            "origin_fixed_to_measurements": fix_origin,
            "ambiguous": len(candidates) > 1 and candidates[1]["rmse_px"]-candidates[0]["rmse_px"] < 1,
            "height_change_m": candidates[0]["mount"]["height_m"]-old_mount.height_m,
            "verified_geometry": False, "ready_for_control": False,
            "note": "Draft fit only. Four fitted points are not independent validation; verify extra floor points before applying."}


PAGE = """<!doctype html><meta charset="utf-8"><title>Camera mount: four floor points</title>
<style>body{font:18px sans-serif;margin:20px;background:#eee}canvas{max-width:100%;height:auto;cursor:crosshair;display:block}button{font-size:18px;margin:12px 10px 12px 0}pre{white-space:pre-wrap}</style>
<h2>INSTRUCTIONS</h2>
<p>คลิกจุดสัมผัสพื้นของเครื่องหมายแต่ละชิ้น ผลจะบันทึกเป็นรายงานเท่านั้น</p>
<canvas id="view"></canvas><button id="undo">ย้อนจุดล่าสุด</button><button id="reset">เริ่มใหม่</button><button id="fit">คำนวณและบันทึก</button><pre id="status"></pre>
<script>
const canvas=document.querySelector('#view'),ctx=canvas.getContext('2d'),status=document.querySelector('#status');
const img=new Image(),points=[],names=LABELS_JSON;
function draw(){ctx.drawImage(img,0,0);ctx.font='bold 18px sans-serif';points.forEach((p,i)=>{ctx.strokeStyle='red';ctx.lineWidth=2;ctx.beginPath();ctx.arc(p[0],p[1],6,0,Math.PI*2);ctx.stroke();ctx.fillStyle='red';ctx.fillText(names[i],p[0]+9,p[1]-9)});status.textContent=JSON.stringify(points)}
img.onload=()=>{canvas.width=img.width;canvas.height=img.height;draw()};img.src='/image.png';
canvas.onclick=e=>{if(points.length===names.length)return;const r=canvas.getBoundingClientRect();points.push([(e.clientX-r.left)*canvas.width/r.width,(e.clientY-r.top)*canvas.height/r.height]);draw()};
document.querySelector('#undo').onclick=()=>{points.pop();draw()};document.querySelector('#reset').onclick=()=>{points.length=0;draw()};
document.querySelector('#fit').onclick=async()=>{if(points.length!==names.length){status.textContent='เลือกจุดให้ครบก่อน';return}try{const r=await fetch('/fit',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token:'TOKEN',pixels:points})});const data=await r.json();status.textContent=JSON.stringify(data,null,2)}catch(e){status.textContent=String(e)}};
</script>"""


def serve_points(image, labels, instructions, submit):
    """Pick original image coordinates locally and invoke a report callback."""
    image_bytes = cv2.imencode(".png", image)[1].tobytes()
    token = secrets.token_urlsafe(24)
    done = []

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, content_type, content):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            if self.path == "/":
                page = PAGE.replace("TOKEN", token).replace("LABELS_JSON", json.dumps(labels)).replace("INSTRUCTIONS", html.escape(instructions))
                self.reply(200, "text/html; charset=utf-8", page.encode())
            elif self.path == "/image.png":
                self.reply(200, "image/png", image_bytes)
            else:
                self.reply(404, "text/plain", b"not found")

        def do_POST(self):
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if self.path != "/fit" or not 0 < size <= 16384:
                    raise ValueError("invalid request")
                data = json.loads(self.rfile.read(size))
                if not secrets.compare_digest(str(data.get("token", "")), token):
                    raise ValueError("invalid session token")
                result = submit(data["pixels"])
                print(json.dumps(result, indent=2), flush=True)
                self.reply(200, "application/json", json.dumps(result).encode())
                done.append(True)
            except (ValueError, KeyError, TypeError, cv2.error) as exc:
                self.reply(400, "application/json", json.dumps({"error": str(exc)}).encode())

        def log_message(self, *args):
            pass

    with HTTPServer(("127.0.0.1", 0), Handler) as server:
        server.timeout = 0.5
        url = f"http://127.0.0.1:{server.server_port}/"
        print("Open on this computer:", url, flush=True)
        webbrowser.open(url)
        try:
            while not done:
                server.handle_request()
        except KeyboardInterrupt:
            pass
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("image", "camera-calibration", "camera-mount"):
        parser.add_argument("--"+name, type=Path, required=True)
    for name in ("near-x", "far-x", "left-y", "right-y"):
        parser.add_argument("--"+name, type=float, required=True)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mount-fit"))
    parser.add_argument("--fix-origin", action="store_true", help="retain measured height/offsets; fit only angles")
    args = parser.parse_args(argv)
    calibration = CameraCalibration.load(args.camera_calibration)
    mount = CameraMount.load(args.camera_mount)
    image = cv2.imread(str(args.image.expanduser()))
    if image is None or image.shape[:2] != (calibration.height, calibration.width):
        parser.error("image must be the original raw PNG at the calibrated resolution")
    world = floor_rectangle(args.near_x, args.far_x, args.left_y, args.right_y)

    def submit(pixels):
        report = fit_mount(calibration, mount, world, pixels, fix_origin=args.fix_origin)
        report.update({"world_xyz_m": world.tolist(), "pixels_xy": pixels,
                       "previous_mount": asdict(mount),
                       "calibration": str(args.camera_calibration.expanduser().absolute())})
        folder = args.output_dir.expanduser() / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        folder.mkdir(parents=True, exist_ok=False)
        (folder / "report.json").write_text(json.dumps(report, indent=2))
        (folder / "mount.candidate.json").write_text(json.dumps(report["candidates"][0]["mount"], indent=2))
        cv2.imwrite(str(folder / "raw.png"), image)
        return {"saved_to": str(folder.absolute()), **report}
    return serve_points(image, ["A", "B", "C", "D"],
                        "เลือก A ซ้ายไกล → B ขวาไกล → C ซ้ายใกล้ → D ขวาใกล้", submit)
