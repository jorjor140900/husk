# Fit the camera mount from measured floor points

`fit_camera_mount` opens a local browser to select four points in the original
raw image. It uses the raw-camera intrinsics/distortion and all six mount pose
parameters. The detector, ROS topics and robot commands are not used or changed.

Record the image with the camera fixed in its actual mount and unchanged focus
and capture mode. Measure coordinates from the same robot ground-frame origin:
X forward, Y left, Z up. The four points must lie on a flat floor. Use distinct
corners of a measured rectangle, with its sides parallel to the robot axes.
For the field measurements supplied on 2026-10-27:

```bash
ros2 run husky_vision fit_camera_mount \
  --image "$HOME/husky_vision_ws/robot/ground-calibration.png" \
  --camera-calibration "$HOME/husky_vision_ws/camera_calibration/camera-v2.yaml" \
  --camera-mount "$HOME/husky_vision_ws/robot/robot-mount.json" \
  --near-x 1.27 --far-x 1.87 --left-y 0.375 --right-y=-0.225 \
  --fix-origin
```

Select A far-left, B far-right, C near-left, D near-right. Browser resizing
preserves original pixel coordinates. Undo/reset are available before saving.
If no browser opens, open the printed localhost URL on the same computer.
This works with headless OpenCV; it does not require `cv2.imshow` or Qt.

The session saves `report.json`, `mount.candidate.json`, and `raw.png` under a
new `outputs/mount-fit/<timestamp>` directory, then exits. Nothing replaces the
active mount. The report includes both planar PnP candidates when available,
reprojection residuals, an ambiguity flag, and the previous measured mount.
Large disagreement with measured height/offsets must be resolved rather than
assuming the fit is physically correct. A small residual at the four fitted
points is not independent validation: check extra measured floor points before
applying a candidate or using the ground path for control.

`--fix-origin` retains the measured height and XYZ offsets and fits only roll,
pitch and yaw. A large remaining pixel residual indicates that the measurements,
point correspondence, or camera calibration need checking; do not force a
translation change just to reduce that residual. Omitting the option returns
the unrestricted planar PnP solutions for comparison.

## Check independent points without fitting

Keep three new markers at E=(1.00,0), F=(1.50,0), G=(1.50,+0.20) meters from
the robot ground origin. Save a fresh raw image after placing them. Run:

```bash
ros2 run husky_vision check_camera_mount \
  --image "$HOME/husky_vision_ws/robot/ground-calibration.png" \
  --camera-calibration "$HOME/husky_vision_ws/camera_calibration/camera-v2.yaml" \
  --camera-mount "$HOME/husky_vision_ws/robot/robot-mount.preview.json"
```

Click E near-center, F far-center, G far-left, at the center of each marker's
contact with the floor. Use the base of an upright marker, not its top/shadow.
The report lists measured vs projected XY, signed errors and total distance
errors in centimeters, under `outputs/mount-check/<timestamp>/report.json`.
It does not fit any parameters, write a new mount, enable control or publish ROS
messages. Passing these software calculations is not a physical validation;
review the measured errors and the measurement uncertainty before applying a
candidate.

## ตรวจว่าความกว้างที่ฉายลงพื้นแคบผิดจริงหรือ AI mask แคบ

ใช้เทปหกจุดที่วัดระยะจริง ไม่ใช้ขอบ mask/เส้นแดง/เส้นเหลืองเป็นจุดอ้างอิง
ยึดกล้องในตำแหน่งใช้งานจริง ตั้ง focus/zoom/ความละเอียดตรงกับ calibration และใช้พื้นราบ
การขยับกล้องเปลี่ยน mount; การเปลี่ยน focus/zoom อาจเปลี่ยน intrinsics ด้วย

วัดจาก **origin ของ robot ground frame**: X ไปหน้าหุ่น, Y ไปซ้ายหุ่น
ไม่วัดจากกันชนหรือหน้าเลนส์ สำหรับ rig นี้จุดใต้เลนส์มี offset X=-.26, Y=+.21 m
จึงเป็นคนละ origin; ตรวจตำแหน่ง origin จริงก่อนวางเทป

ตัวอย่าง crossbar กว้าง 1.00 เมตร centered ที่ Y=0 ทั้งสามแถว:

| จุด | X หน้า (m) | Y ซ้าย (m) |
| --- | ---: | ---: |
| N-L | 1.50 | +0.50 |
| N-R | 1.50 | -0.50 |
| M-L | 2.00 | +0.50 |
| M-R | 2.00 | -0.50 |
| F-L | 2.50 | +0.50 |
| F-R | 2.50 | -0.50 |

มองซ้าย/ขวาจากทิศหันหน้าของหุ่น เทปซ้ายกับเทปขวาห่างกัน 100 cm จริง
ตัวเลขนี้เป็นระยะระหว่างเทป ไม่ใช่การบังคับความกว้างทางเดิน 120 cm ให้เหลือ 100 cm
ถ้าใช้ขอบทางแทนเทป ต้องวัดพิกัดให้ตรงทั้ง X และ Y ไม่ใช่สมมติว่าหุ่นอยู่กลางทาง/ขนานทาง
เลือกขนาด/ระยะที่เห็นครบทั้งหกจุดแล้วใส่ค่าที่วัดได้จริงใน arguments

เปิด node กล้องเดิมค้างไว้ ไม่ต้องมี robot/map/goal/TF หรือ AI สำหรับเครื่องมือตรวจนี้
ทุก Terminal source ROS/workspace/venv และ export Cyclone เหมือนการวัด latency
คำสั่งนี้อ่านภาพดิบสดหนึ่งเฟรมจาก ROS แล้วปิด subscriber ก่อนเปิดหน้าเลือกจุด:

```bash
ros2 run husky_vision check_camera_mount \
  --capture-topic /camera/image_raw \
  --camera-calibration "$HOME/husky_vision_ws/camera_calibration/camera-active.yaml" \
  --camera-mount "$HOME/husky_vision_ws/robot/robot-mount.json" \
  --near-x 1.5 --far-x 2.5 --corridor-width 1.0 \
  --output-dir "$HOME/husky_vision_ws/outputs/mount-check"
```

วางเทปและหยุดกล้อง/หุ่นให้นิ่งก่อนรัน คลิกศูนย์กลางเทปบนพื้นตามลำดับ
N-L, N-R, M-L, M-R, F-L, F-R ไม่คลิกเงาหรือยอดกรวย
ไฟล์ภาพเดิมยังใช้ `--image FILE.png` แทน `--capture-topic` ได้
เครื่องมือไม่รับภาพคนละ frame, ขนาดไม่ตรง calibration หรือ source stamp เก่าเกิน 500 ms
บันทึกภาพดิบและ report ใหม่ ไม่แก้ mount, calibration, profile หรือส่งคำสั่งหุ่น

ดู `width_checks` ร่วมกับ `error_xy_cm`:

- `measured_width_m=1.0`, `estimated_width_m` ใกล้ 1.0 และ XY errors เล็กตามความคลาดการวัด:
  geometry บริเวณจุดเหล่านั้นสอดคล้องกับการวัด ถ้า AI corridor ยังแคบ ให้ตรวจ mask/colour veto ต่อ
- ระยะเทปจริง 1.0 แต่ประมาณได้ .8 ทุกแถว: geometry ฉายแคบ 20 cm โดยไม่เกี่ยวกับ AI
- ความผิดเพิ่มตามระยะหรือซ้าย/ขวาไม่เท่ากัน: ตรวจ mount angles, distortion และพิกัดจุดวัด
  รูปแบบ error อย่างเดียวไม่ระบุพารามิเตอร์ที่ผิดได้แน่นอน
- `estimated_lateral_width_m` และ `center_error_xy_cm` ช่วยแยกการเอียง/เลื่อนแกน
  ความยาว crossbar ถูกอย่างเดียวไม่ได้ยืนยันว่าทิศและตำแหน่งถูก

หุ่นที่เผื่อข้างละ 8 cm มีความไวต่อ error หลายเซนติเมตร ต้องเทียบกับ measurement uncertainty
และระยะเผื่อทั้งหมด ไม่ใช้ threshold จาก software เป็นใบรับรองความปลอดภัย

หาก geometry ผิด ให้ตรวจขนาด raw/focus/zoom และ intrinsics ก่อน จากนั้นวัด height/XY offsets
แล้วใช้ `fit_camera_mount --fix-origin` ตามหัวข้อแรก เพื่อ fit มุมโดยเก็บ origin ที่วัดไว้
ใช้พิกัด rectangle ที่วัดจริงของรอบนั้น ไม่คัดลอกพิกัดเก่าที่ไม่ตรงกับจุดใหม่
ผลเป็น `mount.candidate.json` เท่านั้น ตรวจด้วยจุดอื่นที่ไม่ได้ใช้ fit อีกครั้งก่อนนำมาใช้
อย่าขยับ height/pitch หรือลด footprint เพื่อบังคับให้ mask กว้างตามที่ต้องการ
