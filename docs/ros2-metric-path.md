# จากเส้นในภาพ → เส้นหน่วยเมตรใน RViz (ROS 2 Jazzy)

ใช้ workspace เดิม `~/husky_vision_ws` และโมเดลเดิม ขั้นนี้เพิ่ม
`/drivable_area/reference_ground` ชนิด `nav_msgs/Path` สำหรับตรวจระยะบนพื้นราบ
ชุด perception/projection นี้ไม่ส่งคำสั่งเคลื่อนที่และยังไม่ตรวจ footprint ของ Husky
หากยังตรวจระยะจริงไม่ได้ สามารถพักขั้นนี้แล้วทำ [controller ใน simulation](ros2-control.md) ต่อได้
ภาพและเส้น overlay เดิมใช้งานเหมือนเดิม

ทำตามลำดับ: **อัปเดต → คาลิเบรตเลนส์ → วัดการติดตั้ง → ตรวจเส้นหน่วยเมตร**
ยังไม่ต้องรู้รายละเอียด ROS 1 หรือ bridge

## 1. อัปเดตแพ็กเกจที่มีอยู่

หยุด launch เดิมด้วย Ctrl+C เปิด Terminal ใน **โฟลเดอร์ Git เดิมที่มี `RoadDetection.py`**:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py --workspace ~/husky_vision_ws --update-code
```

อัปเดต ROS adapters, launch, metadata ของแพ็กเกจ, เครื่องมือ projection และไฟล์ประกอบ
สำรองไฟล์ที่มีอยู่ก่อนใน `.source-backups/<เวลา>/` ไม่เขียนทับโมเดล,
`road_navigation`, `config.example.json` หรือไฟล์ calibration ที่เก็บนอก source package
หากแก้ `setup.py`/`package.xml` เองไว้ ให้ตรวจสำเนาสำรองหลังอัปเดต

เปิด **Terminal ใหม่** เพื่อ build:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
sudo apt install ros-jazzy-camera-calibration ros-jazzy-rviz2 python3-yaml
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
```

ใช้ `local_setup.bash` ตามเดิมเพื่อไม่ดึง parent workspace เก่ามาอีก

## 2. เตรียมตารางคาลิเบรต (พิมพ์หรือแสดงบนจอ)

เปิด PDF ที่อยู่ในเครื่องหลังอัปเดต:

```bash
xdg-open ~/husky_vision_ws/src/husky_vision/config/checkerboard-9x6-25mm.pdf
```

**หากยังพิมพ์ไม่ได้:** เปิด PDF บนจอคอม/แท็บเล็ตให้ใหญ่และคงซูมไว้
หันเว็บแคมไปถ่ายตารางบนจอ ใช้จอปัจจุบันก็ได้โดยวางหน้าต่าง calibrator ไว้ด้านหลังตาราง
อย่าให้ภาพ preview ของกล้องมาทับตารางจนเกิดภาพซ้อนในภาพ
คงตารางบนจอให้อยู่นิ่ง แล้วขยับ/เอียงตัวเว็บแคมเพื่อเก็บหลายมุมแทนการย้ายกระดาน
วัดช่องบนจอด้วยไม้บรรทัดจริง เช่น ถ้าวัดได้ 20 mm ให้ใช้ `--square 0.020`
ตัวเลข 25 mm ใน PDF เป็นขนาดสำหรับพิมพ์ ไม่รับรองว่าจอแสดงขนาดนั้นจริง
หลีกเลี่ยงภาพสะท้อน/แถบกระพริบ และต้องเห็นขอบขาวกับมุมตารางชัดครบ

**ถ้าพิมพ์ได้ภายหลัง:** พิมพ์ A4 แนวนอน โดยเลือก **Actual size / 100%** ปิด Fit to page
ใช้ไม้บรรทัดวัดช่องหนึ่งต้องกว้าง **25 mm** และแปะกระดาษบนแผ่นแข็งเรียบ
ตารางมี 10×7 ช่อง นับ **มุมภายใน 9×6** เป็นค่าที่ต้องกรอกให้โปรแกรม
ถ้าพิมพ์ขนาดอื่น ให้วัดขนาดช่องจริงแล้วเปลี่ยน `--square` เป็นเมตร
ทั้งสองวิธีต้องให้ webcam ถ่ายตารางจริงที่อยู่ตรงหน้าเลนส์

คาลิเบรตที่ความละเอียด **ภาพกล้อง 640×480** ไม่ใช่ tensor ของโมเดล 320×192
คง zoom/focus และโหมด capture เดิมหลังคาลิเบรต หาก webcam เปลี่ยน focus อัตโนมัติ
ให้ล็อก focus เมื่อทำได้และใช้ค่าเดียวกันตอนทดลอง

## 3. เปิดเฉพาะกล้อง แล้วคาลิเบรตด้วยเครื่องมือ ROS

Terminal A — ใช้เฉพาะ webcam เพื่อไม่ต้องรัน AI ระหว่างคาลิเบรต:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
ros2 run husky_vision webcam_node --ros-args \
  -p camera_index:=2 -p width:=640 -p height:=480 -p fps:=15.0
```

ตรวจ log ว่าขนาดจริงเป็น 640×480 หากต่างออกไป ต้องคาลิเบรตและเปิดกล้องครั้งต่อไปด้วยขนาดจริงนั้น

Terminal B — เปิด **Terminal ใหม่โดยไม่ activate venv** เพื่อให้ GUI ใช้ system OpenCV ของ ROS:
เปลี่ยน `--square 0.025` เป็นขนาดช่องจริงที่วัดได้ หากใช้ตารางบนจอ

```bash
source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=xcb ros2 run camera_calibration cameracalibrator \
  --size 9x6 --square 0.025 --no-service-check \
  --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

1. ถือกระดานให้เห็นครบทุกช่องและขอบขาว ย้ายไปซ้าย ขวา บน ล่าง และตรงกลาง
2. เปลี่ยนระยะใกล้–ไกล และเอียงกระดานหลายมุม ค้างให้นิ่งในแต่ละท่าเพื่อหลีกเลี่ยง motion blur
3. รอแถบ X, Y, Size, Skew มีตัวอย่างหลากหลายจนปุ่ม **CALIBRATE** กดได้ แล้วกด
4. เมื่อคำนวณเสร็จ ดูภาพ rectified และค่า reprojection error; ถ้ายังบิดชัดหรือ error สูงให้เก็บภาพใหม่
5. กด **SAVE** จะได้ `/tmp/calibrationdata.tar.gz` และบันทึกไฟล์นี้เก็บไว้อีกที่ก่อน reboot

ใช้ **SAVE** ได้โดยไม่ต้องกด COMMIT เพราะ webcam node นี้ไม่มีบริการ `set_camera_info`
ตัวเลือก `--no-service-check` จึงจำเป็น
ถ้า `/tmp/calibrationdata.tar.gz` มีไฟล์จากครั้งก่อน ให้ย้ายเก็บก่อนกด SAVE ซึ่งใช้ชื่อเดิม

กลับมา Terminal A: Ctrl+C เพื่อปิดกล้อง แล้วนำเข้า YAML ด้วย venv เดิม:

```bash
ros2 run husky_vision import_camera_calibration \
  --output "$HOME/husky_vision_ws/camera_calibration/camera.yaml"
```

เครื่องมืออ่านเฉพาะ `ost.yaml` จาก archive ตรวจรูปแบบและไม่เขียนทับไฟล์ปลายทางเดิม
หากต้องการเก็บครั้งที่สองให้เลือกชื่อใหม่ เช่น `camera-v2.yaml`
รองรับ pinhole `plumb_bob`/`rational_polynomial`; ยังไม่รองรับ fisheye `equidistant`

## 4. ยึดกล้องและวัดตำแหน่งจริง

ใช้ขาตั้งชั่วคราวบนพื้นราบได้ กำหนดแกน **X ไปข้างหน้า, Y ไปซ้าย, Z ขึ้น**
จุดกำเนิด `vision_ground` อยู่ที่พื้นใต้เลนส์สำหรับการทดลองครั้งนี้
อย่าถือกล้องด้วยมือระหว่างตรวจระยะ เพราะความสูง/มุมที่เปลี่ยนจะทำให้พิกัดเมตรผิด

```bash
cp -n ~/husky_vision_ws/src/husky_vision/config/camera_mount.example.json \
  ~/husky_vision_ws/camera_calibration/mount.json
nano ~/husky_vision_ws/camera_calibration/mount.json
```

แทนค่า `null` ด้วย **ค่าที่วัดจริง**:

- `height_m`: ความสูงจากพื้นถึง **ศูนย์กลางเลนส์** หน่วยเมตร
- `pitch_down_deg`: มุมก้มจากแนวนอน หน่วยองศา บวกคือก้มลง; 0 คือมองขนานพื้น
- `roll_deg`: มุมเอียงซ้ายขวาของชุดกล้อง ตามกฎมือขวารอบแกน X; เริ่มด้วยปรับขาตั้งให้ระดับแล้วใช้ 0
- `yaw_deg`: ทิศของกล้องเทียบแกน X บวกคือหันซ้าย; กำหนดแกน X ตามกล้องแล้วใช้ 0 ในการทดลอง
- `camera_x_m`, `camera_y_m`: ระยะเลนส์จากจุดกำเนิดบนพื้น; เมื่อใช้จุดใต้เลนส์เป็นกำเนิด ทั้งสองเป็น 0
- `ground_frame`, `camera_frame`: ใช้ `vision_ground` และ `camera_optical_frame` ตามไฟล์ตัวอย่าง

ตัวอย่างการแปลงหน่วย: วัดสูง 60 cm ให้กรอก `0.60`; **อย่าใช้ 0.60 ถ้าไม่ได้วัดได้เท่านี้**
วัดมุมก้มด้วย inclinometer/เครื่องวัดมุมที่อ้างอิงแนวเลนส์กับแนวนอน
ถ้าตัวกล้องไม่ขนานกับแนวเลนส์ ต้องเผื่อ offset นั้นด้วย แล้วตรวจระยะบนพื้นตามขั้นถัดไป
หากยังไม่ทราบมุมจริง ให้หยุดที่การคาลิเบรตเลนส์ก่อน ไม่เดาค่าเพื่อให้มีเส้นขึ้น

มุมใช้ลำดับ `Rz(yaw) Ry(pitch_down) Rx(roll)` จาก frame กล้องที่มองไปข้างหน้า
ก่อนแปลงแกน optical ของ ROS (X ขวา, Y ลง, Z ไปตามเลนส์)
เมื่อย้ายไปติด Husky จริง ต้องวัด mount ใหม่และใช้ TF ของหุ่นแทน frame ทดลองนี้

## 5. เปิด AI เดิม แล้วดูเส้นหน่วยเมตร

Terminal A:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
ros2 launch husky_vision drivable_area.launch.py \
  camera_index:=2 camera_width:=640 camera_height:=480 \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=320 model_height:=192 cpu_threads:=2
```

Terminal B — ต้อง activate venv เพราะ projection ใช้ NumPy/OpenCV จากชุดเดียวกัน:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision ground_path.launch.py \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera.yaml" \
  camera_mount:="$HOME/husky_vision_ws/camera_calibration/mount.json"
```

RViz เปิดมุมมองบนพื้นให้ เส้นเหลืองเป็น projection หน่วยเมตร ตารางช่องละ **0.5 m**
แกนแดงคือ +X และแกนเขียวคือ +Y ถ้าไม่เห็นเส้น ตรวจว่า Fixed Frame เป็น `vision_ground`
ใช้ `rviz:=false` เพื่อรันเฉพาะ projection แล้วเปิด RViz แยกได้

ตรวจสถานะใน Terminal อีกหน้าที่ source Jazzy/workspace แล้ว:

```bash
ros2 topic echo /drivable_area/ground_status --once
ros2 topic echo /drivable_area/reference_ground --once
```

`ok` หมายถึงฉายเส้นครบ; `range_clipped`/`horizon_clipped`/`segment_gap_clipped`
หมายถึงเก็บเฉพาะส่วนต่อเนื่องด้านใกล้ที่ฉายได้ ไม่มีการเชื่อมข้ามช่วงที่ฉายไม่ได้
`insufficient_ground_path:*` คือไม่มีเส้นยาวพอให้แสดง
`image_path_invalid:*` คือ detector เดิมไม่มี reference
`projection_rejected` ให้ดู warning เรื่อง resolution/frame/ข้อมูล calibration
ถ้า timestamp เก่าหรือไม่มีผลใหม่ เส้นจะถูกล้าง; `max_frame_age_sec` เริ่มต้น 2 วินาที
ค่านี้ใช้ตรวจภาพทดลอง ยังไม่ใช่เกณฑ์ watchdog ที่ยอมรับสำหรับควบคุมหุ่น

## 6. ตรวจระยะก่อนทำ controller

### แสดงจุดระยะบนภาพกล้องแล้ววัดเทียบจริง

อัปเดตแพ็กเกจด้วย `--update-code` และ build ตามขั้น 1 แล้วหยุด launch กล้อง/AI/preview เดิมด้วย Ctrl+C
ยึดกล้องในตำแหน่งเดิม คง capture/focus/zoom ตาม calibration และใช้พื้นราบ
โหมดนี้เปิดเฉพาะ webcam + เรขาคณิต + image_view จึงไม่ต้องโหลดโมเดลหรือเปิด RViz

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision ground_check.launch.py \
  camera_index:=2 \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera-v2.yaml" \
  camera_mount:="$HOME/husky_vision_ws/camera_calibration/mount.json"
```

เปลี่ยนชื่อ calibration ให้ตรงกับไฟล์จริง โหมดนี้ไม่แก้ mount/calibration และไม่ส่ง TF, Path หรือคำสั่งมอเตอร์
ถ้าใช้กล้อง publisher ภายนอกอยู่แล้ว ให้ใช้ `use_webcam:=false image_topic:=/your/image_raw`
แสดงจุดจากพิกัดพื้น Z=0 ผ่าน intrinsics และ distortion ของภาพดิบ
แถบข้อความที่เพิ่มบนภาพมีไว้แสดงผลเท่านั้น พิกัด `raw_pixel_uv` ใน status ยังอ้างอิงภาพดิบ 640×480

1. ใช้สายห้อยน้ำหนัก/แนวดิ่งหาจุดบนพื้น **ใต้ศูนย์กลางเลนส์** ติดเทปเล็ก ๆ เป็นจุดเริ่มวัด
2. กำหนด +X ไปข้างหน้า, +Y ไปซ้ายตาม ground frame; ในชุดทดลองให้ yaw/roll ตรงกับแกนและค่าที่วัดไว้
3. วัดและติดเทปแบนบนพื้นตามตาราง ไม่วัดจากขาตั้ง หน้ากล้อง หรือหน้าต่างบนจอ
4. ดูว่ากากบาทที่มีชื่อเดียวกันทับกลางเทปจริงหรือไม่ ตรวจหลายระยะและด้านซ้ายขวา

| จุดบนภาพ | ระยะไปข้างหน้าจากจุดใต้เลนส์ | ระยะซ้าย/ขวา |
| --- | --- | --- |
| `C010` | 10 cm | ตรงกลาง Y=0 |
| `C025` | 25 cm | ตรงกลาง Y=0 |
| `C050` | 50 cm | ตรงกลาง Y=0 |
| `L025` | 25 cm | ไปซ้ายอีก 20 cm |
| `R025` | 25 cm | ไปขวาอีก 20 cm |

เลือกจุดที่อยู่ในภาพจริง แถบบนบอก `Visible center X` จุดนอกภาพไม่แสดง
กล้องก้มประมาณ 70° อาจเห็นเฉพาะช่วง 10–50 cm จึงไม่กำหนดให้ต้องเห็น 1–3 m
จุดอื่นมี `C075`, `C100`, `C150`, `C200`, `C300` เมื่อระยะนั้นอยู่ในมุมมองกล้อง
`L`/`R` ทุกจุดอยู่ที่ Y=+0.20/-0.20 m; ระยะ X อ่านจากตัวเลขชื่อจุดเป็น cm
สำหรับ mount ที่ camera_x/y ไม่เป็นศูนย์ ตารางนี้ยังเริ่มจากพื้นใต้เลนส์ แต่แกนยังตาม ground frame

ถ้าต้องการบันทึก error เป็น cm ให้เลื่อนเครื่องหมายแบนอีกชิ้นบนพื้นจนภาพของมันตรงกับกากบาท
แล้ววัด X/Y จริงของชิ้นนั้นเทียบกับค่าที่ป้ายระบุ แยกจากจุดตรวจที่ติดไว้เดิม
เก็บผลหลายจุดก่อนปรับความสูง/มุมก้ม และคงจุดหนึ่งไว้ตรวจซ้ำหลังปรับ
ตรวจ focus, ขนาด capture, จุดอ้างอิงเลนส์, roll/yaw และพื้นราบด้วย
หลังแก้ mount หรือ calibration ต้อง Ctrl+C แล้วเปิดโหมดตรวจใหม่ให้โหลดค่าล่าสุด

อ่านค่าที่ใช้และจุดที่มองเห็นได้:

```bash
ros2 topic echo /ground_check/status --once
```

`valid_ground_overlay` ยืนยันเพียงว่าซอฟต์แวร์ฉายจุดและรับภาพถูกขนาด/frame/เวลา
`physical_geometry_verified` ยังเป็น false เพราะการทับตรงกับเครื่องหมายจริงต้องวัดตรวจเอง
ถ้า frame/resolution ผิดหรือภาพเก่า/กล้องหยุด ภาพเปลี่ยนเป็นคำเตือนและล้างตาราง
หลังตรวจระยะแล้วกลับไปเปิด `vision_preview.launch.py` เพื่อเก็บความหน่วงของ AI ต่อ
ระยะมองที่สั้นจากกล้องก้มมากต้องตรวจเทียบกับ lookahead และระยะหยุดก่อนเลือกการติดตั้งบนหุ่นจริง

### ตรวจพิกัดพิกเซลเองเพิ่มเติม

ใช้จุดบนพื้นหลายระยะที่อยู่ในภาพ เช่น 1 m, 2 m และ 3 m เมื่อมุมกล้องมองถึง
เพื่อทดสอบพิกัดจุดบนภาพเดียวกันโดยตรง ใช้ Python ใน Terminal ที่เปิด venv:

```python
from husky_vision.ground_geometry import CameraCalibration, CameraMount, project_path
from pathlib import Path
root = Path.home() / "husky_vision_ws/camera_calibration"
camera = CameraCalibration.load(root / "camera.yaml")
mount = CameraMount.load(root / "mount.json")
# ใส่จุดกึ่งกลางเทปสองจุดที่อ่านจาก /camera/image_raw 640x480
# เรียงจากใกล้ไปไกล; u คือแนวนอน v คือแนวตั้ง ไม่ใช่พิกัดบนหน้าต่างที่ถูกยืด
pixels = [[u_near, v_near], [u_far, v_far]]  # แทนตัวแปรด้วยตัวเลขที่อ่านได้จริง
points, reason = project_path(pixels, [480, 640], [480, 640], camera, mount,
                              max_segment_m=5.0)
print(points, reason)  # X ต้องใกล้ระยะเทปจริง; Y ซ้ายบวก ขวาลบ
```

โค้ดตัวอย่างนี้ยังรันไม่ได้จนกว่าจะแทน `u_near`/`v_near`/`u_far`/`v_far` ด้วยค่าจริง
อ่านพิกัดจากภาพดิบที่เซฟจาก viewer แล้วเปิดด้วยเครื่องมือดูภาพที่บอกพิกัดพิกเซล
ใช้จุดที่รู้ตำแหน่ง Y ด้วยเพื่อตรวจแกนซ้ายขวา และบันทึก error เป็น cm ในแต่ละระยะ
ระยะช่วงไกลไวต่อมุมก้มมาก หากไม่ตรงให้ตรวจ mount, focus, calibration และพื้นราบก่อน
อย่าปรับค่ากล้องเพื่อบังคับเส้นให้ดูตรงเพียงภาพเดียว

นี่เป็นมุมมองด้านบนของเส้นทาง ไม่ได้สร้าง occupancy grid หรือ BEV ของทั้งภาพ
RGB ที่ทายเสาผิดจะยังฉายเสานั้นลงพื้นผิดได้ พื้นลาด/ต่างระดับและการสั่นของหุ่นต้องมีข้อมูลท่าทาง/ความลึกเพิ่ม
ทำ [controller ใน simulation](ros2-control.md) ควบคู่กันได้โดยไม่ต้องรอระยะภาคสนาม
ก่อนต่อหุ่นจริงยังต้องมี metric drivable/obstacle map + footprint,
odometry/TF และเงื่อนไขหยุด ก่อนต่อ bridge ไปหุ่นจริง

## Topic และ frame

| Topic | ชนิด | ความหมาย |
| --- | --- | --- |
| `/drivable_area/reference_ground` | `nav_msgs/Path` | เส้นเปิด near→far หน่วยเมตรบน Z=0 ของ `vision_ground`; timestamp ของภาพต้นทาง |
| `/drivable_area/ground_status` | `std_msgs/String` JSON | ผล projection; `ready_for_control=false`, `footprint_checked=false` |
| `/tf_static` | TF | mount ที่วัดจริง: `vision_ground` → `camera_optical_frame` |
| `/ground_check/overlay/compressed` | `sensor_msgs/CompressedImage` | กากบาทระยะสำหรับวัดเทียบพื้นจริง ใช้เฉพาะโหมดตรวจ |
| `/ground_check/status` | `std_msgs/String` JSON | พิกัดเมตร/พิกเซลของจุดที่มองเห็นและสถานะภาพของโหมดตรวจ |

`vision_ground` เป็น frame ท้องถิ่นของชุดกล้องทดลอง ไม่ใช่ `map`/`odom` และไม่ติดตามตำแหน่งหุ่นในโลก
ห้ามมี node อื่นส่ง TF ของ child `camera_optical_frame` ซ้ำขณะใช้ launch ทดลองนี้
การแปลงนี้แก้หน่วยและเลนส์ แต่ไม่ได้รับรองพื้นที่โล่งหรือ clearance ของ Husky

## การตรวจซอฟต์แวร์

ชุด unit tests ครอบคลุม ray–plane intersection,
การชดเชย distortion, camera rotation/offset, half-pixel resize, เครื่องหมายแกน Y,
การปฏิเสธ resolution ผิด และการนำเข้า YAML โดยไม่เขียนทับไฟล์เก่า
ตรวจใน official `ros:jazzy-perception` container เพิ่ม: สร้าง/อัปเดต workspace แยก,
colcon build, launch arguments และส่ง Path/TF ผ่าน DDS จริงด้วยข้อมูลเรขาคณิตสังเคราะห์
ตรวจ timestamp/พิกัดเมตร, ล้างเส้นเมื่อ invalid/stale/timeout และกลับมาส่งเมื่อข้อมูลใหม่ถูกต้อง
PDF เป็น vector A4 และ OpenCV ตรวจพบมุมภายในครบ 54 จุดจากภาพ render
โหมด `ground_check` ตรวจพิกัดด้วยสูตร ray ของกล้องก้ม 70°, การชดเชย distortion/rotation/offset
และส่งภาพผ่าน launch/JPEG/official image_view ใน ROS Jazzy ด้วยภาพสังเคราะห์และ Qt offscreen
ตรวจ frame/resolution/stale/future/timeout ว่าล้างตาราง และกลับมาแสดงเมื่อภาพใหม่ถูกต้อง
ยังไม่ได้ทดสอบการคาลิเบรต webcam ของผู้ใช้ ระยะภาคสนาม หรือหน้าต่าง RViz บนเครื่องผู้ใช้
