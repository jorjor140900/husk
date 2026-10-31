# ทดลองตัดหญ้าสีเขียวออกจากพื้นที่วิ่ง

โมเดล binary SegFormer อาจนับหญ้าเป็น drivable จากภาพบางแสง แม้หญ้าอีกฝั่งจะไม่ผ่าน
แมพ LiDAR ไม่ได้ระบุว่า grass/ทางเดิน และเส้นเหลืองในภาพไม่ได้รับรองพื้นที่ปลอดภัย
การเพิ่ม probability threshold อย่างเดียวไม่แก้กรณีโมเดลให้หญ้าคะแนนสูง

ชุดนี้เพิ่มตัวกรอง **สีเขียวจากภาพ BGR ดิบ** เป็น veto เสริม ไม่ใช่โมเดลจำแนกหญ้าใหม่
ใช้ HSV ร่วมกับ excess green แบบ normalized และขยาย veto 2 พิกเซลที่ภาพดิบ
ประมวลผลทั้งซ้ายและขวา ไม่กำหนดว่าฝั่งไหนต้องเป็นหญ้า
ใช้ก่อนรวม temporal history และบังคับตัดซ้ำหลังรวม จึงไม่คืนหญ้าจากเฟรมเก่า
ผลมีผลต่อ raw mask, seed-connected road mask, path, clearance และ segmentation packet
ที่ฉายเข้า AI costmap ของ Nav2 ไม่ได้เปลี่ยนเพียงภาพ overlay

## อัปเดต

ในโฟลเดอร์ Git เดิม:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py \
  --workspace ~/husky_vision_ws --update-code --update-detector
```

ต้องมี `--update-detector` เพราะปกติ updater เก็บโค้ด detector ใน workspace ไว้
flag นี้สำรอง detector/CLI เดิมก่อนเขียน detector, CLI และโมดูล vegetation ใหม่
ไม่เปลี่ยน visualization/ขนาดเส้น, โมเดล, calibration, mount หรือค่าที่วัดใน robot profile

หยุดเฉพาะ AI เดิม แล้ว build:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
```

คง webcam ที่ส่ง `/camera/image_raw` เปิดอยู่ แล้วเปิด AI:

```bash
ros2 launch husky_vision drivable_area.launch.py \
  use_webcam:=false image_topic:=/camera/image_raw \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=256 model_height:=160 cpu_threads:=2 \
  config_file:="$HOME/husky_vision_ws/src/husky_vision/config/outdoor-no-grass.json" \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera-active.yaml" \
  camera_mount:="$HOME/husky_vision_ws/robot/robot-mount.json"
```

บรรทัด Ready ต้องมี `green vegetation veto=True`
status มี `vegetation_veto.enabled=true`, จำนวนพิกเซลที่ตัด และ
`verified_grass_classification=false` เพื่อระบุขอบเขตของวิธีนี้ตามจริง
Nav2 รับ segmentation packet จาก topic เดิม ไม่ต้องเปลี่ยน costmap เพื่อเปิดตัวกรองนี้
คง `allow_motion:=false` ขณะตรวจและเทียบภาพ

## ตรวจภาพก่อนใช้

จอดหุ่นแล้วดูภาพดิบกับ overlay ในสถานการณ์เดียวกัน อย่าใช้ภาพ overlay ที่ถูกย้อมเขียว
เพื่อเลือกเกณฑ์สี เปิด viewer ทีละภาพเพื่อลดภาระ CPU:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/drivable_area/vegetation_veto -p image_transport:=raw \
  -p autosize:=false -p width:=960 -p height:=720
```

สีขาวคือถูก veto สีดำคือไม่มี veto สีเพิ่ม **ไม่ได้หมายความว่าขับได้**
หลังดู mask เปลี่ยน viewer เป็น overlay:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/drivable_area/overlay -p image_transport:=compressed \
  -p autosize:=false -p width:=960 -p height:=720
```

ตรวจหญ้าทั้งสองฝั่ง แดด/เงา และช่วงหญ้าบุกรอยต่อทางเดิน ดูว่าตัดหญ้าโดยไม่ตัด
พื้นทางเดินจนไม่มีทางอ้างอิง ห้ามใช้ผลทดสอบสีสังเคราะห์เป็นหลักฐานว่าภาคสนามผ่าน
ตัวกรองอาจตัดสีเขียวอื่น เช่นพื้นทาสี และอาจพลาดหญ้าแห้ง/เหลือง/มืดมากหรือถูกบัง
เกณฑ์เริ่มต้นใน `outdoor-no-grass.json` ยังต้องตรวจภาพดิบจริงของพื้นที่
หากไม่มีทางที่ผ่าน veto ระบบต้องรายงานไม่มี path แทนการลดข้อห้ามเพื่อให้วิ่งต่อ

ระยะ 2 พิกเซลเป็นเพียงการขยาย veto ในภาพ ไม่ใช่ระยะเผื่อขนาดหุ่นหน่วยเมตร
footprint/inflation, ความสดข้อมูล, localization, LiDAR และ permission gate ยังต้องผ่านแยกกัน
ตัวกรองนี้ไม่ทำให้ส่วน VLP16 runtime หรือการขับอัตโนมัติที่ยังค้างกลายเป็นพร้อมใช้

## ลดเวลา detector เมื่อใช้ Nav2

เมื่อใช้ Nav2 วางเส้นทางไปปลายทาง เส้นเหลืองในภาพเป็น diagnostic เท่านั้น
มี config `config/outdoor-nav2-fast.json` สำหรับข้าม pixel DP/curve search,
การ fit เส้นขอบ และ optical-flow history แต่ยังใช้ SegFormer, threshold เดิม,
green vegetation veto และ blocked mask เดิม ภาพ raw/calibration ยังคง 640×480
ไม่ลด input model, ขนาดหุ่น, sampling ของ ground grid หรือเกณฑ์อายุ 500 ms

อัปเดตด้วย `--update-code --update-detector` และ build ตามด้านบน หยุด/เปิดเฉพาะ
AI ใหม่ โดยเปลี่ยนบรรทัด config ของ launch เป็น:

```bash
  config_file:="$HOME/husky_vision_ws/src/husky_vision/config/outdoor-nav2-fast.json"
```

ไม่จำเป็นต้องหยุด navigation ที่กำลัง localize ได้แล้วเพื่อรับ mask แบบเร็ว:
format, timestamp และ topic ของ segmentation packet เหมือนเดิม
packet จากโหมดเร็วถูกทดสอบว่าเหมือนโหมดสร้างเส้นที่ใช้ segmentation/threshold
เดียวกันเมื่อปิด temporal history ทั้งคู่ โมเดลไม่ได้ถูกฝึกใหม่

บรรทัด Ready จะมี `green vegetation veto=True; image reference=False`
ในโหมดนี้ไม่มีเส้นเหลืองพิกเซล; overlay เดิมอาจแสดง
`NO PATH: nav2_segmentation_only` ซึ่งหมายถึงไม่ได้สร้าง reference ในภาพ
ไม่ได้หมายความว่า Nav2 หาเส้นทางไป goal ไม่ได้ สีเขียวแสดง segmentation ทุกส่วน
รวมถึงส่วนที่ไม่เชื่อมกับ seed; Nav2 ใช้ full segmentation เช่นนี้อยู่แล้ว
ไม่ publish path หรือ clearance ที่อ้างว่าผ่านการตรวจโดยไม่มีการคำนวณ
อย่าใช้ config นี้กับ controller แบบเก่าที่ต้องรับเส้นเหลือง/ground reference

เริ่มวัดโดยปิด image_view/RViz ไว้ก่อนและคง backend ทำงานอยู่ รัน preflight
เดิมอีกครั้ง แล้วดู `Frame age`, `inference` และ `detector` จาก AI
ในการทดสอบภาพสังเคราะห์บน host นี้ detector แบบเดิมมี median ประมาณ 135 ms
ส่วนโหมดเร็วประมาณ 3 ms (ไม่รวม inference/ROS/ground projection)
ตัวเลขนี้ไม่ใช่ผลวัดบนคอมภาคสนาม ต้องยืนยัน delivery budget อีกครั้ง

เวอร์ชันใหม่ของ `vision_grid` เพิ่ม `timings_ms` ใน `/husky_nav/vision_status`:
decode, อายุ source ตอนรับ, capture TF, projection, callback และอายุ source ตอนส่ง
ช่วยแยกว่าช้าที่ inference/detector หรือการฉาย grid หากยังเกินเกณฑ์
node vision_grid ที่เปิดค้างก่อนอัปเดตต้องเริ่มใหม่จึงจะแสดง timing เพิ่มเหล่านี้
แต่ไม่จำเป็นต้องเริ่มใหม่เพียงเพื่อใช้ mask จาก config เร็ว

## แสดงเส้นทาง Nav2 บนภาพกล้อง โดยไม่กลับไปสร้างเส้นพิกเซล

### คงภาพและเส้นเหลืองแบบเดิม โดยแยกงานจอออกจาก Nav2

หากต้องการเส้นเหลืองจากภาพเหมือนเดิม **แม้ Nav2 ยังไม่มี goal plan**
ให้เลือก `async_reference_preview:=true` แทน `show_nav2_path`
โหมดนี้ทำ inference เพียงครั้งเดียวต่อ capture แล้วส่ง segmentation ปัจจุบัน
ไป Nav2 ก่อน ส่วน optical flow/DP/curve/boundary ของจอไปทำใน process อีกตัว
ที่ priority ต่ำกว่า (`nice +10` บน Linux) และ OpenCV หนึ่ง thread
คง detector config เดิมสำหรับจอ ไม่เปลี่ยน visualization/ความหนาเส้น/calibration/mount
แต่ละ overlay ใช้ภาพกับ probability จาก capture เดียวกัน ไม่เอาเส้นเก่าทับภาพใหม่
เมื่อมี calibration/mount จุดเริ่มเส้นเหลืองจะใช้ตำแหน่งแนวกลางหุ่น Y=0
ที่แถวเริ่มเดิม (`seed_y_fraction`) แทนการเริ่มกลางภาพเสมอ ซึ่งผิดเมื่อกล้องติดเยื้อง
ตัวอย่าง mount ที่วัดก่อนหน้า: ภาพ 640×480 แถว 450 เดิมเริ่ม x=320 แต่แกนหุ่นอยู่ x≈408
ไม่ปรับแถวเริ่ม/threshold/clearance เพื่อหลบ mask ที่ปิดทาง
ถ้าจุดเริ่มจริงถูก veto หรือ clearance ไม่พอ ยังคงไม่มีเส้นและแสดง X แดงเล็กที่จุดนั้น
ถ้าแกนจริงอยู่นอกภาพ จะรายงาน `calibrated_reference_start_not_visible`
และไม่ใช้จุดกลางภาพแทน ข้อมูลจุดเริ่มอยู่ที่ `reference_preview.last_display.start`

หยุด AI เดิม อัปเดต `--update-code` และ build ตามด้านบน แล้วเปิด:

```bash
ros2 launch husky_vision drivable_area.launch.py \
  use_webcam:=false image_topic:=/camera/image_raw \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=256 model_height:=160 cpu_threads:=2 \
  config_file:="$HOME/husky_vision_ws/src/husky_vision/config/outdoor-no-grass.json" \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera-active.yaml" \
  camera_mount:="$HOME/husky_vision_ws/robot/robot-mount.json" \
  show_nav2_path:=false async_reference_preview:=true reference_preview_fps:=2.0
```

webcam/navigation/image_view เปิดค้างไว้ได้สำหรับการเปลี่ยนโหมด AI นี้
main process ใช้ full-resolution current threshold + grass veto โดยไม่ทำ temporal flow
เหมือน fast Nav2 config ส่วนจอยังใช้ temporal/reference settings เดิมทั้งหมด
mask ที่ inference ช้าจน capture เกิน 500 ms ยังเป็น invalid ไม่เปลี่ยน timestamp
และ vision_grid/costmap ยังตรวจอายุซ้ำเหมือนเดิม
ไม่ต้องเพิ่มเกณฑ์อายุ Nav2 ให้เท่ากับเวลาประมวลผลจอ

worker เก็บได้เพียงหนึ่งงานกำลังทำ หนึ่งเฟรมล่าสุดรอทำ และหนึ่งผลล่าสุด
เฟรมรอที่ถูกแทนจะถูกทิ้ง ไม่มีคิวสะสม ไม่มี inference ซ้ำหรือ model อีกชุด
ไม่มี viewer ก็ไม่เริ่ม worker และไม่ส่งงานเส้นเหลืองเพิ่ม
เมื่อกล้องเปลี่ยน/ย้อนเวลา/timeout จะ reset generation เพื่อไม่รับผลเก่าจาก worker
ปิด AI แล้วปิดเฉพาะ process ที่ AI สร้างเองด้วย

ภาพจอยังคงใช้เกณฑ์อายุเดิม (`max_frame_age_sec` เริ่มต้น 2 วินาที)
จึงอาจอัปเดตช้ากว่า mask ที่ส่งให้ Nav2 แต่ไม่สลับระหว่าง raw/overlay ทุกครั้งที่ mask
พ้นเกณฑ์ 500 ms ถ้าเส้นใน mask ทำไม่ได้จริง จะแสดงเหตุผลเดิมของ detector
ไม่ได้สร้างเส้นเหลืองปลอมให้ผ่านอุปสรรค เส้นนี้ยังเป็น **image reference สำหรับจอ**
ไม่ใช่แผนจาก goal หรือคำสั่งมอเตอร์ ในโหมดนี้ `/drivable_area/status` และ
`reference_pixels` ไม่เผยแพร่เส้นจาก worker เป็นข้อมูลควบคุม
ข้อมูลจอและ worker อยู่ใน `reference_preview` ของ status; timing หลักเป็น inference/fast segmentation

การแยก process ลดการรอ DP แต่ยังใช้ CPU ร่วมกัน และไม่รับรองว่า inference/transport
จะผ่าน 500 ms บนเครื่องภาคสนาม ต้องวัดด้วย preflight อีกครั้ง โดยคง `allow_motion:=false`
การแก้ latency นี้ไม่แก้กรณีโหลด outdoor map ขณะหุ่นอยู่ในห้อง:
ต้องใช้แผนที่ของพื้นที่จริงและตรวจ scan-to-map alignment ก่อนเลือก goal

ทดสอบ ROS2 แยกเครือข่ายโดยใช้ model/กล้อง/TF จำลอง และ vision_grid เป็น process
แยกเหมือน launch จริง จงใจหน่วง pixel planner ใน worker เพิ่ม 650 ms:
ได้ segmentation 85 captures, ภาพแสดง 10 เฟรมที่มีเส้นเหลืองครบ และ veto grid 81 เฟรม
segmentation delivery p95 ประมาณ 184 ms ใน fixture นี้ พร้อม original capture stamps
และไม่มี command publisher ตัวเลขนี้ไม่ใช่ผลวัดคอมภาคสนาม

### เส้นใน RViz อ้อม ทั้งที่ทางเดินจริงตรง

ตรวจ `/husky_nav/goal_status` ก่อน ถ้าเป็น `path_ready_preview_only` แปลว่าสร้างแผนได้แล้ว
เส้นจาก `/plan` ที่ planner_server เผยแพร่เป็นแผน Nav2 ส่วนเส้นเหลืองแบบ pixel reference
ใช้ mask ภาพและจุดเริ่มของตัวเอง จึงไม่จำเป็นต้องมีรูปร่างตรงกัน
หาก goal อยู่ข้างหน้าบนทางตรงและต้นทุนต่ำ แนวควรใกล้ตรง แต่ Navfn พิจารณาต้นทุนสะสม
จึงอาจอ้อมบริเวณ inflation/สิ่งกีดขวาง/AI veto แม้ saved map จะยังดูเป็นสีขาว

หลังเลือก goal ใหม่ ใช้ตัวตรวจแบบอ่านอย่างเดียว:

```bash
ros2 run husky_vision navigation_plan_check --duration 4
```

อ่าน `/husky_nav/goal_plan`, `/global_costmap/costmap` และ `/map` โดยไม่ส่ง goal,
คำสั่งขับ บริการ หรือแก้ parameter รายงานความยาวแผน ระยะตรง อัตราส่วนการอ้อม
ระยะเบี่ยงด้านข้างสูงสุด และตัวอย่างพิกัดช่องต้นทุนที่ปิดแนวตรง
`blocked_cells` นับ cost ≥99 (inscribed/lethal), `soft_cost_cells` เป็น cost 1..98
และ `blocked_but_free_in_saved_map` ชี้ว่าข้อห้ามมาจาก layer ที่เพิ่มเหนือ saved map
ยังต้องแยกด้วยภาพ costmap/AI veto/scan ว่าเป็นข้อมูลจริงหรือ projection/localization ผิด
`unknown_cells` กับ `outside_costmap_cells` แยกไว้ ไม่ถือว่า free
การสุ่มแนวตรงห่างกันครึ่งเซลล์เป็น diagnostic เท่านั้น ไม่ใช่ swept-footprint collision check
และ timestamp ของ saved map เป็นเวลา static map ไม่ใช่เงื่อนไขความสดเหมือน mask

ใน RViz เปิด `Global planning costmap` และ `AI veto mask` เพื่อเทียบตำแหน่งหุ่น/goal
กับบริเวณที่อ้อม ตรวจ laser-to-map alignment และทิศหน้าหุ่นให้ตรงของจริง
แมพ LiDAR ที่เป็นพื้นที่ขาวกว้างไม่ได้ระบุสนามหญ้าทั้งหมดว่าเป็นพื้นที่ห้ามเข้า
mask กล้องเป็น veto เฉพาะบริเวณที่เห็น จึงไม่รับรองเส้นทาง global นอกมุมกล้องว่าเป็นทางเดิน
ถ้าแผนออกนอกทางเดินจริง ต้องแก้ขอบเขต traversability/keepout ของพื้นที่นั้นก่อนเคลื่อนที่
ไม่บังคับลากเส้นตรง หรือปิด layer/ลด footprint เพียงเพื่อให้รูปดูตรง

ถ้า p95 vision ยังเกิน 500 ms เล็กน้อย ทดลองลดเฉพาะ `reference_preview_fps:=1.0`
แล้ววัด preflight เดิมใหม่ อัตรานี้จำกัดงานจอ ไม่ลดขนาดโมเดล ไม่เปลี่ยนขนาดหุ่น/threshold
และไม่ขยาย deadline ของ Nav2 ค่า input model และจำนวน CPU threads ใช้เดิมก่อนเปรียบเทียบ

### ปรับระยะเผื่อเฉพาะด้านข้าง

`footprint.margin_m` เดิมเป็นค่ารอบตัว ถ้าต้องเปลี่ยนเฉพาะข้างให้เพิ่ม
`footprint.lateral_margin_m` เป็นจำนวนบวก หน่วยเมตร ถ้าไม่ใส่จะใช้ margin_m
ทั้ง Nav2 footprint, stop-sweep sampling, collision bounds และ marker ใช้ค่าเดียวกัน
ROS safety node ใช้ค่าเริ่มต้น `lateral_margin_m=0` เพื่อสืบทอด margin_m เดิม
โดย robot profile ที่ระบุ field นี้ต้องใส่ค่าบวกชัดเจน ไม่ใช้ 0 เป็นระยะเผื่อจริง

ตัวอย่างที่ผู้ใช้ขอ: หุ่นจริงกว้าง .66 m, หน้า .50 m, หลัง .56 m,
margin_m=.15 และ lateral_margin_m=.08 จะได้ Nav2 footprint
`[[.65,.41],[.65,-.41],[-.71,-.41],[-.71,.41]]` กว้าง .82 m
ไม่แก้ half_width_m=.33 หรือระยะหน้า/หลังที่วัดจริง ระยะหน้า/หลังยังเผื่อ .15 m
ตัวตรวจ swept geometry เพิ่ม sample spacing .01 m ตามเดิม จึงกว้าง .84 m ใน marker
inflation_radius ยัง .81 m เพราะขนาดแนวยาวเท่าเดิม เป็นรัศมีแผ่ต้นทุนรอบนอก
ไม่ใช่ความกว้างระยะเผื่อด้านข้างโดยตรง จึงไม่ตั้ง inflation_radius=.08 แทน field นี้

หยุด navigation, อัปเดต source ด้วย `--update-code`, build และ source workspace ตามด้านบน
ไม่ต้องหยุด webcam/AI เพื่อเปลี่ยน margin นี้ แล้วสำรองและแก้ profile จริงบนคอม Jazzy:

```bash
python3 - <<'PY'
from pathlib import Path
from datetime import datetime, timezone
import json
import shutil

p = Path.home()/"husky_vision_ws/robot/robot.json"
data = json.loads(p.read_text())
backup = p.with_name("robot-before-side08-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")+".json")
shutil.copy2(p, backup)
data["footprint"]["lateral_margin_m"] = 0.08
p.write_text(json.dumps(data, indent=2)+"\n")
print("Backup:", backup)
print("Footprint:", data["footprint"])
PY
```

เปิด navigation ด้วย robot.json เดิมและ `allow_motion:=false`, ตั้ง 2D Pose Estimate ใหม่
ตรวจ footprint แล้วเลือก goal ใหม่เพื่อให้เทียบกับ costmap ล่าสุด:

```bash
ros2 param get /global_costmap/global_costmap footprint
ros2 param get /husky_control/laser_safety lateral_margin_m
ros2 run husky_vision navigation_plan_check --duration 4
```

กรณีทางจริงกว้าง 1.20 m แม้ margin ข้างเดิม .15 ก็มีความกว้างเหลือ .24 m
การอ้อมมากจึงไม่ได้พิสูจน์ว่า margin เป็นสาเหตุเดียว การเปลี่ยนเป็น .08 เป็นการทดลอง
วางแผนตามที่ผู้ใช้เลือก ไม่ใช่การรับรองความปลอดภัยในการวิ่ง ต้องวัดความกว้าง/ความคลาดเคลื่อนจริง
และแยกว่าจุดต้นทุนผิดมาจาก AI, LiDAR หรือ projection/localization ก่อนเปิดเคลื่อนที่

ถ้าภาพ overlay ดูเป็นทางว่างแต่ costmap ปิดทาง ให้ตรวจมาสก์ปัจจุบันที่ส่ง Nav2:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/drivable_area/mask -p image_transport:=raw \
  -p autosize:=false -p width:=640 -p height:=480
```

สีขาวคือ drivable ที่ผ่าน threshold ปัจจุบัน สีดำบนพื้นคือ veto/non-drivable
ส่วนเหนือ horizon เป็นส่วนที่ไม่ได้วิเคราะห์ ในโหมด async ภาพ overlay แบบเดิมยังมี temporal
history สำหรับจอ แต่มาสก์ของ Nav2 ใช้เฟรมปัจจุบัน จึงอาจเห็นต่างกัน โดยเฉพาะบริเวณคะแนนใกล้ threshold
เปิด `AI veto mask` ใน RViz และซ่อนเฉพาะ display `Global planning costmap` ชั่วคราว
เพื่อดู veto ก่อน inflation โดยไม่ปิด layer ใน Nav2 หากพิกัด veto อยู่ผิดเมื่อเทียบกับภาพ/ทางจริง
ต้องตรวจ calibration/mount/TF; หากมาสก์ปัจจุบันมีช่องดำบนพื้นจริง ต้องแก้การจำแนก
หาก veto ไม่ปิดบริเวณนั้นแต่ costmap มีต้นทุน ให้ตรวจ scan/obstacle layer ต่อ
ไม่ลบจุดหรือเติมรู mask อัตโนมัติเพียงเพื่อให้ได้เส้นตรง

### วัดช่องทางที่ฉายและ goal เทียบกับแนวหน้าหุ่น

หาก mask ภาพขาวต่อเนื่อง แต่ยังไม่รู้ว่าช่องว่างในแมพแคบ/เยื้อง/goal อยู่นอกแนว
ให้รันตัวตรวจจาก **โฟลเดอร์ Git เดิม** ใน shell ที่ source Jazzy และ venv แล้ว:

```bash
git pull --ff-only origin work
python3 tools/check_corridor_geometry.py \
  --profile "$HOME/husky_vision_ws/robot/robot.json"
```

ไม่ต้อง build, export, หยุด webcam/AI/navigation หรือเปลี่ยน profile สำหรับคำสั่งนี้
ตัวตรวจ subscribe segmentation packet และ goal, อ่านค่า footprint ผ่าน GetParameters,
และ lookup TF ณ capture time เท่านั้น ไม่มี command/goal publisher หรือการเขียน parameter
ใช้ calibration/mount ใน profile ที่ระบุ พร้อมตรวจ frame/resolution และอายุ capture <=500 ms

รายงานหน้าตัดที่อยู่หน้าแกนหุ่น 0.75, 1, 1.5, 2 และ 2.5 เมตร ตามสมมติฐานพื้นราบ:

- `width_around_robot_axis_m`: ความกว้างช่วง drivable ที่คร่อมแกน Y=0 ถ้าแกนถูก veto/มองไม่เห็น
  จะเป็น null ไม่ย้ายไปเลือกช่วงว่างข้าง ๆ แทน
- `drivable_intervals_y_m` และ `interior_veto_holes`: แสดงทุกช่วงขาวและช่วงดำที่ถูกขาวขนาบ
  จึงแยกได้ว่า width เช่น .80 m เป็นช่องว่างที่ถูกจุดดำเล็กตัด หรือขอบทางทั้งหมดแคบจริง
  ไม่ตีความ width ของช่วงเดียวเป็นความกว้างทางเดินจริงโดยอัตโนมัติ
- หาก `colour_veto_matches_capture=true` จะเทียบ `/drivable_area/vegetation_veto` ที่ stamp เดียวกัน:
  `colour_veto_present` บอกว่าตัวกรองสีมีส่วน veto ช่วงนั้นได้ ส่วน
  `model_veto_without_colour_samples` นับจุดที่ถูกปฏิเสธโดย segmentation โดยไม่มี colour veto
  ไม่ลบ/เติมช่วงดำ และไม่ได้รับรองว่าเป็น false positive หรือสิ่งกีดขวางจริง
- `corridor_center_y_m` และขอบซ้าย/ขวา: ดูว่าตัวหุ่นเยื้องจากกลางทางที่ตรวจพบหรือไม่
- `goal_ground_xy_m`: goal ในแกนหุ่น ณ เวลาภาพ x=หน้า, y=ซ้าย, หน่วยเมตร
  พร้อมมุม `goal_angle_from_robot_forward_deg`
- `goal_line_y_m` และ `reserved_width_fits_at_goal_line`: ดูว่าแนวตรงไป goal
  ยังมีความกว้างพอใน mask หรือเลี้ยวออกไปขอบทาง/หญ้า
- `runtime_costmap_reserved_width_m` และ `runtime_width_matches_profile`:
  แยกว่า profile ใหม่ถูกโหลดเข้า costmap แล้วหรือยัง

ถ้า `goal_ground_xy_m=null` ให้อ่าน `goal_projection_error` และอายุ
`latest_goal_transform_age_ms` ก่อน ยังสรุปไม่ได้ว่า goal อยู่ตรงแนวหุ่น
ตัวตรวจไม่ใช้ latest TF แทน capture-time TF สำหรับการแปลง goal

ความกว้างนี้อนุมานจาก RGB และเรขาคณิตที่ตั้งไว้ ไม่ใช่การวัดแทนตลับเมตร และไม่ตรวจ
swept footprint/ความสูงสิ่งกีดขวาง หากต่างจากทางจริง 1.20 m ให้แยก mask boundary
กับ calibration/mount/พื้นเอียงก่อนปรับเรขาคณิต ไม่ตั้ง scale ให้ผลตรงโดยไม่มีจุดตรวจอิสระ
บริเวณนอกภาพหรือไม่วิเคราะห์ยังเป็น unobserved ไม่ถือว่าขับได้
`capture_still_fresh` กับ source age บอกความสด ณ รายงาน ถ้า false ต้องเก็บใหม่
ก่อนใช้ตีความสภาพปัจจุบัน รายงานนี้ไม่อนุญาตการเคลื่อนที่

### แสดงแผนไปยัง goal บนกล้อง

คนดูยังต้องเห็นทิศทางที่ระบบตั้งใจไป โหมดเร็วจึงเปิดการฉาย **แผนจาก Nav2**
ลงบน overlay ได้ เส้นเหลืองนี้มาจาก path ไปยังปลายทางจริง ไม่ได้สร้างจากการหา
กลาง mask ขึ้นมาใหม่ ใช้ TF จากกรอบ path ไป ground ณ **เวลาถ่ายภาพ** แล้วฉาย
ด้วย calibration/mount เดิม สำหรับพื้นราบและ Nav2 แบบ 2D

อัปเดต source/build ตามด้านบน คง navigation ที่ active อยู่ไว้ หยุดและเปิด AI ใหม่:

```bash
ros2 launch husky_vision drivable_area.launch.py \
  use_webcam:=false image_topic:=/camera/image_raw \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=256 model_height:=160 cpu_threads:=2 \
  config_file:="$HOME/husky_vision_ws/src/husky_vision/config/outdoor-nav2-fast.json" \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera-active.yaml" \
  camera_mount:="$HOME/husky_vision_ws/robot/robot-mount.json" \
  show_nav2_path:=true nav2_path_topic:=/husky_nav/goal_plan overlay_max_fps:=2.0
```

เปิด image_view ของ `/drivable_area/overlay` แบบ compressed ตามคำสั่งด้านบน
หลังเลือก **2D Goal Pose** ใน RViz และได้ `path_ready_preview_only` จะเห็นเส้น
เหลืองของ goal plan บนภาพกล้อง เฉพาะช่วงที่อยู่ในมุมมองและไม่เกิน 8 เมตรตามแนวแผน
จุดเริ่มวาดเป็นจุดใกล้หุ่นบนแผน ไม่สร้างเส้นเชื่อมจากหุ่นไปเส้นทางที่ไม่มีอยู่จริง
เส้นน้ำเงินยังใช้ตำแหน่งกลางหุ่นที่คาลิเบรตไว้ และขนาดภาพยังเท่าเดิม

ก่อนเลือก goal จะแสดงว่ากำลังรอ goal/plan ไม่มีเส้นทางปลอมให้ดู
แผนที่ยังไม่สำเร็จ, goal ใหม่ที่ยังไม่มีแผนใหม่, heartbeat ของ goal ขาดเกิน 2 วินาที,
แผนอายุเกิน 120 วินาที, camera frame/ขนาดไม่ตรง, ภาพเกิน 500 ms หรือไม่มี TF
จะไม่แสดงเส้นเก่าแทน เส้นที่อยู่นอกมุมมองแยกด้วยช่องว่าง ไม่ลากข้ามส่วนที่มองไม่เห็น
เมื่อกล้องขาดหรือผลเก่า จะคง **ภาพกล้องดิบล่าสุด** พร้อมคำเตือนและอายุ capture
ล้าง mask/เส้นจากภาพนั้น ไม่ส่งจอดำสลับกับกล้อง โดยคง timestamp ของภาพที่แสดงจริง
ไม่ได้ตั้ง timestamp ใหม่ให้ภาพดูสด หากยังไม่เคยได้รับภาพจะไม่สร้างภาพดำขึ้นมา
สถานะ mask/control ยังคง invalid จนได้ข้อมูลใหม่ที่ผ่านเกณฑ์

ถ้า TF ที่เวลาถ่ายภาพยังมาไม่ถึง จะเก็บเฉพาะ packet ล่าสุดและลองใหม่ทุก 20 ms
โดยไม่บล็อก callback รับ TF ทั้งการสร้าง veto grid และการฉายเส้นบนภาพ
ยังต้องได้ **TF ของ capture เดิม** และส่งผลก่อน source อายุเกิน 500 ms
ไม่ใช้ latest TF แทน ไม่แก้ timestamp และไม่ขยาย timeout
สถานะ `waiting_for_capture_tf` ระหว่างรอยังเป็น invalid สำหรับ grid
ถ้า TF ไม่มาทันจะทิ้ง pending frame ตามเกณฑ์เดิม
การรอ TF ไม่ได้รับรองว่า CPU/DDS จะส่งทันทุกเฟรม ผู้รับยังตรวจอายุ source ซ้ำ
เพราะ packet ที่ส่งทันอาจมาถึงหลัง 500 ms ได้ โดย preflight/layer ยังต้องปฏิเสธตามเดิม

หัวภาพระบุ `NAV2 PLAN` พร้อมอายุแผน และ `PLAN DISPLAY ONLY`
นี่คือแผนที่ controller จะใช้ติดตาม ไม่ใช่ภาพรับรองว่ากำลังมีคำสั่งมอเตอร์หรือปลอดภัย
ถ้า bridge/gate ยัง disabled หุ่นยังไม่เดิน สำหรับโหมด NavigateToPose ในอนาคต
เลือก `nav2_path_topic:=/plan` เพื่อดู global plan ของ Nav2 แทน preview action
ยังต้องตรวจ local collision filter และคำสั่ง v/omega แยกจากภาพแผน

mask ส่งไป Nav2 **ก่อน** วาด/บีบอัด overlay และ `overlay_max_fps` จำกัดเฉพาะ
ภาพที่แสดง ไม่ลดอัตรา inference หรือ segmentation packet ไม่มีผู้ดูภาพก็ไม่ฉาย
path หรือบีบอัด JPEG ใน mode นี้ไม่เพิ่ม publisher คำสั่งเคลื่อนที่
ข้อมูลภาพแสดงอยู่ใน `nav2_camera_path` ของ `/drivable_area/status`

บนภาพสังเคราะห์ 640×480 ของ host ทดสอบ projection/drawing ใช้ median ประมาณ 1 ms
(ไม่รวม TF, ROS transport และ JPEG) ยังต้องวัดภาคสนามหลังเปิด viewer
ทดสอบ ROS2 แยกเครือข่ายโดยใช้กล้อง/แผน/TF/model จำลองแล้วว่าแสดงเส้นได้,
goal ใหม่ไม่ค้างเส้นเก่า, กล้องหยุดล้างภาพเส้น และ mask ยังส่งเร็วกว่าภาพแสดง
การทดสอบนี้ไม่ยืนยัน geometry จริงหรือการขับหุ่น

การแก้ TF retry ต้อง **หยุด navigation และ AI เดิม**, อัปเดต/build ตามด้านบน
แล้วเปิด navigation ใหม่ด้วย profile/map เดิมและ `allow_motion:=false`
คง webcam, ROS1 gateway และ SSH tunnel ไว้ จากนั้นตั้ง **2D Pose Estimate** อีกครั้ง
เปิด AI ด้วยคำสั่งข้างบน และเลือก goal ใหม่เมื่อ localization/vision กลับมาสด
ไม่เปิด vision_grid เพิ่มซ้อนกับตัวที่ navigation launch เปิดอยู่

### กด goal แล้วไม่มีเส้นเหลือง

ตรวจสอง topic ก่อน เพื่อแยกกรณีไม่มีแผนออกจากฉายแผนไม่ได้:

```bash
ros2 topic echo /husky_nav/goal_status --once --full-length
ros2 topic echo /drivable_area/status --once --full-length --field data
```

`path_ready_preview_only` จึงมี goal plan ให้ฉาย ถ้า `goal_failed` จะไม่มีแผนให้วาด
โค้ดใหม่ส่ง `error_name` (อ่านจาก constants ของ action ที่ติดตั้งจริง) และ
`error_msg` จากผล action เพิ่ม และแสดงสาเหตุในหัวภาพแทนเส้นที่ไม่มีอยู่จริง
กรณีภาคสนามที่พบรหัส **207** พร้อม log `Costmap timed out waiting for update`
คือ planner รอ costmap ที่ current ไม่ทัน ต้องแก้ความสดของ mask/TF/ข้อมูลนำเข้า
ก่อนสรุปว่าปลายทางผิดหรือเส้นทางถูกกีดขวาง ไม่เพิ่ม timeout เพื่อให้ข้อมูลเก่าผ่าน
พิกัดเริ่ม `(0,0)` ใน warning ที่ล้มก่อนมี start pose ไม่ใช่หลักฐานว่าหุ่นอยู่ origin
ค่า `nav2_camera_path` จะคงเหตุผลของภาพแสดงล่าสุดไว้แม้เฟรม inference นี้ไม่ได้วาด
เพราะติด display rate cap จึงไม่ถูกเขียนทับเป็น `viewer_not_rendered` ทุกเฟรม

หลัง vision/TF สดแล้ว ทดลอง goal ใกล้หุ่นประมาณ 1–2 เมตรบนกลางทางเดินจริง
ตรวจตำแหน่งหุ่นและแนว LiDAR ให้ตรง map ก่อน ถ้าข้อมูลสดแล้วแต่ planner รายงาน
`NO_VALID_PATH` ให้ดู **global costmap** ประกอบ เพราะพื้นที่สีขาวใน saved map อาจถูก obstacle/AI veto
หรือ inflation ปิดทางใน costmap ได้ ใน RViz เปิด display `Global planning costmap`
(ค่าเริ่มต้นปิด) หรือ Add → Map → `/global_costmap/costmap`, เลือก color scheme costmap
และวาง goal ในพื้นที่ที่เชื่อมต่อกับหุ่นได้จริง ไม่ล้าง obstacle/ปิด AI/ลด footprint
เพื่อให้ตัวเลขผ่านโดยไม่มีข้อมูลรองรับ ถ้าตัวกรองหญ้า veto พื้นทางเดินจริง ต้องแก้การจำแนก
จากภาพดิบก่อน ไม่เลื่อน goal หลบข้อผิดพลาดโดยไม่ตรวจบริเวณนั้น

### ทำไม p95 ผ่าน แต่ Planning inputs ready ยัง False

preflight ตรวจทั้ง p95 ของช่วงสังเกต และ **mask ล่าสุด ณ เวลาตรวจ**
เช่น p95=482 ms ผ่าน แต่เฟรมท้ายอายุ 575 ms ต้องขึ้น PENDING ตามเกณฑ์ 500 ms
การผ่าน p95 ไม่ได้แปลว่าทุกเฟรมผ่าน และไม่ได้อนุญาตใช้เฟรมเก่าขับต่อ
รายงานใหม่แสดง reason และ source_age_ms ของตัวล่าสุดเพื่อช่วยแยกกรณีนี้
ต้องลด latency/ความแกว่งให้มี mask ที่สดต่อเนื่องก่อนเปิดเคลื่อนที่จริง

หากต้องคืนพฤติกรรมเดิม ให้หยุด AI และเปิดใหม่โดยใช้ config เดิม
ค่าเริ่มต้นของ `reject_green_vegetation` เป็น false ไม่ต้องเปลี่ยนโมเดลกลับ

## ลดจุดห้ามผ่านจากสีเขียวกระจัดกระจายบนพื้น (ทดลองแบบ planning-only)

ผลภาคสนามแยกสาเหตุได้บางส่วน: เส้นตรงมี 20 ช่อง blocked ทั้งที่ saved map ว่าง
และเมื่อปิด colour veto ชั่วคราว ช่อง blocked เหลือ 0 ความยาวแผนลดจากประมาณ
10.26 m เป็น 3.20 m จึงต้องแก้หลักฐานสีที่ใส่เข้า costmap ก่อนปรับ planner
การปิดตัวกรองทั้งหมดไม่ใช่ค่าขับจริง เพราะหญ้าที่โมเดลมองเป็นพื้นจะไม่ถูกตัดแล้ว
ภาพ overlay ที่ระบายเขียวไม่สามารถใช้ปรับเกณฑ์สีของภาพดิบได้

มี config ทดลอง `config/outdoor-supported-green-preview.json` เพิ่ม spatial support:

- หา candidate สีเขียวจาก **ภาพดิบก่อน resize** ตามเกณฑ์สีเดิม
- แต่ละกลุ่มสีเขียวที่เชื่อมกันต้องมีอย่างน้อยหนึ่งพิกเซลที่บริเวณ 7×7 รอบตัว
  เป็น candidate ไม่น้อยกว่า 50% ก่อนจะยอมรับเป็น colour veto
- ถ้ากลุ่มผ่าน จะเก็บทั้งกลุ่มรวมขอบและใบหญ้าเส้นบางที่เชื่อมกับกลุ่มไว้ แล้วจึง
  dilate 2 px ตามเดิม ไม่กัดขอบหญ้าด้วย opening และไม่เติมช่องดำใน segmentation
- การกรองนี้ใช้กับ **สีเท่านั้น** โมเดลที่ปฏิเสธพื้นหรือ blocked mask ที่ระบุชัด
  ยังคงเป็นข้อห้าม แม้อยู่ตรงจุดสีเขียวที่ถูกตัดออกจาก colour veto

เป็น heuristic ทดลอง ไม่ใช่ตัวจำแนกหญ้าที่ตรวจสอบแล้ว อาจพลาดต้นหญ้าหรือวัตถุเขียว
เล็ก ๆ ที่แยกเดี่ยว และยังแก้จุดเขียวใหญ่/เงาสีเขียวที่ผ่าน spatial support ไม่ได้
ค่าเริ่มต้นและ config เดิมไม่เปลี่ยน (`vegetation_support_radius_px=0` ปิด support)
ต้องเทียบกับภาพดิบหน้างานก่อนเลือกใช้ ไม่มีการลด footprint หรือปิด LiDAR

อัปเดตบนคอม Jazzy ได้ขณะหุ่นปิด:

```bash
cd ~/husk-live
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py \
  --workspace ~/husky_vision_ws --update-code --update-detector
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
```

เมื่อทดสอบครั้งหน้า ใช้คำสั่ง AI เดิม แต่เปลี่ยนเฉพาะ config เป็น:

```bash
config_file:="$HOME/husky_vision_ws/src/husky_vision/config/outdoor-supported-green-preview.json" \
show_nav2_path:=false async_reference_preview:=true reference_preview_fps:=1.0
```

ยังเก็บเส้นเหลืองจากภาพผ่าน worker เดิมไว้ เส้นนั้นเป็น reference ไม่ใช่แผน goal
การอัปเดตนี้ไม่เปลี่ยน visualization/calibration/mount/robot profile หรือเปิดมอเตอร์
ใน `/drivable_area/status` ตรวจ `vegetation_veto.support_radius_px=3` และ
`support_fraction=0.5` เพื่อแยกจาก config เดิม รัน navigation ด้วย
`allow_motion:=false` แล้วเลือก goal เดิมใหม่ก่อนเทียบ เพราะเส้นเก่าไม่ replan เอง

ถ้ามีภาพดิบก่อน overlay ที่บันทึกไว้ สามารถเทียบ colour veto แบบ offline
โดยไม่เปิด ROS/โมเดล/หุ่น (เลือก output directory ใหม่):

```bash
cd ~/husk-live
python3 tools/compare_vegetation_veto.py \
  --image /path/to/raw-camera.png \
  --output outputs/green-support-comparison
```

`colour-before.png` / `colour-supported.png` เป็น veto ก่อน/หลัง และ `changes.png`
แสดงสีแดง=คง veto, ฟ้า=จุด colour-only ที่ถูกเอาออก เครื่องมือนี้ไม่วัด semantic
accuracy และไม่สร้างเส้นทาง จึงยังยืนยันว่าแก้ภาพภาคสนามนี้แล้วไม่ได้
เมื่อมีข้อมูลสด ให้เทียบ `navigation_plan_check` และ `check_corridor_geometry.py`
หลังเลือก goal เดิมใหม่ โดยดูทั้งการคงขอบหญ้าและการหายของจุดห้ามผ่านเทียม
ไม่บังคับเส้นตรงผ่านข้อห้ามหรือเปลี่ยน unknown นอกกล้องให้เป็นทางปลอดภัย

## ระยะยาว

เก็บภาพดิบของทางจริงจากตำแหน่งกล้องเดิม ทั้งแดด เงา ทางแยก และหญ้าทั้งสองด้าน
label หญ้าเป็น non-drivable และทางเดินเป็น drivable แล้ว fine-tune โมเดล
แยก validation ตามช่วง/บริเวณการเดิน ไม่ใช้เฟรมติดกันเป็นหลักฐานทดสอบอิสระ
ประเมิน false drivable บนหญ้าแยกจาก pixel accuracy รวม เพราะพื้นที่ทางเดินมากอาจกลบความผิดพลาด

ทดสอบอัตโนมัติครอบคลุม veto ทับผล AI ความมั่นใจสูง, temporal history,
การ downsample, blocked mask อื่น, packet ไป Nav2 และการ backup ตอนส่งออก
บน host ทดสอบ ตัวกรองอย่างเดียวที่ภาพสังเคราะห์ 640×480 ใช้ประมาณ 4 ms;
ยังต้องวัด latency จริงบนคอมผู้ใช้และตรวจอายุผลตามเกณฑ์ควบคุมเดิม
