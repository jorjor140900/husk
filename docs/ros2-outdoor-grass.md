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
เมื่อกล้องขาดหรือผลเก่า จะส่งภาพ diagnostic ที่ไม่มีเส้น โดยคง timestamp เดิม
ไม่ได้ตั้ง timestamp ใหม่ให้ภาพดูสด

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

### ทำไม p95 ผ่าน แต่ Planning inputs ready ยัง False

preflight ตรวจทั้ง p95 ของช่วงสังเกต และ **mask ล่าสุด ณ เวลาตรวจ**
เช่น p95=482 ms ผ่าน แต่เฟรมท้ายอายุ 575 ms ต้องขึ้น PENDING ตามเกณฑ์ 500 ms
การผ่าน p95 ไม่ได้แปลว่าทุกเฟรมผ่าน และไม่ได้อนุญาตใช้เฟรมเก่าขับต่อ
รายงานใหม่แสดง reason และ source_age_ms ของตัวล่าสุดเพื่อช่วยแยกกรณีนี้
ต้องลด latency/ความแกว่งให้มี mask ที่สดต่อเนื่องก่อนเปิดเคลื่อนที่จริง

หากต้องคืนพฤติกรรมเดิม ให้หยุด AI และเปิดใหม่โดยใช้ config เดิม
ค่าเริ่มต้นของ `reject_green_vegetation` เป็น false ไม่ต้องเปลี่ยนโมเดลกลับ

## ระยะยาว

เก็บภาพดิบของทางจริงจากตำแหน่งกล้องเดิม ทั้งแดด เงา ทางแยก และหญ้าทั้งสองด้าน
label หญ้าเป็น non-drivable และทางเดินเป็น drivable แล้ว fine-tune โมเดล
แยก validation ตามช่วง/บริเวณการเดิน ไม่ใช้เฟรมติดกันเป็นหลักฐานทดสอบอิสระ
ประเมิน false drivable บนหญ้าแยกจาก pixel accuracy รวม เพราะพื้นที่ทางเดินมากอาจกลบความผิดพลาด

ทดสอบอัตโนมัติครอบคลุม veto ทับผล AI ความมั่นใจสูง, temporal history,
การ downsample, blocked mask อื่น, packet ไป Nav2 และการ backup ตอนส่งออก
บน host ทดสอบ ตัวกรองอย่างเดียวที่ภาพสังเคราะห์ 640×480 ใช้ประมาณ 4 ms;
ยังต้องวัด latency จริงบนคอมผู้ใช้และตรวจอายุผลตามเกณฑ์ควบคุมเดิม
