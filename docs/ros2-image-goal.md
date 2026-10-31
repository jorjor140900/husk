# วิ่งตามเส้นจาก image + AI โดยมีปลายทางบนแผนที่

โหมด `steering_mode:=vision` ใช้ SegFormer/colour veto สร้าง reference เฉพาะหน้า
ในหน่วยเมตร แล้ว Pure Pursuit ติดตาม reference นั้น เส้นเหลืองในกล้องวาดจาก
**reference เดียวกันที่ controller ใช้ ณ capture เดียวกัน** ไม่ใช่เส้น preview
ที่สร้างแยกอีกชุด และไม่ใช่เส้น global Nav2 ที่เอามาแสดงทับ

```text
RViz goal + AMCL + map/LiDAR planner -> coarse direction/branch guide
camera -> AI labels -> calibrated observed corridor -> metric reference
                         guide biases branch choice --^
metric reference -> Pure Pursuit -> disabled navigation gate -> collision filter
                                             -> robot-local permission/watchdog -> mux
same metric reference + same capture -> yellow camera overlay
```

## สิ่งที่เพิ่ม

- Global planner คำนวณ guide ใหม่ประมาณทุกวินาที ใช้แผนที่/scan/inflation
  โดยไม่ใส่ AI veto เข้า **global guide** ในโหมดนี้ เพราะ AI จำกัดเส้นที่ขับจริงใน local planner อยู่แล้ว
- Local planner ใช้ label จาก AI: `1` คือหลักฐานพื้น, `0` คือ veto, `255` คือไม่เห็น
  ตรวจช่องกว้างตาม `half_width_m + lateral_margin_m` และเก็บขอบ/ช่องดำไว้
  ไม่สร้างเส้นผ่าน unknown ไม่ย้าย seed ไปยังพื้นอีกกลุ่ม และไม่ทำ smoothing ข้าม veto
- จุดเริ่มอยู่บนแกนหุ่น Y=0 ที่แถวภาพใกล้หุ่นตาม calibration/mount
  สร้างเส้นเรียบในระยะมองข้างหน้าสูงสุด 2.8 m เลือกทางกลาง corridor เป็นหลัก
  ใช้ waypoint จาก guide ช่วยเลือกทิศ/แขนง ไม่คัดลอกความโค้ง global path
- ตรวจ TF ที่ capture time และ retry เฉพาะ packet ล่าสุดจนหมดอายุ 500 ms
  ไม่มีการใช้ latest TF ทดแทน capture TF, ไม่มี inference เพิ่ม, ไม่มี pixel-DP worker
- Controller สร้าง private `TwistStamped` จำกัด 0.05 m/s, 0.2 rad/s
  หยุดเมื่อ path/ภาพ/odom/guide/TF ใช้ไม่ได้, ยกเลิก goal หรือถึงเป้าภายใน 15 cm
  สถานะ goal reached ล็อกจนเลือก goal ใหม่ ไม่เล็งให้ตรง yaw ของ goal ในรุ่นนี้
- โหมดนี้ไม่เปิด Nav2 controller หรือ navigator แม้ `allow_motion=true`
  จึงมี steering producer เดียว และไม่มี publisher ไป `/cmd_vel` โดยตรง

## อัปเดตขณะหุ่นปิดได้

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

ไม่ต้องแก้ calibration/mount/profile เพื่ออัปเดตซอฟต์แวร์นี้ และไม่ได้อัปเดต
visualization เดิมของผู้ใช้ การแก้ค่านี้ไม่ทำให้ตรวจภาคสนามผ่านโดยอัตโนมัติ

## เปิดเพื่อดูเส้นและคำขอแบบ preview เมื่อมีหุ่น

เปิด gateway แบบ preview, tunnel และ webcam ตามเดิม ใช้ ground/camera frames
ที่วัดไว้ หยุด navigation เก่าและ AI เก่าก่อนเปิดใหม่ เพื่อไม่ให้มีสองระบบแย่งกัน

AI ใช้ fast segmentation โดย **ปิด preview เส้นเหลืองเก่า** ตัวอย่างใช้ config
spatial-support แบบทดลองที่ลด noise สีเขียว ยังต้องเทียบภาพดิบจริงก่อนขับ:

```bash
ros2 launch husky_vision drivable_area.launch.py \
  use_webcam:=false image_topic:=/camera/image_raw \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=256 model_height:=160 cpu_threads:=2 \
  config_file:="$HOME/husky_vision_ws/src/husky_vision/config/outdoor-supported-green-preview.json" \
  camera_calibration:="$HOME/husky_vision_ws/camera_calibration/camera-active.yaml" \
  camera_mount:="$HOME/husky_vision_ws/robot/robot-mount.json" \
  show_nav2_path:=false async_reference_preview:=false
```

การกรองสีแบบ support อาจพลาดกลุ่มเขียวเล็กแยกเดี่ยว หรือยัง veto จุดเขียวใหญ่บนพื้น
หากต้องใช้การกรองสีเดิมแทน ให้เลือก `outdoor-nav2-fast.json` ไม่ใช้ `no-colour.json`
เป็นค่าขับจริง เพียงเพื่อให้เส้นผ่านได้

เปิด navigation:

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision navigation.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  steering_mode:=vision allow_motion:=false
```

กด 2D Pose Estimate ให้ตรงพื้นที่จริง แล้วเลือก goal ตามเดิม RViz โหมดนี้ซ่อน
global guide โดยเริ่มต้น และแสดง `Image AI controller reference` เป็นเส้นเหลือง
จึงไม่สับสนว่าเส้นไหนคือ reference ที่ controller ใช้ ถ้าอยากตรวจ coarse guide
เปิด `Goal preview plan` กลับได้

ดูภาพ **topic ใหม่** ที่ใช้ reference จริง:

```bash
QT_QPA_PLATFORM=xcb ros2 run image_view image_view --ros-args \
  -r image:=/husky_nav/vision_overlay \
  -p image_transport:=compressed \
  -p autosize:=false -p width:=960 -p height:=720
```

ภาพยังมีขนาดดิบเท่าเดิม ไม่ crop/ปรับ calibration เส้นเหลืองหนา 2 px
และเส้นแกนหุ่นสีน้ำเงิน 3 px สีเขียวคือ mask ของ capture เดียวกัน
ถ้า AI ขาดจะใช้ภาพดิบสดพร้อมข้อความ waiting for fresh AI และซ่อนเส้นเก่า
หากไม่มี goal/ไม่มี reference ที่ผ่าน mask จะรายงานเหตุผลและไม่สร้างเส้นหลอก

```bash
ros2 topic echo /husky_nav/vision_control_status --once --full-length
ros2 topic echo /husky_nav/control_status --once --full-length
```

`tracking` ใน vision status คือ **คำขอ preview** ไม่ใช่ยืนยันว่ามอเตอร์ถูกเปิด
`planning_only` ใน gate ต้องยังเป็นศูนย์เมื่อ `allow_motion=false`
ตำแหน่ง goal เป็นเกณฑ์หยุดจริง ไม่ใช่ปลาย reference เฉพาะหน้า ซึ่งอาจมองไกลกว่า
goal ใกล้ ๆ เพื่อกำหนดทิศเลี้ยวได้ เส้นเหลืองจึงแสดง reference การเลี้ยวเฉพาะหน้า
และหุ่นจะหยุดตามระยะ goal ไม่วิ่งจนสุดเส้นทุกครั้ง
`navigation_plan_check` เดิมยังตรวจ **global guide** ไม่ใช่เส้น local นี้
ให้ดู `vision_control_status.reference_max_lateral_m` และ `Image AI controller reference`
เพื่อประเมินเส้นที่ controller ใช้จริง

ยกเลิก goal และส่งคำขอศูนย์:

```bash
ros2 service call /husky_nav/cancel_vision_goal std_srvs/srv/Trigger "{}"
```

## ขอบเขตที่ต้องเห็นชัดก่อนใช้กับหุ่น

นี่คือ local forward-only planner ยังไม่รองรับถอยหลังหรือหมุนอยู่กับที่เพื่อหา
ทางที่อยู่นอกกล้อง ถ้า goal อยู่หลังหุ่น/guide ต้องหันเกิน 65 องศา จะหยุดและ
รายงาน `goal_not_ahead` หรือ `route_requires_reorientation` แทนการเดาทิศ
ทางแยกใช้ได้เมื่อแขนงที่เลือกยังอยู่ในส่วนที่กล้องเห็นและตรวจ corridor ได้

กล้องมี blind zone ระหว่างฐานหุ่นกับจุดเริ่ม reference รายงานเป็น
`unseen_near_field_m` ไม่มีการต่อเส้นจาก origin หรืออ้างว่าใต้หุ่นเป็นพื้นที่ว่าง
การตรวจ local reference นี้ตรวจ **ความกว้างขวางเส้น** ไม่ใช่ swept footprint
เต็มคัน/ระยะเบรก/ขอบตก/พื้นราบจริง จึงยังต้องใช้ final collision filter และ
การตรวจภาคสนามเดิม โดยเฉพาะ VLP16 runtime obstacle integration ที่ยังค้าง
จอย/local permission และ watchdog ยังต้องผ่านเช่นเดิม โหมดนี้ไม่ปลด gate เหล่านั้น
และไม่เปลี่ยน field flags ให้ true

หญ้าขึ้นเขียวใน mask หรือจุดดำผิดบนพื้นยังต้องแก้การจำแนกภาพ ไม่สามารถชดเชย
ด้วยการบังคับเส้นตรงได้ และแผนที่ LiDAR ที่ว่างไม่ได้รับรองว่าพื้นนอกกล้องเป็นทางเดิน

## หลักฐานการทดสอบ

Unit tests ครอบคลุม corridor ตรง/โค้ง, goal เลือกแขนง, seed ถูกบล็อก,
ทางแคบ, unknown, ช่องดำ, capture-time TF, เปลี่ยน/ยกเลิก goal และ goal reached latch
ทดสอบ ROS Jazzy ใน container แบบ network none แล้ว: รับ segmentation/TF/odom/guide จำลอง,
สร้าง private request, gate โหมด preview คงศูนย์, หยุดเมื่อ mask บล็อก/เก่า/ยกเลิก goal,
ซ่อนเส้นเก่าขณะกล้องยังส่งภาพ และภาพ overlay มี source stamp ตรงกับ controller reference
วงจรปิด kinematic จำลองไปถึง goal ในระยะ tolerance 15 cm โดยไม่มี `/cmd_vel` publisher
guide ใน probe เป็นข้อความจำลอง ไม่ใช่ Nav2 planner จริง; action client/refresh ของ
dispatcher และการเลือก launch mode ตรวจแยกด้วย unit tests

fixture อยู่ใน `tests/native_vision_goal_probe.py` ใช้เฉพาะ container ทดสอบที่ไม่มี network
และตั้ง `HUSKY_ISOLATED_TEST=1` หลัง build/source exported package แล้ว
ไม่รัน fixture นี้บน ROS graph ที่เชื่อมต่อหุ่นจริง ผลเหล่านี้ไม่ใช่หลักฐานการวิ่ง
หรือระยะหยุดจริงบนหุ่น และไม่ได้ประเมินคุณภาพ SegFormer กับภาพภาคสนามใหม่

## วัดกล้องและ AI ตอนหุ่นปิด

หากกล้อง USB ยังเสียบคอม เปิดเฉพาะ webcam และ AI ตามปกติได้ ไม่ต้องเปิด
gateway/tunnel/navigation/RViz หรือเลือก goal ให้รอ AI ขึ้น Ready ก่อนเริ่มวัด
ใช้ภาพกล้องและหน้าต่าง preview ตามสภาพที่รู้สึกว่าช้า เพื่อให้วัด workload นั้นจริง
อย่าเปิด webcam ซ้ำถ้ามี node จับอุปกรณ์อยู่แล้ว

ใน Terminal ใหม่ที่ source ROS/venv แล้ว:

```bash
cd ~/husk-live
git pull --ff-only origin work
python3 tools/measure_vision_latency.py --duration 20 \
  --output-dir "$HOME/husky_vision_ws/outputs/vision-latency"
```

สคริปต์อยู่ใน Git checkout ไม่ต้อง export/build ใหม่สำหรับการวัดนี้ อ่านเฉพาะ
header ของภาพดิบ, segmentation packet และตัวเลข status ไม่ decode ภาพหรือรันโมเดล
ไม่ส่ง command/goal/service และไม่ปรับพารามิเตอร์ บันทึก JSON ใหม่แล้วจบเอง
มี warmup 2 วินาทีก่อนช่วงวัด การ subscribe packet ทำให้ AI publish PNG ตามปกติ
จึงรวม overhead ของการส่ง segmentation แต่ไม่ activate งานวาด preview โดยค่าเริ่มต้น

ดู `streams.camera` เทียบ `streams.ai_mask`:

- `delivered_fps`: จำนวน capture ไม่ซ้ำต่อวินาที ไม่ใช้ heartbeat ซ้ำเป็น FPS
- `source_age_at_receive_ms.p95/max`: อายุจาก timestamp ต้นทางจนถึงตัววัด
- `unique_capture_delivery_gap_ms.p95/max`: ช่องว่างระหว่างผลคนละ capture
- `sampled_age_gate_closed_percent`: สัดส่วนเวลาโดยประมาณที่ข้อมูลล่าสุดไม่ผ่าน
  เกณฑ์อายุ 500 ms รวมช่วงรอผลถัดไป ตรวจทุก 50 ms ค่านี้บอกความเสี่ยงหยุดเพราะอายุ
  **ไม่ใช่** เปอร์เซ็นต์การหยุดจริง และไม่รวม permission/TF/obstacle gates อื่น
- `ai_reported_timings_ms`: เวลา input/inference/detector/การส่งภาพที่ AI รายงาน

ตัวอย่าง: camera 15 FPS อายุ 30 ms แต่ mask 2 FPS อายุ 300 ms ตอนส่ง
อาจมีช่วง mask เกิน 500 ms ระหว่างรอเฟรมถัดไปได้ แม้ p95 ตอนรับจะผ่าน
ถ้า mask อายุ 1,500–2,500 ms จริง age gate จะปิดแทบตลอด

ถ้าอยากวัดการ publish ภาพ preview เพิ่ม ให้เปิด viewer ตามใช้งานจริงและเพิ่ม:

```bash
python3 tools/measure_vision_latency.py --duration 20 --with-overlay \
  --overlay-topic /husky_nav/vision_overlay/compressed
```

สำหรับ AI overlay เก่า ใช้ `--overlay-topic /drivable_area/overlay/compressed` แทน
การ subscribe overlay อาจเปิดงานวาด/JPEG/worker หากเดิมไม่มี viewer จึงไม่เปิดเป็น default
ตัวเลขนี้วัดจนถึงการรับภาพ overlay ยังไม่รวมคิว/การวาดของ image_view และจอจริง
เมื่อมี navigation/goal/TF สดแล้ว ใช้ `--with-reference` เพื่อวัด reference ของ
controller เพิ่มได้ ตอนหุ่นปิดไม่มี reference เป็นเรื่องปกติ

timestamp webcam ปัจจุบันประมาณเวลาที่ host อ่านเฟรมสำเร็จ ไม่ใช่เวลา exposure
จึงยังมองไม่เห็น delay ในกล้อง/USB/backend ก่อน timestamp ถ้าตัวเลขต่ำแต่ตาเห็น
ดีเลย์ 1–2 วินาที ต้องเทียบ timer ที่อยู่ในภาพกับ timer จริงพร้อมกันอีกขั้น
โดยคงตำแหน่ง mount ของกล้องไว้ การวัด baseline ขณะหุ่นปิดไม่แทนการวัดพร้อม
navigation และการเชื่อมต่อจริงในวันใช้งาน

### แยกกล้อง USB ออกจากการส่งภาพ ROS

`webcam_node` ใช้ SingleThreadedExecutor สำหรับ callback ROS และยังอ่าน USB
ต่อเนื่องใน latest-frame worker แยกต่างหาก ไม่มีการเพิ่มคิวภาพ เปลี่ยนขนาดภาพ
หรือตั้ง timestamp ใหม่ให้ภาพเก่า ส่วน AI ยังใช้ executor สองเธรดตามเดิม
การเปลี่ยนนี้ลด handoff ของ callback กล้อง แต่ไม่ได้ยืนยันว่าแก้อาการค้างบนเครื่องจริงแล้ว

หลัง update/export/build แล้ว ใช้คำสั่งเปิดกล้องเดิม และอ่านรายงาน:

```bash
ros2 topic echo /camera/capture_status --once --full-length
```

- `capture_fps` มาจากจำนวนครั้งที่ OpenCV อ่านเฟรมสำเร็จใน worker
- `published_fps` มาจากจำนวนครั้งที่ publisher ส่งคืนสำเร็จ ไม่รับรองว่า subscriber ได้รับครบ
- `capture.capture_gap_max_ms`, `capture.device_read_max_ms` เป็นค่าสูงสุดตั้งแต่เปิดกล้อง
- `delivery.slot_wait_max_ms`, `message_copy_max_ms`, `dds_publish_max_ms`,
  `callback_max_ms`, `publish_gap_max_ms` เป็นค่าสูงสุดในช่วงรายงานล่าสุด
- `window_sec` อาจเกินหนึ่งวินาทีถ้า callback ถูกบล็อก จึงใช้ช่วงเวลาจริงคำนวณ FPS

ถ้า capture สม่ำเสมอแต่ published FPS ต่ำ ให้ดูเวลา callback/publish และ scheduling
ถ้าทั้งสองสม่ำเสมอแต่ตัววัดหรือ AI รับไม่ครบ ให้ตรวจ DDS/เครื่องมือวัด/subscriber ต่อ
ถ้า capture เองมีช่องว่าง ให้ตรวจ V4L2/OpenCV/USB/การตั้งกล้องและ scheduling ของ worker
รายงานนี้ไม่มีคำสั่งเคลื่อนหุ่นและไม่เปลี่ยนเกณฑ์ freshness

หากทดสอบ V4L2 โดยตรง ต้องปิดทุกโปรแกรมที่จับกล้องก่อน ตัวอย่างรับ 300 เฟรม
โดยไม่เปลี่ยน format หรือ calibration:

```bash
time timeout 30s v4l2-ctl \
  -d /dev/v4l/by-id/usb-Generic_Web_Camera_200901010001-video-index0 \
  --stream-mmap=4 --stream-count=300 --stream-to=/dev/null
```

ดู FPS ที่รับจริงร่วมกับเวลารัน แม้ `--get-parm` จะโฆษณา 30 FPS ก็อาจรับจริง 15 FPS
และถ้ากด Ctrl+C ก่อนจบจะยังไม่ได้รับครบ 300 เฟรม
