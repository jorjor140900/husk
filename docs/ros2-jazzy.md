# เปิดกล้องและ AI ผ่าน ROS 2 Jazzy

**หากต้องการเริ่มใหม่แยกจากของเก่า ใช้ [คู่มือสร้าง workspace ใหม่ `husky_vision_ws`](ros2-fresh-start.md)**
คู่มือด้านล่างเป็นวิธีใช้แพ็กเกจจาก repository โดยตรงใน workspace เดิม

แพ็กเกจชื่อ **`husky_drivable_area`** อยู่ที่ root ของ repository นี้
`/home/jorjeen/drivable_area_ws` เป็น workspace; แพ็กเกจจะอยู่ใน `src/` ของ workspace
ใช้ detector และโมเดล SegFormer เดิม ไม่ต้อง train ใหม่ และไม่ต้องแก้โปรแกรม standalone

ข้อมูลไหลตามนี้:

```text
webcam_node -> /camera/image_raw -> drivable_area_node
                                    -> /drivable_area/overlay
                                    -> /drivable_area/mask
                                    -> /drivable_area/navigable_mask
                                    -> /drivable_area/reference_pixels
                                    -> /drivable_area/status
```

ขั้นนี้ยังไม่มี `/cmd_vel` และยังไม่ต้องใช้ ROS1 bridge
reference เป็นพิกเซลของภาพหลัง resize ไม่ใช่เส้นทางหน่วยเมตรสำหรับ Nav2

## 1. เตรียม source code บนเครื่อง Ubuntu ของคุณ

เปิด **Terminal ของ Ubuntu เครื่องที่เสียบกล้อง** และหยุดโปรแกรม `RoadDetection.py` เดิมก่อน
ตัวอย่างนี้สมมติ repository เดิมอยู่ที่ `/home/jorjeen/husk-live` และโมเดลอยู่ที่
`/home/jorjeen/husk-live/models/husky-segformer` ถ้าใช้โฟลเดอร์ชื่ออื่น ให้เปลี่ยน path ตามจริง
อย่าย้ายเฉพาะ `RoadDetection.py` ไปใน workspace เพราะต้องใช้โมดูลอื่นใน repository ด้วย

```bash
cd /home/jorjeen/husk-live
git pull --ff-only origin work
test -f package.xml && test -f models/husky-segformer/config.json

mkdir -p /home/jorjeen/drivable_area_ws/src
ln -s /home/jorjeen/husk-live /home/jorjeen/drivable_area_ws/src/husk
```

`ln -s` ทำครั้งเดียว ถ้ามี source ของ repository นี้อยู่ใน `src/` แล้วให้ใช้ตัวนั้น
อย่าเพิ่มสำเนาซ้ำ เพราะ colcon จะพบแพ็กเกจชื่อเดียวกันสองตัว
ไฟล์ `.git`, โมเดล และผลทดลองจะยังอยู่ในโฟลเดอร์เดิม

ถ้ายังไม่มี repository บนเครื่องนี้ ให้ใช้ `git clone --branch work https://github.com/jorjor140900/husk.git /home/jorjeen/husk-live`
แทนขั้น `git pull` และนำ **โฟลเดอร์โมเดลที่ train แล้ว** ไปไว้ใน `models/husky-segformer`
Git ไม่มี weights โมเดลให้ดาวน์โหลด

## 2. ติดตั้ง dependencies สำหรับ ROS แยกจาก standalone

Jazzy บน Ubuntu 24.04 ใช้ system Python 3.12 ห้ามใช้ venv ที่สร้างด้วย Python คนละรุ่น
venv ใหม่นี้มองเห็น `rclpy` จาก ROS แต่แยกแพ็กเกจ AI ออกจาก venv ที่เปิดกล้องเดิม
node ไม่ใช้ `cv_bridge`; รองรับ raw `sensor_msgs/Image` โดยตรง

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-venv python3-colcon-common-extensions ros-jazzy-rqt-image-view ros-jazzy-compressed-image-transport

/usr/bin/python3 -m venv --system-site-packages /home/jorjeen/drivable_area_ws/.venv
touch /home/jorjeen/drivable_area_ws/.venv/COLCON_IGNORE
source /home/jorjeen/drivable_area_ws/.venv/bin/activate

python -m pip install 'torch>=2.6,<3' --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r /home/jorjeen/husk-live/requirements-ros2.txt
python -c "import rclpy, cv2, torch, transformers; print('ROS + AI imports OK')"
```

ชุดนี้ใช้ CPU เหมือนคำสั่งเดิม การติดตั้ง torch อาจดาวน์โหลดไฟล์ใหญ่ แต่ทำครั้งเดียว
หาก pip แจ้งว่า `torchvision` หรือ `torchaudio` จากระบบไม่ตรงกับ torch ที่ติดตั้ง ให้ส่ง error มาตรวจ
node นี้ไม่ได้ใช้สองแพ็กเกจนั้น

## 3. Build ภายใน workspace

ใช้ Terminal เดิมที่ source ROS และ activate venv แล้ว:

```bash
cd /home/jorjeen/drivable_area_ws
python /usr/bin/colcon build --symlink-install --packages-select husky_drivable_area
source install/local_setup.bash
ros2 pkg executables husky_drivable_area
```

ควรเห็น `drivable_area_node` และ `webcam_node`
**ใช้ `python /usr/bin/colcon` ตามนี้** เพื่อให้ executable ของ node ใช้ Python ใน venv ที่มี torch
ถ้าเคย build แพ็กเกจนี้ด้วย Python ตัวอื่น ให้ลบเฉพาะ `build/husky_drivable_area` และ
`install/husky_drivable_area` แล้ว build ใหม่ อย่าลบทั้ง workspace ที่มีแพ็กเกจอื่น

## 4. เปิดกล้อง + detector

```bash
ros2 launch husky_drivable_area drivable_area.launch.py \
  camera_index:=2 \
  model_path:=/home/jorjeen/husk-live/models/husky-segformer \
  device:=cpu model_width:=448 model_height:=256
```

`camera_index:=2` ตรงกับ `--camera 2` เดิม หากอยากลดเวลาประมวลผลอีก ให้ลอง
`model_width:=320 model_height:=192` แล้วดูว่าขอบเสา/สิ่งกีดขวางยังดีพอหรือไม่
ค่าเริ่มต้นขอภาพกล้อง 640×480 ที่ 15 fps; เปลี่ยนผ่าน `camera_width`, `camera_height`, `camera_fps` ได้
ความละเอียดที่กล้องยอมรับจริงจะแสดงใน log และอาจต่างจากค่าที่ขอ
ความละเอียดโมเดลเป็นคนละค่ากับความละเอียดกล้อง และไม่ต้อง train ใหม่เมื่อปรับขนาด inference

โมเดลต้องเป็นโฟลเดอร์ที่แตก zip แล้ว มี `config.json`, `preprocessor_config.json` และ weights
ถ้า path ผิด node จะรายงาน error ไม่มีการเปลี่ยนกลับไปใช้ baseline เงียบ ๆ

## 5. ดูภาพผลลัพธ์และตรวจ topics

เปิด **Terminal ที่สอง**:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jorjeen/drivable_area_ws/.venv/bin/activate
source /home/jorjeen/drivable_area_ws/install/local_setup.bash
ros2 run rqt_image_view rqt_image_view
```

เลือก `/drivable_area/overlay/compressed` จากรายการด้านบนในหน้าต่าง
ใช้ JPEG เฉพาะภาพแสดงผล เพื่อลดขนาดข้อความภาพที่ส่งไป viewer
สีเขียว/เหลือง/น้ำเงิน/แดงเหมือนโปรแกรมเดิม และมีเวลา processing/frame age เพิ่มด้วย
ดู mask ได้จาก `/drivable_area/mask` หรือพื้นที่หลังเว้นขอบจาก `/drivable_area/navigable_mask`

เปิด Terminal อื่นโดย source เช่นเดียวกัน แล้วตรวจ:

```bash
ros2 topic list
ros2 topic hz /drivable_area/overlay --qos-reliability best_effort
ros2 topic echo /drivable_area/status --once
ros2 topic echo /drivable_area/reference_pixels --once
```

`rqt_image_view` บางเวอร์ชันมีตัวเลือก QoS ให้เลือก **Best effort** เพื่อรับภาพ
ถ้าจอดำ ให้ตรวจว่ามี `/camera/image_raw` ก่อน และดู error ใน Terminal ที่ launch
`Ctrl+C` ใน Terminal ที่ launch ใช้หยุดทั้งสอง node

## ใช้กล้องจาก ROS node อื่นภายหลัง

ถ้ากล้องมี driver ที่ publish `/usb_cam/image_raw` อยู่แล้ว ให้ปิด webcam node ใน launch:

```bash
ros2 launch husky_drivable_area drivable_area.launch.py \
  use_webcam:=false image_topic:=/usb_cam/image_raw \
  model_path:=/home/jorjeen/husk-live/models/husky-segformer
```

รองรับ `bgr8`, `rgb8`, `mono8` รวม row padding; ไม่รับ `CompressedImage`, depth16 หรือ YUV โดยตรง
ต้องใช้ driver/ตัวแปลงภาพให้เป็น raw Image ที่รองรับก่อน
เมื่อมี ROS1 bridge ภายหลัง สามารถเปลี่ยน `image_topic` มาเป็น topic กล้องที่ bridge มาจากหุ่นได้
ไม่จำเป็นต้อง bridge ภาพหาก webcam เสียบกับเครื่อง ROS2 โดยตรง

## ข้อมูลใน topics และหน่วย

| Topic | Message | ความหมาย |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | ภาพ BGR กล้องก่อน detector resize |
| `/drivable_area/overlay` | `sensor_msgs/Image` (`bgr8`) | ภาพผลตรวจพร้อมเส้น สี และสถานะ |
| `/drivable_area/overlay/compressed` | `sensor_msgs/CompressedImage` | JPEG ของภาพผลตรวจสำหรับ rqt คงขนาดและ header เดิม |
| `/drivable_area/mask` | `sensor_msgs/Image` (`mono8`) | 255=พื้นที่ที่ detector รับและเชื่อมกับ seed, 0=พื้นที่อื่น |
| `/drivable_area/navigable_mask` | `sensor_msgs/Image` (`mono8`) | mask หลังเว้นขอบด้วย `clearance_px` |
| `/drivable_area/reference_pixels` | `geometry_msgs/PolygonStamped` | **ลำดับจุดของเส้นเปิด** จากใกล้ไปไกล, x=u, y=v, z=0 หน่วยพิกเซล ไม่ปิดเส้นกลับจุดแรก |
| `/drivable_area/status` | `std_msgs/String` (JSON) | valid/reason, path, ขนาดภาพ, source header, frame age, reference geometry รวม dx และขอบแดง |

ทุกผลลัพธ์ที่ได้จากภาพจะคง `header.stamp` และ `header.frame_id` ของภาพต้นทาง
timestamp จาก webcam node เป็นเวลา host อ่านภาพเสร็จโดยประมาณ ไม่ใช่เวลาเปิดรับแสงของกล้อง
frame_id ระบุภาพต้นทางเท่านั้น **ห้ามส่งจุดพิกเซลเข้า TF หรืออ้างเป็นพิกัดเมตรใน camera/base_link**
ถ้าต้องการเส้นทางเมตร ต้องเพิ่ม calibration และการแปลงลง ground plane/depth ก่อน

พิกัด reference และ `dx` อิงขนาด `processed_shape_hw` ใน JSON ซึ่งตรงกับ mask/overlay
ไม่ใช่ขนาด `/camera/image_raw` ก่อน resize; `dx` บวกหมายถึงเรฟอยู่ทางขวาของแกนสีน้ำเงิน
แกนสีน้ำเงินเป็นสมมติฐานว่ากล้องอยู่แนวกึ่งกลางหุ่น ยังไม่ได้วัด pose ของหุ่นจริง
สีแดงเป็นเส้นประมาณขอบสำหรับแสดงผล ไม่ใช่ข้อบังคับในการวางแผน

Images ใช้ Best effort / Keep last / depth 1; reference และ status ใช้ Reliable / depth 1 แบบ volatile
ผู้รับ reference ต้องรับเส้นว่างเป็นการยกเลิกเส้นเดิม และตรวจ timestamp ของเส้นทุกครั้ง
JSON `valid_image_path` หมายถึงเส้นอยู่ใน mask ที่ประเมิน ไม่ใช่การยืนยัน traversability จริง
`quality_score` ยังเป็น heuristic และไม่ใช่คะแนนความปลอดภัย

## ป้องกันความหน่วงและข้อมูลค้าง

อ่านภาพกล้องต่อเนื่องและเก็บเฉพาะเฟรมล่าสุด ทั้ง DDS และ node ประมวลผลมี queue ลึก 1
subscriber รับภาพได้ระหว่าง inference; ไม่ทำ inference ยาว ๆ ใน callback รับภาพ
`subscription_frames_skipped` ใน status คือจำนวนภาพที่แทนภาพเก่าใน slot ระหว่างรอประมวลผล

ค่า `max_frame_age_sec` เริ่มต้น 2 วินาที:

- input หรือผลหลัง inference เก่าเกินค่านี้: งดผลเฟรมนั้นและส่ง reference ว่างด้วยเหตุผล `stale_input`/`stale_result`
- ไม่มีภาพเข้าเกินค่านี้: ส่ง reference ว่างและ `camera_timeout` พร้อมล้าง temporal history
- แปลงภาพหรือประมวลผลล้มเหลว: ส่ง reference ว่างและ `processing_error`
- detector ไม่พบ reference: ส่งเส้นว่างตาม reason ของ detector

ภาพ overlay/mask ส่งเฉพาะเฟรมที่ประมวลผลสำเร็จ จึงอาจยังเห็นภาพสุดท้ายค้างใน viewer เมื่อกล้องหลุด
ดู status และ timestamp ประกอบเสมอ ไม่มีการใส่ timestamp ใหม่ให้ภาพเก่า
การตรวจ timeout ทำในกลุ่ม callback ประมวลผล: ถ้า inference ค้าง ผู้รับต้องตรวจอายุ timestamp เองด้วย
เครื่องส่งภาพที่อยู่คนละเครื่องต้อง sync clock; ถ้า timestamp เป็นศูนย์จะวัดได้เฉพาะเวลาหลังรับภาพ

## ขอบเขตการตรวจใน cloud

ตรวจ `colcon build --symlink-install` ผ่าน รวมการติดตั้ง resource/launch/config และ Python ของ entry points
tests ทั้ง 62 รายการผ่านในชุด dependencies ROS-specific (Python 3.12, NumPy 1.26.4,
OpenCV 4.10, torch 2.6 CPU) ครอบคลุม detector/โมเดล, Image/row padding, queue และการล้าง reference เก่า
การตรวจ callback ใช้ message/Node stand-ins เพื่อทดสอบ logic ไม่ใช่การตรวจ ROS middleware
ทดสอบ node/DDS จริงเพิ่มเติมใน official ROS Jazzy container ด้วยกล้องสังเคราะห์แล้ว
ดู [ผลวัดและวิธีรันซ้ำ](ros2-stream-validation.md); ยังไม่ได้ทดสอบ webcam และการวาด GUI บนเครื่องผู้ใช้
ขั้น build และเปิดกล้องข้างต้นคือการตรวจ integration บน Ubuntu Jazzy ของคุณ
เมื่อเปิดได้แล้ว ขั้นถัดไปคือบันทึก rosbag ผลตรวจและวางแผน calibration ก่อนสร้าง controller หรือ bridge คำสั่งหุ่น
