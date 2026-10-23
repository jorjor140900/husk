# สร้าง workspace และแพ็กเกจ ROS 2 Jazzy ใหม่

ใช้ **workspace `~/husky_vision_ws`** และ **แพ็กเกจ `husky_vision`**
โปรแกรมสร้างจะคัดลอกเฉพาะ detector ปัจจุบัน, ROS nodes, config, launch และโมเดลที่ train แล้ว
workspace ใหม่นี้ไม่ใช้ symlink กลับ repository จึงแก้โค้ด ROS จากโฟลเดอร์เดียวได้
ไม่ต้อง train ใหม่ ไม่มีการส่งคำสั่งให้หุ่นเคลื่อนที่

## 1. สร้าง workspace ใหม่ (ครั้งเดียว)

หยุดโปรแกรมกล้องเดิม เปิด **Terminal ใหม่บน Ubuntu** แล้วเข้าโฟลเดอร์เดิมที่ใช้รัน
`RoadDetection.py` ซึ่งมี `models/husky-segformer` อยู่ข้างใน จากนั้นรัน:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py \
  --workspace ~/husky_vision_ws \
  --model-source models/husky-segformer
```

อย่าสร้าง `~/husky_vision_ws` ด้วย `mkdir` ก่อนคำสั่งนี้ โปรแกรมต้องการปลายทางที่ยังไม่มี
ถ้ามีชื่อนี้อยู่แล้ว ให้เลือกชื่อ workspace ใหม่และเปลี่ยน path ในคำสั่งถัดไปตามกัน
หากโมเดลอยู่โฟลเดอร์อื่น ให้เปลี่ยนเฉพาะ `--model-source` เป็น path จริงของโมเดล
การคัดลอกโมเดลใช้พื้นที่ดิสก์เพิ่มตามขนาดโมเดล

โครงสร้างหลังสร้างสำเร็จ:

```text
husky_vision_ws/
├── src/
│   └── husky_vision/             ← แพ็กเกจ ROS ใหม่ มีเพียงแพ็กเกจเดียว
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/husky_vision
│       ├── husky_vision/         ← ROS nodes.py และ transport.py
│       ├── road_navigation/     ← detector, โมเดล inference, การสร้างเส้น/ภาพ
│       ├── launch/drivable_area.launch.py
│       ├── config.example.json
│       └── requirements-ros2.txt
├── models/husky-segformer/       ← สำเนาโมเดลที่ train แล้ว
├── README.md
└── START_HERE.md
```

ต่อจากนี้ทำงานใน `~/husky_vision_ws` ทั้งหมด ไม่ต้องเข้า `drivable_area_ws` เดิม
โค้ดและโมเดลเป็นสำเนาแยกจาก repository เดิม การ pull repository ในอนาคตจะไม่เปลี่ยนสำเนานี้อัตโนมัติ
ถ้ามีแพตช์สำหรับ ROS adapters ให้ใช้ `--update-code` ตามขั้นตอนท้ายคู่มือนี้
ถ้าต้องการบันทึกการเปลี่ยนแปลงของ workspace ใหม่ลง Git ให้สร้าง repository แยกภายหลัง
ไฟล์ `.gitignore` เตรียมให้แล้ว โดยไม่รวมโมเดล, venv และไฟล์ build

## 2. ติดตั้ง dependencies (ครั้งแรก)

ใช้ ROS 2 Jazzy บน Ubuntu 24.04 กับ system Python 3.12
เปิด Terminal ใหม่ที่ยังไม่ได้ source workspace เก่า แล้วรัน:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install python3-venv python3-colcon-common-extensions ros-jazzy-rqt-image-view ros-jazzy-compressed-image-transport

/usr/bin/python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate

python -m pip install 'torch>=2.6,<3' --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r src/husky_vision/requirements-ros2.txt
python -c "import rclpy, cv2, torch, transformers; print('ROS + AI imports OK')"
```

ถ้า `~/.bashrc` มีบรรทัด `source .../drivable_area_ws/install/setup.bash` ของ workspace เก่า
ให้ใส่ `#` นำหน้าบรรทัดนั้นก่อนเปิด Terminal ใหม่ ส่วน `/opt/ros/jazzy/setup.bash` ใช้ได้ตามเดิม
ไม่ต้องลบไฟล์หรือ workspace เก่า

## 3. Build (ครั้งแรกและหลังแก้โค้ด)

ใน Terminal เดิม:

```bash
cd ~/husky_vision_ws
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
ros2 pkg executables husky_vision
```

ควรเห็น `husky_vision drivable_area_node` และ `husky_vision webcam_node`
ใช้ `python /usr/bin/colcon` เพื่อให้ node ใช้ Python ของ venv ที่ติดตั้ง torch
โฟลเดอร์ `build/`, `install/`, `log/` ที่เพิ่มมาคือผลจาก build ไม่ใช่ source packages เพิ่ม

## 4. เปิดกล้องและ AI

```bash
ros2 launch husky_vision drivable_area.launch.py \
  camera_index:=2 \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=448 model_height:=256
```

`camera_index:=2` ตรงกับ `--camera 2` เดิม ต้องหยุดโปรแกรมอื่นที่ใช้กล้องตัวเดียวกันก่อน
หากเปลี่ยนชื่อ workspace ให้แก้ `model_path` ตามนั้น
ถ้าต้องการลดความละเอียด inference เพิ่ม ลอง `model_width:=320 model_height:=192`
ค่าเริ่มต้นขอภาพ webcam 640×480, 15 fps; ผลจริงขึ้นกับกล้อง
ค่าเริ่มต้น `cpu_threads:=2` จำกัด worker ของ PyTorch เพื่อเหลือ CPU ให้ ROS และ viewer
หากต้องการเทียบความเร็ว ลอง 2 กับ 4 บนเครื่องจริงโดยคงความละเอียดเดิมไว้
หยุดทั้งสอง node ด้วย `Ctrl+C`

## 5. ดูภาพ (Terminal ที่สอง)

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/.venv/bin/activate
source ~/husky_vision_ws/install/local_setup.bash
ros2 run rqt_image_view rqt_image_view
```

เลือก **`/drivable_area/overlay/compressed`** ในรายการด้านบนของหน้าต่าง
ช่องนี้บีบอัดเฉพาะภาพ preview เป็น JPEG เพื่อให้ข้อมูลภาพที่ส่งมีขนาดเล็กลง
ภาพเข้าโมเดล, mask และพิกัด reference ยังคงใช้ข้อมูลเดิม
หากเพิ่งติดตั้ง compressed-image-transport ให้ปิด rqt เดิมแล้วเปิดใหม่เพื่อโหลด plugin
จะเห็น mask สีเขียว, reference สีเหลือง, แกนกลางสีน้ำเงิน และขอบประมาณสีแดง
ดู mask แยกจาก `/drivable_area/mask` และดู status ด้วย:

```bash
ros2 topic echo /drivable_area/status --once
```

reference ส่งทาง `/drivable_area/reference_pixels` เป็น `geometry_msgs/PolygonStamped`
แต่เป็น **ลำดับจุดของเส้นเปิดในพิกัดภาพ** x=u, y=v, z=0 หน่วยพิกเซลหลัง resize
ห้ามใช้เป็นพิกัดเมตรใน TF/Nav2; ต้องเพิ่ม calibration/depth ก่อนทำ controller
header ของผลตรวจคง timestamp ของภาพต้นทาง และ status มีขนาดภาพ `processed_shape_hw`
เมื่อไม่มี path, กล้องขาด หรือข้อมูลเก่าเกินกำหนด จะส่ง reference ว่างและ reason ใน status
ภาพใน viewer อาจค้างที่เฟรมสุดท้ายได้เมื่อกล้องหลุด ต้องดู status/timestamp ประกอบ

## 6. เปิดใช้งานครั้งถัดไป

เปิด Terminal ใหม่แล้วรันชุดนี้ ไม่ต้องติดตั้ง dependencies หรือสร้างแพ็กเกจซ้ำ:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/local_setup.bash

ros2 launch husky_vision drivable_area.launch.py \
  camera_index:=2 \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=448 model_height:=256
```

หากจะใช้ภาพจาก ROS camera driver หรือ ROS1 bridge ภายหลัง ให้เพิ่ม
`use_webcam:=false image_topic:=/ชื่อ_topic_ภาพ` โดย input ต้องเป็น raw `sensor_msgs/Image`
encoding `bgr8`, `rgb8` หรือ `mono8`

## เมื่อ source แล้วขึ้น `not found` ของ workspace เก่า

`install/setup.bash` จะเรียก parent workspaces ที่ถูกบันทึกตอน build ด้วย
เช่น `660610834_final/install/local_setup.bash` แม้ตอนนี้เปิด Terminal ใหม่แล้ว
การ source `/opt/ros/jazzy/setup.bash` ไม่ได้ลบรายการ parent ที่บันทึกในไฟล์นั้น
แพ็กเกจนี้ใช้ Jazzy เป็นฐานโดยตรง จึงใช้ `install/local_setup.bash` หลัง source Jazzy ได้:

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/install/local_setup.bash
ros2 pkg executables husky_vision
```

ใช้รูปแบบนี้ทุกครั้งที่เปิด Terminal สำหรับ workspace นี้
`local_setup.bash` เพิ่มแพ็กเกจใน workspace ปัจจุบัน โดยไม่เรียก parent chain จากตอน build
ถ้าข้อความของ workspace เก่าขึ้นทันทีที่เปิด Terminal ก่อนพิมพ์คำสั่ง
ให้ตรวจบรรทัด source อัตโนมัติใน `~/.bashrc` ด้วย
หากต้องการสร้าง `install/setup.bash` ใหม่ให้มีเพียง Jazzy เป็น parent
ต้อง build จาก environment ที่ไม่ได้ source workspace อื่นหรือ workspace ตัวเองอยู่ก่อน

## อัปเดตแพตช์ ROS ใน workspace ที่สร้างไว้แล้ว

หยุด launch ด้วย `Ctrl+C` ก่อน เปิด Terminal ในโฟลเดอร์ Git เดิมที่มี `RoadDetection.py` แล้วรัน:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py --workspace ~/husky_vision_ws --update-code
```

คำสั่งนี้อัปเดตเฉพาะ `husky_vision/nodes.py`, `husky_vision/transport.py` และ launch
ไฟล์เดิมรวม local edits จะสำรองไว้ใน `.source-backups/<เวลา>/` ของ workspace
โมเดล, detector หลัก, config และ dependencies ที่ติดตั้งแล้วไม่ถูกเขียนทับ
หลังจากนั้น build ใน Terminal ใหม่ที่ source เฉพาะ Jazzy:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash

ros2 launch husky_vision drivable_area.launch.py \
  camera_index:=2 \
  model_path:="$HOME/husky_vision_ws/models/husky-segformer" \
  device:=cpu model_width:=448 model_height:=256 cpu_threads:=2
```

## แยกความหน่วงของโมเดล การส่งภาพ และ rqt

หาก `/camera/image_raw` ใน rqt ตอบสนองไว แต่ overlay หน่วง ให้ตรวจเส้นทางหลังรับภาพ
ค่า `host age` ที่ฝังบน overlay วัดถึงตอนเตรียมภาพ ไม่ได้วัดอายุภาพตอนวาดบนหน้าจอ rqt
ภาพที่รอแสดงอยู่นานจะยังแสดงตัวเลข host age เดิมที่ฝังไว้ตอนส่ง
ทั้ง timestamp กล้องและ age นี้เริ่มจาก host อ่านภาพเสร็จ ไม่รวมเวลารับแสง/คิวภายใน webcam

แพตช์ ROS ส่งภาพด้วย `array('B')` ซึ่งใช้ fast path ของ generated ROS setter
การส่งด้วย `bytes` เดิมทำให้ setter ของ Jazzy ตรวจ type/range ของทุกไบต์ใน Python
และ node จะเตรียม/ส่ง overlay กับ masks เฉพาะ topic ที่มีผู้รับ
เพิ่มช่อง `/drivable_area/overlay/compressed` สำหรับ rqt เมื่อการส่ง Image ดิบขนาดใหญ่ขาดช่วง
node วาด overlay ครั้งเดียว แล้วส่ง raw/JPEG ตามผู้รับของแต่ละช่อง
ปรับ JPEG quality ได้ด้วย `overlay_jpeg_quality:=80` ซึ่งมีผลกับ preview เท่านั้น

เปิด Terminal ที่ source Jazzy และ workspace แล้วตรวจ:

```bash
ros2 topic echo /drivable_area/status --once
ros2 topic delay /drivable_area/overlay/compressed --window 10
```

`ros2 topic delay` รายงานหน่วยวินาทีจาก source header ถึง subscriber ของคำสั่งนี้
ไม่ใช่เวลาวาดภาพบนหน้าจอ และ timestamp กล้องยังเป็นเวลา host read completion โดยประมาณ
ถ้า delay ต่ำกว่าความหน่วงที่เห็นใน rqt มาก ค่อยแยกตรวจการแสดงผล/ภาระ CPU ของ rqt ต่อ
ถ้า delay สูงด้วย ให้ดู status:

- `input_age_ms`: อายุภาพก่อน inference
- `inference_ms`: เวลาของโมเดลรวม processor/resize probability
- `detector_ms`: mask, temporal filtering และการสร้าง reference
- `image_output_ms`: การวาด แปลง และ publish images
- `frame_age_ms`: อายุภาพหลังส่ง images ก่อนส่ง status/reference

ทั้งหมดเป็นมิลลิวินาที และไม่ได้รวม GUI rendering
เหตุผล `stale_result`/`stale_output` จะมีเวลาของขั้นที่ทำเสร็จแล้วให้ตรวจ แม้ถูกงด reference
หากผลเก่าเกิน `max_frame_age_sec` ค่าเริ่มต้น 2 วินาที reference จะถูกล้างเหมือนเดิม
ควรวัดด้วย resolution เดิมก่อนลดเป็น 320×192 เพราะขนาดที่ลดอาจทำให้รายละเอียดสิ่งกีดขวางเล็ก ๆ หายไป

ค่า delay ต่ำไม่ได้แปลว่าภาพอัปเดตต่อเนื่องเสมอไป หากคำสั่งขึ้น `no new messages` หลายวินาที
viewer อาจยังค้างที่เฟรมสุดท้ายซึ่งฝังตัวเลข host age ต่ำเอาไว้
ให้ตรวจ warning `No fresh reference: ...` ใน Terminal ที่ launch และ `reason` ใน status
โดยเฉพาะ `stale_result`, `stale_input`, `camera_timeout` หรือ `processing_error`
node ยังคงงด reference ของเฟรมที่เก่าเกินกำหนด พร้อมรายงานเวลา inference ของเฟรมที่ทำเสร็จแล้ว
ทุกประมาณ 5 วินาทีที่มีการประมวลผลสำเร็จจะมี log `Frame age ... | inference ... | ...`
เมื่อเลือกช่อง JPEG ใน rqt ควรเห็น `preview subscribers raw=0, compressed=1`
ถ้ายังเป็น `compressed=0` ให้ตรวจว่าเลือก topic ถูกและติดตั้ง plugin แล้ว

สำหรับ workspace ที่สร้างไว้ก่อนมีช่อง compressed ให้อัปเดตด้วย `--update-code` ตามด้านบน
และติดตั้ง plugin เพิ่มครั้งเดียวก่อนเปิด rqt ใหม่:

```bash
sudo apt install ros-jazzy-compressed-image-transport
```

รายละเอียดการทดสอบด้วย ROS 2 จริงและข้อจำกัดอยู่ใน [บันทึกการตรวจ stream](https://github.com/jorjor140900/husk/blob/work/docs/ros2-stream-validation.md)
