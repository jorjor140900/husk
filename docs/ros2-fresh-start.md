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
ถ้าต้องการบันทึกการเปลี่ยนแปลงของ workspace ใหม่ลง Git ให้สร้าง repository แยกภายหลัง
ไฟล์ `.gitignore` เตรียมให้แล้ว โดยไม่รวมโมเดล, venv และไฟล์ build

## 2. ติดตั้ง dependencies (ครั้งแรก)

ใช้ ROS 2 Jazzy บน Ubuntu 24.04 กับ system Python 3.12
เปิด Terminal ใหม่ที่ยังไม่ได้ source workspace เก่า แล้วรัน:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install python3-venv python3-colcon-common-extensions ros-jazzy-rqt-image-view

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
หยุดทั้งสอง node ด้วย `Ctrl+C`

## 5. ดูภาพ (Terminal ที่สอง)

```bash
source /opt/ros/jazzy/setup.bash
source ~/husky_vision_ws/.venv/bin/activate
source ~/husky_vision_ws/install/local_setup.bash
ros2 run rqt_image_view rqt_image_view
```

เลือก **`/drivable_area/overlay`** ในรายการด้านบนของหน้าต่าง
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
