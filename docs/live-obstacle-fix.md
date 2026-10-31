# แก้กรอบสิ่งกีดขวางค้างเมื่อ localization ปรับตำแหน่ง

โหมด image/AI steering เปลี่ยนชั้นสิ่งกีดขวางที่ **global planner ใช้จริง** เป็น
`husky_vision_nav2::OdomObstacleLayer` ยังรับ scan สดจาก `/husky_nav/mapping_scan`
การเปลี่ยนนี้อยู่ใน costmap ก่อน inflation ไม่ใช่การซ่อนสีใน RViz

## พฤติกรรมที่แก้

ObstacleLayer เดิมสะสม occupied cells ในพิกัด `map` เมื่อตำแหน่ง `map -> odom`
เปลี่ยน จุดใหม่ถูกวางคนละตำแหน่งกับจุดเก่า ลำแสงใหม่อาจไม่ผ่านจุดเก่าอีก
ทำให้จุดเก่าและ inflation รอบจุดนั้นยังขวาง planner อยู่

ชั้นใหม่แปลง scan เข้า `odom` ณ timestamp ต้นฉบับ เก็บ measured hits ในกรอบนี้
และฉายทั้งชุดเข้า map ใหม่ทุก costmap update ตาม `map -> odom` ปัจจุบัน
รวม bounds ทั้งตำแหน่งเก่าและใหม่เพื่อให้ Nav2 คำนวณ static map และ inflation ใหม่ครบ
เมื่อ AMCL ปรับตำแหน่ง จึงไม่ทิ้ง occupied cells และวง inflation ไว้ที่ตำแหน่งเก่า

- จุดสิ่งกีดขวางสดถูก mark ตั้งแต่ scan แรกที่ประมวลผล ไม่มีการรอยืนยันหลายเฟรม
- ล้าง hit เก่าเมื่อมี **finite measured ray ที่ผ่านเซลล์นั้น** แล้ว mark จุดปัจจุบันทั้งหมด
- `+inf` ไม่ล้างจุดเก่า และไม่มีการลบสิ่งกีดขวางด้วย timeout
- คง footprint clearing ตามขอบตัวหุ่นที่ตั้งไว้ใน Nav2 เดิม ไม่สร้างวงตัดจุดเพิ่มเอง
- scan/TF ขาดหรือผิดทำให้ costmap ไม่ current พร้อมคงสิ่งกีดขวางที่ทราบไว้
- ชั้นนี้เพิ่มเฉพาะ lethal costs; static obstacles และ unknown cells จากชั้นอื่นยังคงอยู่
- footprint, inflation radius, ความเร็ว, permission และ watchdog ใช้ค่าตาม profile เดิม

ใช้ชั้นใหม่นี้เฉพาะ global costmap ใน `steering_mode=vision`
โหมด Nav2 steering เดิมยังใช้ ObstacleLayer เดิม

## อัปเดตฝั่งคอม ROS2 ได้ตอนที่ไม่มีหุ่น

```bash
cd ~/husk-live
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py \
  --workspace "$HOME/husky_vision_ws" --update-code --with-navigation

source /opt/ros/jazzy/setup.bash
source "$HOME/husky_vision_ws/.venv/bin/activate"
cd ~/husky_vision_ws
python /usr/bin/colcon build --symlink-install \
  --packages-select husky_vision_nav2 husky_vision
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

ต้อง build ทั้งสอง packages เพื่อให้มี plugin ใหม่และ launch ที่เรียกใช้
export จะสำรอง source เดิมและคง robot profiles, โมเดล และ camera calibration ไว้
การแก้นี้ไม่เปลี่ยน ROS1 gateway จึงอัปเดตเฉพาะฝั่งคอม

เมื่อใช้งานครั้งหน้า restart `vision_field.launch.py` ตาม profile/flags เดิม
เพื่อสร้าง costmap ใหม่และโหลด plugin ใหม่ การเปลี่ยน plugin ใช้การ restart
หลังเปิดแล้วอ่านชื่อชั้นที่ planner ใช้อยู่ได้ด้วย:

```bash
ros2 param get /global_costmap/global_costmap obstacle_layer.plugin
```

ต้องได้ `husky_vision_nav2::OdomObstacleLayer`
การ reset trial, enable link และเริ่ม operator session ยังใช้ขั้นตอนเดิม

## ขอบเขตผล

ทดสอบ native C++ บน Jazzy Nav2 1.3.14 (source commit
`d5d768b7b38a05e7cfd2b6c693b786b402161bed`) ผ่าน 13 cases สำหรับชั้นใหม่
และ 4 cases ของ VisionMaskLayer เดิม ใช้ LayeredCostmap, ObstacleLayer เดิม และ InflationLayer จริง
สร้างกรณี map->odom เลื่อน 1 m ที่ทำให้ชั้นเดิมทิ้ง lethal cell เก่าไว้ได้
ชั้นใหม่คืนเซลล์เก่าเป็นค่าจาก base map ขณะที่จุดสิ่งกีดขวางปัจจุบันยังเป็น lethal ตามเดิม
ทดสอบการโหลด plugin, footprint clearing, scan เก่า/ผิด, unknown returns และ TF expiry ของ AMCL ด้วย
รวมการ replay scan geometry จริงทั้ง 34 เฟรมด้วย clock/TF จำลอง

นี่เป็นการแก้การสะสมในกรอบ map ระหว่าง localization corrections
ไม่ได้จำแนกว่าจุดดิบเป็นพื้น ตัวหุ่น วัตถุจริง หรือ sensor artifact
ถ้าจุดผิดยังถูกส่งมาสดอย่างต่อเนื่อง จะยังถูก mark; ไม่ลบมันจากการเดาว่าหน้างานโล่ง
ส่วนการแก้จุดดิบต้องอาศัย geometry/ข้อมูลที่สัมพันธ์กับภาพจริง

Bag ที่มี 34 scans บันทึกขณะหุ่นอยู่นิ่งและหลังออกจากสถานที่เกิดเหตุ
การนำ ranges จริงมา replay ด้วย clock และ map correction จำลองใช้ตรวจ implementation
ไม่ได้ยืนยันว่าหุ่นจริงขณะเคลื่อนที่หรือทุกสาเหตุของกรอบผิดได้รับการแก้แล้ว
ชุด supervised goal ยังมี LiDAR final safety ไม่ผ่าน physical validation ตามเดิม
