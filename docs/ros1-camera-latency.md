# ตรวจความหน่วงกล้อง ROS1 -> Jazzy

ข้อมูลจาก cpr-a200-1068: กล้อง 640x480 MJPEG ต่อ USB กับ ROS1 Noetic,
AI อยู่บน Jazzy และใช้ rosbridge ผ่าน SSH tunnel ในเครือข่าย IoT
ต้องตรวจชนิดลิงก์จริงแยกจากชื่อเครือข่าย: เครื่องหนึ่งอาจต่อ ethernet อีกเครื่องต่อ wifi

## ผลวัดก่อนแก้

| เงื่อนไข | กล้องบน ROS1 | Image ที่ ROS2 ได้รับ |
| --- | --- | --- |
| เปิด AI | ~15 FPS, ช่วงห่างสูงสุด 80 ms | ~1.5–3.5 FPS, ขาดช่วงถึง ~1.9 s |
| ปิด AI | ยังใช้ source 15 FPS | ~4.2 FPS แต่ยังขาดช่วงถึง ~1.9 s |
| ลด source เป็น 5 FPS, ปิด AI | ~5 FPS, ช่วงห่างสูงสุด 201 ms | ~0.7 FPS, ขาดช่วงถึง ~2.6 s |

จึงไม่ใช่ inference เพียงอย่างเดียว ต้องแยก ROS1 compressed publisher/gateway,
rosbridge/tunnel/network, client decode และ CPU/DDS ฝั่งรับ
ผลนี้ยังไม่พิสูจน์ว่า Wi-Fi เป็นสาเหตุเดียว การวัดด้วย ros2 topic hz เองก็มีต้นทุนบนเครื่องรับ
ตัวเลข Frame age ใน detector คืออายุจาก source stamp ถึงจบการประมวลผล/ส่งผล
ไม่ใช่เวลาของ inference และไม่รวมเวลาจอแสดงผลที่เกิดหลัง publish

## การแก้ซอฟต์แวร์

- ROS1 compressed subscriber ใช้ buffer 2 MiB, queue=1, tcp_nodelay
- แยก wall-clock watchdog/permission/state loop ออกจาก image/JSON telemetry thread
  เพื่อไม่ให้การ encode/publish ภาพเป็นงานในลูปตรวจคำสั่งหมดอายุ
- ROS1 จำกัดอัตราส่งภาพด้วย `image_max_fps` (default 10), เก็บภาพล่าสุดเพียงช่องเดียว
  และทิ้งภาพต้นทางเก่ากว่า `image_max_age_sec` (default 0.5 s)
- client ขอ rosbridge image queue=1/throttle 100 ms; telemetry อื่นไม่ throttle เพิ่ม
- status เพิ่ม `camera_transport`: received_frames, source_age_at_receipt_ms,
  last_gap_ms, max_gap_ms, receipt_age_ms และ decode_publish_ms
- คง source timestamps เดิม ไม่มีการขยาย path timeout หรือ restamp ภาพเพื่อให้ผ่าน

ภาพและ control ยังใช้ WebSocket เดียวกัน การแก้นี้ไม่รับรอง deadline บน Wi-Fi
watchdog ใน Python ยังขึ้นกับ OS/GIL/process ที่ทำงานได้ ฐานหุ่นต้องมี watchdog/E-stop ของตัวเอง
ต้องวัดหลังติดตั้งจริงก่อนเปิดการเคลื่อนที่

## อัปเดตทั้งสองฝั่ง

หยุดเฉพาะ gateway ฝั่ง ROS1 และ robot_link/detector ฝั่ง Jazzy ก่อนอัปเดต
กล้องและ LiDAR เปิดค้างได้ ตัวอย่างนี้ใช้ workspace จริงของผู้ใช้บนหุ่นชื่อ `~/husky_ws`

บน Jazzy ในโฟลเดอร์ Git ที่มี RoadDetection.py:

```bash
git pull --ff-only origin work
python3 tools/create_ros2_workspace.py --workspace ~/husky_vision_ws --update-code
scp ros1/husky_vision_bridge/scripts/robot_gateway.py \
  administrator@10.82.12.224:~/husky_ws/robot_gateway.media-update.py
scp ros1/husky_vision_bridge/launch/bridge.launch \
  administrator@10.82.12.224:~/husky_ws/bridge.media-update.launch
```

แทน IP ถ้าเปลี่ยนแล้ว บนเครื่องหุ่นใน terminal ที่ source ROS1/workspace:

```bash
source /opt/ros/noetic/setup.bash
source ~/husky_ws/devel/setup.bash
python3 - <<'PY'
from pathlib import Path
import shutil
import subprocess
root = Path(subprocess.check_output(['rospack', 'find', 'husky_vision_bridge'], text=True).strip())
updates = [(Path.home()/'husky_ws/robot_gateway.media-update.py', root/'scripts/robot_gateway.py'),
           (Path.home()/'husky_ws/bridge.media-update.launch', root/'launch/bridge.launch')]
if not (root/'package.xml').is_file() or any(not a.is_file() or not b.is_file() for a, b in updates):
    raise SystemExit('Package/source file missing; no files updated')
for source, target in updates:
    backup = target.with_name(target.name+'.before-media-update')
    if not backup.exists():
        shutil.copy2(target, backup)
    shutil.copy2(source, target)
    print('Updated:', target)
PY
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME
export ROS_IP=192.168.131.1
roslaunch husky_vision_bridge bridge.launch \
  profile_path:="$HOME/husky_ws/bridge-camera-observe.json" \
  allow_robot_output:=false image_max_fps:=5 image_max_age_sec:=0.5
```

บน Jazzy terminal ใหม่:

```bash
cd ~/husky_vision_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
python /usr/bin/colcon build --symlink-install --packages-select husky_vision
source install/local_setup.bash
ros2 launch husky_vision robot_link.launch.py base_frame:=base_link forward_commands:=false
```

เปิด SSH tunnel ที่ใช้ทดสอบค้างไว้ หากเปลี่ยนเป็น LAN ต้องปิด tunnel เดิมแล้วเปิดใหม่โดยใช้ IP สาย LAN
การมีสายเสียบอยู่ไม่ได้ทำให้ tunnel ที่เชื่อมไป IP Wi-Fi เดิมย้ายลิงก์เอง
ตรวจเส้นทางด้วย `ip route get ROBOT_IP` ก่อนเทียบผล

## วัดหลังแก้

บนหุ่นวัด raw และ compressed rate อย่างละประมาณ 10 s:

```bash
rostopic hz /husky_vision_camera/image_raw
rostopic hz /husky_vision_camera/image_raw/compressed
```

บน Jazzy วัดตอนปิด AI ก่อน แล้วจึงวัดซ้ำตอนเปิด AI ด้วย resolution/thread เดิม:

```bash
ros2 topic hz /husky_link/image_raw
ros2 topic echo /husky_link/status --once --full-length
```

กด Ctrl+C เพื่อจบ hz แต่ละคำสั่ง เปรียบเทียบ source age และช่องว่างระหว่างเฟรมด้วย
FPS เฉลี่ยอย่างเดียวอาจซ่อนช่วงที่ภาพหายหลายวินาที; max_gap_ms เป็นค่าสูงสุดสะสมตั้งแต่เริ่ม client
source_age_at_receipt_ms วัดก่อน decode/publish; decode_publish_ms วัดฝั่งแปลงภาพ/ส่ง DDS
ถ้าต้นทางนิ่งแต่ปลายทางยังหาย ให้ทดสอบ LAN ที่ route ถูกต้องก่อนปรับโมเดลเพิ่ม

## ผลทดสอบที่ทำแล้ว

unit tests ตรวจ latest-only slot, source stamp, การทิ้งภาพเก่า/ผิดเวลา/ใหญ่เกิน และทดสอบ
บล็อก image serialization ขณะที่ local watchdog ยังหยุดและ disarm ได้
ทดสอบจริงกับ Noetic/official rosbridge 0.11.17 ใน container แยก ด้วย JPEG สังเคราะห์ 252,633 bytes
ได้ประมาณ 9.3 FPS, observed max image gap 124 ms และ max source age 109 ms ในช่วงทดสอบ 6 s
link loss ทำให้ receiver จำลองหยุดประมาณ 266 ms; deadman release ประมาณ 42 ms
รวมตรวจ fresh-command + deadman loss และการไม่เริ่มใหม่เองผ่านด้วย
นี่เป็นสภาพทดสอบ local ไม่ใช่ผล Wi-Fi หรือการเบรกของ Husky จริง และไม่รับรอง latency ของเครื่องผู้ใช้
