# ตรวจการรับส่ง overlay ด้วย ROS 2 Jazzy จริง

การผ่าน unit tests และ `colcon build` อย่างเดียวไม่วัด DDS หรือการแสดงภาพใน viewer
จึงเพิ่ม `tools/probe_ros2_stream.py` สำหรับส่งภาพกล้องสังเคราะห์ผ่าน ROS จริง
พร้อมนับจำนวนภาพที่รับได้เทียบกับเฟรมที่ node ประมวลผลขณะมีผู้รับ

## สภาพแวดล้อมและข้อมูลทดลอง

- Official `ros:jazzy-ros-base` container, digest `sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410`
- จำกัด CPU quota เท่ากับ 1 core, PyTorch 2.6 CPU ใช้ 2 threads
- ภาพสังเคราะห์ที่มี texture 640×480, ส่ง 15 fps; input ของโมเดล 448×256
- โมเดลสังเคราะห์สถาปัตยกรรม SegFormer-B0 สองคลาส ตั้ง classifier ให้รับพื้นทั้งหมดเพื่อใช้วัดการทำงานของ pipeline
- **ไม่ได้ใช้ weights ที่ train แล้วของผู้ใช้ และไม่ได้ประเมินความแม่นยำในการตรวจถนน**
- วัด 20 วินาทีรวมช่วงเริ่ม process แล้วรอรับ output ที่ค้างอีก 2 วินาที
- ทั้งสองรอบใช้ node รุ่นเดียวกัน เปลี่ยนเฉพาะ transport ที่ subscriber รับ
- ใช้ ROS subscriber แบบ headless ไม่ได้เปิด rqt GUI หรือกล้องจริง

## ผลวัดหนึ่งคู่ของการทดลอง

| ค่า | Raw Image | JPEG CompressedImage |
|---|---:|---:|
| เฟรมประมวลผลขณะมีผู้รับ | 34 | 36 |
| ภาพที่ subscriber รับได้ | 27 | 36 |
| เฟรมประมวลผลที่ไม่มาถึง subscriber | 7 | 0 |
| ขนาด payload มัธยฐาน | 921,600 bytes | 58,377 bytes |
| อายุภาพเมื่อรับ มัธยฐาน | 585 ms | 528 ms |
| ช่วงห่างภาพสูงสุด | 1.39 s | 0.69 s |

รอบ raw ที่ทดลองก่อนหน้าพบช่วงห่างสูงสุด 2.63 วินาทีเช่นกัน ขณะที่ status ยังรายงานการประมวลผลต่อเนื่อง
นี่แสดงว่าอายุภาพที่มาถึงต่ำสามารถเกิดร่วมกับภาพขาดช่วงได้
การทดสอบนี้ยังไม่ได้แยกสาเหตุระดับ middleware/transport buffer แต่ยืนยันว่าการลด payload ของ preview
ช่วยหลีกเลี่ยงภาพที่หายในการทดลองนี้ได้ ไม่ใช่หลักฐานว่าความหน่วงบนคอมพิวเตอร์ทุกเครื่องจะเท่ากัน

## แพตช์และการใช้ rqt

node เพิ่ม `/drivable_area/overlay/compressed` เป็น `sensor_msgs/CompressedImage`
JPEG quality เริ่มที่ 80, คงขนาดภาพและ source header เดิม
บีบอัดเฉพาะภาพที่วาดสำหรับแสดงผล ไม่เปลี่ยนภาพเข้าโมเดล, probability, mask หรือพิกัด reference
ทั้ง raw และ compressed ส่งเฉพาะเมื่อมี subscriber และใช้ overlay ที่วาดร่วมกันครั้งเดียว

ติดตั้ง `ros-jazzy-compressed-image-transport` แล้วปิดและเปิด rqt ใหม่
เลือก `/drivable_area/overlay/compressed` ในรายการ topics
log ของ detector ควรแสดง `preview subscribers raw=0, compressed=1`

## รันซ้ำโดยไม่เปิด webcam

ใน repository นี้ โดย source ROS Jazzy และใช้ Python environment ที่มี dependencies ครบ:

```bash
python tools/probe_ros2_stream.py \
  --model /absolute/path/to/husky-segformer \
  --transport raw --seconds 20

python tools/probe_ros2_stream.py \
  --model /absolute/path/to/husky-segformer \
  --transport compressed --seconds 20 --require-all-frames
```

ตัว probe ใช้ namespace สุ่มสำหรับกล้องสังเคราะห์และ detector แยกจาก topics ของหุ่น
ตรวจว่า JPEG decode กลับเป็นภาพขนาดเดิมได้ และรายงาน `missing_processed_images`
`--require-all-frames` ทำให้จบด้วย error หากไม่ได้รับครบหรือไม่ได้รับภาพเลย
รูปแบบการตรวจนี้ไม่ได้วัด GUI rendering, ความหน่วงใน webcam หรือความถูกต้องของโมเดลบนพื้นจริง

ตรวจเพิ่มแล้วว่า workspace ที่สร้างใหม่และ `--update-code` สามารถ build ด้วย colcon ภายใน Jazzy container
และ `ros2 launch husky_vision drivable_area.launch.py --show-args` โหลด launch/config ที่ติดตั้งได้
