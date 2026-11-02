# คิว waypoint สำหรับทางรูป U

เปิด `waypoint_mode:=true` ร่วมกับ `guidance_mode:=image_only` หรือ local_fusion
เมื่อเลือกโหมดคิว การคลิก 2D Goal Pose ใน RViz เป็นการเพิ่มจุดลงคิว ไม่สั่งหุ่นวิ่งทันที
กดจุดเรียงตามทาง U ตั้งแต่ก่อนเข้าโค้ง ผ่านแต่ละโค้ง จนถึงปลายทาง
ตัวคิวส่งทีละจุดให้ image controller แล้วเปลี่ยนไปจุดถัดไปเมื่อถึงภายใน 0.15 m
ตรวจ identity ของ goal ด้วย source stamp เพื่อไม่ให้ status เก่าข้าม waypoint ใหม่

AI ยังต้องรองรับทางแต่ละช่วง หญ้า label non-drivable ไม่ถูกเปลี่ยนให้เป็นทาง
คิวให้ลำดับการอ้อม ไม่ได้ทำให้หุ่นหา topology ของทาง U จาก goal สุดท้ายจุดเดียวเอง
หากไม่เห็นแขนทางหรือ waypoint ห่าง/อยู่ด้านหลังมากเกิน อาจต้องเพิ่มจุดย่อย

## อัปเดต

ใช้ขั้น pull/export/build ฝั่งคอมตาม [คู่มือ junction turn](ros2-junction-turn.md)
ถ้าอัปเดต junction profile และ gateway ไปแล้ว ครั้งนี้อัปเดตเฉพาะ ROS2 ฝั่งคอม
หากยังใช้ profile หมุนสะสม 20° เดิม ต้องเปลี่ยนเป็น junction profile ทั้งสองฝั่งตามคู่มือนั้นก่อน

## เปิดระบบและเรียงจุด

```bash
QT_QPA_PLATFORM=xcb ros2 launch husky_vision vision_field.launch.py \
  profile:="$HOME/husky_vision_ws/robot/robot-nojoy-junction.json" \
  map:="$HOME/husky_vision_ws/maps/outdoor1.yaml" \
  guidance_mode:=image_only waypoint_mode:=true \
  supervised_trial:=true assume_near_field_clear:=false allow_motion:=true
```

1. ตั้ง 2D Pose Estimate ให้ตรงตำแหน่งและทิศหุ่น
2. ใช้ 2D Goal Pose คลิกจุดตามลำดับตลอด U: ตามขาแรก → เข้ามุม → ออกจากมุม → ตามขาอีกฝั่ง → ปลายทาง
3. ตรวจลำดับที่ `/husky_nav/waypoints/status` และเส้นสีฟ้า `Waypoint queue (guide)` ใน RViz
   เส้นสีฟ้าแสดงลำดับจุดเท่านั้น ส่วนเส้นควบคุมจริงยังเป็นเส้นเหลืองจากภาพ
4. ขณะ operator ยัง STOPPED เริ่มคิวเพื่อให้ controller เตรียมจุดแรก:

```bash
ros2 service call /husky_nav/waypoints/start std_srvs/srv/Trigger "{}"
ros2 service call /husky_nav/reset_trial std_srvs/srv/Trigger "{}"
ros2 service call /husky_nav/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /husky_link/set_enabled std_srvs/srv/SetBool "{data: true}"
```

5. กด g ใน console บนหุ่นครั้งเดียว หุ่นจะตามแต่ละจุดเองโดยไม่ต้องกด goal ใหม่เมื่อถึงจุดย่อย
   permission/session เดิมยังมีผลตลอดคิว ไม่เริ่ม session หรือ re-arm เอง

## ดูสถานะและยกเลิก

```bash
ros2 topic echo /husky_nav/waypoints/status --once --full-length
ros2 service call /husky_nav/waypoints/clear std_srvs/srv/Trigger "{}"
```

`current_index` เริ่มนับที่ 0; `queue_completed` หมายถึงถึงครบทุกจุด
clear ขอ cancel goal ปัจจุบันและล้างคิวเมื่อ controller ตอบสำเร็จ รอสถานะ `queue_cleared`
ก่อนเริ่มคิวใหม่ ปุ่ม Space ของ operator ยังคงหยุดการอนุญาตทันทีตามเดิม
ข้อมูล/TF/การสื่อสารผิดเงื่อนไขยังหยุดตามเดิม คิวไม่ข้ามจุดเพื่อหลบ fault

ระหว่างเปลี่ยน goal ย่อย gate ส่งศูนย์เป็นการพักตามปกติ ไม่ latch fault จากการถึงจุด
การรอ reference ของ goal ใหม่มีช่วงเตรียมไม่เกิน 0.5 s พร้อมตรวจ permission/odom/TF ตามเดิม
หากข้อมูลไม่กลับมาภายในช่วงนั้นหรือภาพระบุว่าไม่มีทาง จะหยุดตามเงื่อนไขเดิม
คิวอยู่ในหน่วยความจำ ปิดโปรแกรมแล้วต้องคลิกเก็บจุดใหม่

โหมดเดิมที่รับ goal เดียวใช้ `waypoint_mode:=false` ซึ่งยังเป็นค่าเริ่มต้น
