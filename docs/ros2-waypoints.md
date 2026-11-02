# คิว waypoint สำหรับทางรูป U

**อัปเดตตัวกรอง mask รอบนี้ต้องใช้ `--update-code --update-detector`**
เพราะมีการเพิ่มพารามิเตอร์ optical flow ใน detector ด้วย การ update-code อย่างเดียวคง detector เก่าไว้

```bash
cd ~/husk-live
git pull --ff-only origin experiment/local-ai-lidar
python3 tools/create_ros2_workspace.py --workspace "$HOME/husky_vision_ws" \
  --update-code --update-detector
```

จากนั้น build husky_vision และ restart field launch ตามขั้นตอนเดิม

## ลดเส้นกระพริบ

field launch ใช้ `stabilize_mask:=true` เป็นค่าเริ่มต้นสำหรับ navigation:
กรอง probability ข้ามภาพโดย optical flow ก่อน blend (current weight 0.35)
optical flow ใช้ภาพกว้าง 160 พิกเซลแล้ว scale กลับเข้าสู่ภาพจริง เพื่อลดภาระ CPU
หลักฐานปัจจุบันที่ต่ำกว่า probability floor และ colour veto ยังชนะทันที
ไม่ได้เติม binary non-drivable ทั้งหมดให้เป็น free และไม่ยืด source timestamp

เมื่อเส้นเดิมยังสด ตัวคอนโทรลแปลงเส้นเดิมจาก odom กลับเข้าภาพใหม่
ตรวจแถบทั้งความกว้างด้วย mask ปัจจุบันก่อน reuse geometry พร้อม hysteresis
ถ้าค้น candidate ใหม่ไม่สำเร็จแต่เส้นเดิมผ่านการตรวจภาพใหม่ ก็ใช้เส้นที่ตรวจใหม่นี้ได้
ถ้าไม่มีภาพใหม่/TF ใช้ reference ได้ภายใน deadline เดิม 0.5 s เท่านั้น
ภาพใหม่ที่มี veto/unknown บนแถบไม่สามารถทำให้ reference เก่าผ่านการ revalidation
status มี `reference_reused_after_current_mask_check` เพื่อระบุการใช้ geometry เดิม

เส้นแดงใช้ EMA เฉพาะขอบที่ fit ได้ในภาพปัจจุบัน ไม่มีขอบใหม่จะไม่วาดขอบเก่าค้าง
การกรองขอบเป็นงานแสดงผล ไม่เปลี่ยน mask หรือ reference ควบคุม

`waiting_for_goal_guide` ของรุ่นก่อน หรือ `waiting_for_waypoint_or_start` ของรุ่นใหม่
หมายถึงไม่มี goal ที่คอนโทรลใช้อยู่ ต้องตรวจคิวและสั่ง start เมื่อมีจุดแต่ยังไม่ได้เริ่ม
ไม่ใช้เส้นจาก goal ก่อนหน้าเดินต่อหลัง cancel/clear/เปลี่ยน goal

เฟรม stale_input/stale_result หนึ่งเฟรมจะไม่ส่ง packet ล้างผลอื่นที่ source ยังสดไม่เกิน 0.5 s
ไม่ต่ออายุผลเดิม หากครบอายุหรือกล้องขาดยัง invalidate ตามเดิม
status ของ AI เพิ่ม `drivable_pixels` และ `mask_pixels` เพื่อแยก mask ว่างจากการขาด publication

probability smoothing เพิ่มงาน CPU; ถ้าต้องเปรียบเทียบกับชุด fast เดิม ใช้
`stabilize_mask:=false` ตอนเปิด launch ส่วนการ revalidate reference และกรองเส้นแดงยังมีผล

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

## ภาพกลับมาแล้วแต่ฐานยังหยุด: vision pause

ใน image_only ชั้น trial เปิด `pause_on_vision_gap=true`:
เมื่อ gate ปิดชั่วคราวเพราะ start_blocked/no corridor/รอ capture TF/reference เก่า
จะส่งศูนย์และรายงาน `vision_paused_waiting_for_fresh_reference` แทนการ latch fault ค้าง
เมื่อ reference สดกลับมาและผ่าน gate ต่อเนื่อง 0.5 s จึงส่งคำสั่งตามเส้นอีกครั้ง
ต้องยังมี permission สดและ session เดิมทำงานอยู่ ไม่มีการ re-arm, ต่อเวลา หรือ reset งบระยะ/มุม

การกด Space, disable, permission หาย, odom ขาด, ข้อมูลผิดรูปแบบ, gateway/การสื่อสารขาด
และเพดาน trial ยังหยุดตามเดิม ไม่ใช่กรณีที่กลับเดินเองด้วย vision pause
โหมดอื่นยังใช้ fault latch เดิม

หากค้างจาก fault เก่าก่อนอัปเดต ให้หยุด console, อัปเดต/build และ restart field launch
แล้ว reset_trial/enable/g ครั้งใหม่ งบเก่าจะไม่ถูกล้างอัตโนมัติจากเฟรมที่กลับมา

ระหว่างเปลี่ยน goal ย่อย gate ส่งศูนย์เป็นการพักตามปกติ ไม่ latch fault จากการถึงจุด
การรอ reference ของ goal ใหม่มีช่วงเตรียมไม่เกิน 0.5 s พร้อมตรวจ permission/odom/TF ตามเดิม
หากข้อมูลไม่กลับมาภายในช่วงนั้นหรือภาพระบุว่าไม่มีทาง จะหยุดตามเงื่อนไขเดิม
คิวอยู่ในหน่วยความจำ ปิดโปรแกรมแล้วต้องคลิกเก็บจุดใหม่

โหมดเดิมที่รับ goal เดียวใช้ `waypoint_mode:=false` ซึ่งยังเป็นค่าเริ่มต้น

## จุดใกล้และเส้นเลี้ยวเกิน goal

เพิ่ม exact-goal circular candidate สำหรับ goal ใกล้ที่ระยะหน้าเกินจุดเริ่มภาพไม่ถึง 0.5 m
และอยู่ในระยะ 1.75 m พร้อมปรับขั้นต่ำ reference ตามช่วงที่มองเห็นและการปัดกริด
เพื่อไม่ให้รัศมี/มุมแบบขั้นบันไดบังคับเลือกโค้งเลย goal เล็กน้อย
candidate ยังต้องผ่าน mask และ curvature; ไม่มีการย้าย goal หรือเพิ่ม tolerance 0.15 m

ถ้า vision_control_status ยัง tracking และความเร็วไม่เป็นศูนย์ แต่ฐานหยุด ให้ดู
`/husky_control/safety/status` และ `/husky_link/status` เพื่อหาเหตุหลังชั้น controller
เส้นเหลืองกับ private command เพียงอย่างเดียวไม่ยืนยันว่าคำสั่งผ่านถึงฐาน
