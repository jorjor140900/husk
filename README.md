# husk — camera-only road detection baseline

พัฒนาและทดสอบการตรวจจับพื้นที่กับเส้นนำทางจาก **ภาพ / วิดีโอ / webcam โดยไม่ต้องใช้ ROS หรือหุ่นจริง**
ยังไม่มีโมเดล AI ที่ฝึกแล้วใน repository นี้ โค้ดส่วนกลางรองรับ probability map จาก segmentation เพื่อเพิ่มโมเดลภายหลัง

## ติดตั้งและเริ่มใช้งาน

ต้องใช้ Python 3.10 ขึ้นไป ตัวอย่าง Linux/macOS:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python RoadDetection.py --image path/to/frame.jpg --output outputs/image-01
python RoadDetection.py --video path/to/clip.mp4 --output outputs/clip-01 --save-masks
python RoadDetection.py --camera 0 --max-frames 300 --output outputs/webcam-01
```

ถ้ายังไม่มีวิดีโอ สร้างฉากจำลองที่มีเงาและสิ่งกีดขวางเพื่อทดลองตัวรันได้:

```bash
python examples/make_demo.py --output outputs/demo-input
python RoadDetection.py --image outputs/demo-input/scene.png --output outputs/demo-result
```

ฉากนี้เป็นข้อมูลสังเคราะห์ ไม่ใช่ผลทดสอบความแม่นยำ outdoor

Windows ใช้ `.venv\Scripts\activate` แทน `source` ข้างต้น
แต่ละครั้งต้องใช้ output directory ใหม่ เพื่อไม่เขียนทับผลทดลองเดิม
ค่าเริ่มต้นทำงานแบบ headless และไม่เปิดหน้าต่าง
ถ้าต้องการหน้าต่างบนเครื่อง desktop ให้แทนที่แพ็กเกจ headless ด้วย GUI build แล้วเพิ่ม `--show`:

```bash
python -m pip uninstall -y opencv-python-headless
python -m pip install 'opencv-python>=4.10,<5'
python RoadDetection.py --camera 0 --show --output outputs/webcam-gui-01
```

อย่าติดตั้ง OpenCV headless และ GUI พร้อมกันใน venv เดียว เครื่อง Linux ต้องมี desktop/display และ system libraries ที่ GUI build ต้องการด้วย

## ผลลัพธ์

- ภาพ: `overlay.png`, `raw_mask.png`, `mask.png`, `navigable_mask.png`, `probability.npy`, `result.json`
- วิดีโอ/webcam: `overlay.avi`, `frames.jsonl`, และ PNG mask แต่ละเฟรมเมื่อใช้ `--save-masks`
- ทุกการทดลองบันทึก `config.json` และ `summary.json`; วิดีโอผลลัพธ์ใช้ MJPEG และคง FPS ของต้นฉบับเมื่อมีข้อมูล
- สีเขียวคือพื้นที่ลักษณะคล้ายพื้น, เส้นสีฟ้าอ่อนบาง ๆ คือขอบพื้นที่หลังเว้นระยะ
- **เส้นสีเหลือง** คือเส้นทางอ้างอิงที่ผ่านการตรวจ mask; **เส้นสีน้ำเงินแนวตั้ง** คือแนวกึ่งกลางหุ่นที่สมมติจากตำแหน่งกล้อง
- **เส้นตรงสีแดงซ้าย–ขวา** คือขอบนอกของ drivable mask ที่ประมาณเป็นเส้นตรง วาดเฉพาะด้านและช่วงที่มีข้อมูลรองรับ ไม่ใช้ขอบภาพแทนขอบถนนที่มองไม่เห็น
- เส้นสีแดงเป็นภาพประกอบ ไม่ได้ใช้สร้าง mask หรือบังคับเส้นสีเหลืองผ่านสิ่งกีดขวาง หากหาขอบด้านใดไม่ได้ เส้นด้านนั้นจะไม่ปรากฏ
- ค่า `dx` เทียบตำแหน่งเส้นสีเหลืองกับเส้นกลางสีน้ำเงินที่แถว look-ahead เดียวกัน: บวก=เรฟอยู่ขวา, ลบ=เรฟอยู่ซ้าย หน่วยพิกเซล `n/a` หมายถึงยังไม่มี path ครอบคลุมแถวนั้น
- แนวกึ่งกลางเริ่มที่กลางภาพ โดยสมมติว่ากล้องติดตั้งตรงแนวหน้าหุ่น ยังไม่ใช่การวัดตำแหน่งหรือ heading ของหุ่นจริง และไม่มีการส่งคำสั่งเคลื่อนที่
- เส้นทางเรียงจากใกล้ไปไกล อยู่ในพิกัด **พิกเซลของภาพหลัง resize** ไม่ใช่เมตรหรือมุม steering
- `valid_image_path` หมายถึงมีเส้นทางที่สอดคล้องกับ mask และระยะมองล่วงหน้าที่กำหนด ไม่ใช่การยืนยันว่าพื้นจริงปลอดภัย
- `quality_score` เป็น heuristic จากหลักฐานในภาพและความยาวเส้นทาง ไม่ใช่ calibrated AI confidence
- `limited_lookahead` หมายถึงเส้นทางหยุดก่อนสุด ROI; `insufficient_lookahead` และ `start_blocked_or_too_narrow` ไม่มีเส้นทางคืนมา
- ค่า `processing_ms` วัดเฉพาะ detector ไม่รวมอ่านไฟล์ เขียนไฟล์ หรือ bridge/network
- ภาพวิดีโอที่มีขนาดคี่จะเติมขอบดำด้านขวา/ล่างหนึ่งพิกเซลสำหรับ codec; พิกัดใน JSON ยังคงอิงภาพก่อนเติมขอบ

## สิ่งที่เปลี่ยนจากเวอร์ชันเดิม

1. แยก detector ออกจาก ROS และ trackbar; import แล้วไม่มีการเปิดหน้าต่างหรือเริ่มกล้อง
2. ใช้สี Lab จาก patch ใกล้จุดเริ่มหลาย patch และให้น้ำหนักความสว่างอ่อนลง ไม่มี Canny เป็นกำแพงตัดทุกเงาหรือรอยต่อ
3. ไม่เติมรูสิ่งกีดขวาง ไม่เติมเต็มทั้งแถว และไม่สร้าง mask จากลิ่มเส้นตรง
4. กรอง probability ตามเวลาโดยชดเชยการเคลื่อนที่ด้วย optical flow และตรวจ photometric consistency; หลักฐานลบของเฟรมปัจจุบันล้างพื้นที่เก่าทันที
5. ใช้เฉพาะ component ที่เชื่อมกับจุดเริ่ม ไม่มีการย้ายจุดเริ่มไปยังพื้นที่อีกฝั่งเมื่อจุดเดิมถูกบัง
6. หาเส้นทางด้วย dynamic programming บนทุกแถวของภาพ ตรวจพื้นที่ระหว่างจุดทั้งหมดและเว้นระยะจากขอบ จึงไม่เฉลี่ยเส้นขอบข้ามสิ่งกีดขวาง
7. เส้นทางเดิมที่ถูก warp แล้วเป็นเพียง soft preference; ไม่มีการทำ spline ที่อาจลัดออกนอก mask

## ปรับค่า

```bash
python RoadDetection.py --video path/to/clip.mp4 --config config.example.json --output outputs/tuned-01
python RoadDetection.py --image path/to/frame.jpg --clearance-px 12 --goal-x 0.6 --output outputs/tuned-02
```

`config.example.json` มีค่าทั้งหมด ค่า CLI เขียนทับเฉพาะ field ที่ระบุ:

- `max_width`: ความกว้างภาพผลลัพธ์สูงสุด รักษาสัดส่วน; ค่าเริ่มต้น 640
- `analysis_width`: ความกว้างสูงสุดที่ใช้ประเมินสี; ค่าเริ่มต้น 320
- `horizon_fraction`: ตัดบริเวณด้านบนแบบคงที่ ควรปรับตามมุมกล้อง; ยังไม่ได้ประมาณ horizon จริง
- `seed_x_fraction`, `seed_y_fraction`: จุดพื้นใกล้หุ่นที่ **สมมติว่าขับได้**; อย่าให้ตัวหุ่นหรือวัตถุบังจุดนี้
- `chroma_tolerance`: ยิ่งสูงยิ่งรับสีต่างกันได้มาก แต่เพิ่มโอกาสลามออกนอกพื้น
- `probability_threshold`, `current_probability_floor`: เกณฑ์รับพื้นที่และเกณฑ์หลักฐานปัจจุบัน; ค่าสีเป็น appearance score ไม่ใช่ความน่าจะเป็นที่ผ่านการ calibrate
- `temporal_alpha`: น้ำหนักเฟรมใหม่; ค่าสูงตอบสนองเร็ว ใช้ `--no-temporal` เพื่อทดสอบทีละเฟรมอิสระ
- `clearance_px`: ระยะเผื่อจากขอบบนภาพผลลัพธ์ **ไม่ใช่ขนาด Husky จริง**
- `max_lateral_step`: ระยะเปลี่ยน x สูงสุดต่อหนึ่งแถวภาพ; วิธีนี้ยังไม่รองรับทางที่ต้องย้อนลงภาพหรือเลี้ยวกลับ
- `goal_x_fraction`: ทิศทางเป้าหมายซ้าย/ขวาบนภาพ ไม่ได้เลือก mission goal ให้อัตโนมัติ
- `robot_center_x_fraction`: ตำแหน่งเส้นกลางหุ่นสีน้ำเงินในแนวนอน ค่าเริ่มต้น `0.5` คือกลางภาพ เป็นค่าคนละตัวกับ seed/goal ใช้ปรับภายหลังเมื่อทราบการติดตั้งกล้อง
- `lookahead_y_fraction`: แถวที่ใช้เทียบเรฟกับแนวหุ่น ค่าเริ่มต้น `0.65` ของความสูงภาพ ถ้า path ไม่ถึงแถวนี้ ค่า `dx` เป็น `n/a`
- `min_path_height_fraction`: ระยะมองล่วงหน้าขั้นต่ำเทียบกับความสูงภาพ

## เพิ่ม AI ภายหลัง

แยกการทำนาย mask ออกจากการสร้างเส้นทางไว้แล้ว ตัวอย่าง Python:

```python
from road_navigation import RoadDetector

detector = RoadDetector()
# frame: uint8 BGR HxWx3; probability: float HxW ในช่วง [0, 1]
# probability ต้องผ่าน sigmoid/softmax และ resize กลับมาเท่าภาพ input แล้ว
result = detector.process(frame, drivable_probability=probability)
```

ถ้ามี obstacle mask ที่ยืนยันได้ ส่ง `blocked_mask=obstacles` โดย nonzero หมายถึงห้ามผ่าน
mask นี้มีสิทธิ์ veto เหนือ probability และ history; การย่อภาพรักษาพิกเซล blocked แม้มีขนาดเล็ก
การ resize probability อย่างเดียวอาจเฉลี่ยรายละเอียดเล็ก ๆ หายไป จึงควรส่ง hard obstacle mask แยกเมื่อมีข้อมูลนั้น
สำหรับภาพเดี่ยวใช้ `--probability map.npy` และ `--blocked-mask obstacles.png` ได้ด้วย
เรียก `detector.reset()` ก่อนประมวลผลคลิปใหม่ที่ไม่ต่อเนื่อง

ขั้นถัดไปของโมเดล: เก็บวิดีโอจริงหลายสภาพแสง → ทำ label พื้นที่ที่อนุญาต/สิ่งกีดขวาง/ไม่ทราบ → fine-tune segmentation เช่น SegFormer-B0 → เปรียบเทียบกับ baseline บนคลิปที่แยกจากชุดฝึก
ให้แยกชุดฝึก/ทดสอบตามคลิปหรือสถานที่ และวัด false positive ในพื้นที่ห้ามผ่าน, IoU, ความแกว่งเส้นทาง และ latency
ห้ามใช้ screenshot ที่มีสี overlay เป็นภาพฝึกแทนภาพต้นฉบับ

## ทดสอบและขอบเขต

```bash
python -m unittest discover -s tests -v
```

Tests ครอบคลุมการอ้อมสิ่งกีดขวาง, ทางแคบ, จุดเริ่มถูกบัง, การไม่กระโดดข้ามรู/มุม, เงาบนพื้นจำลอง, การล้าง history และการอ่านเขียนภาพ/วิดีโอจริงแบบ headless
รวมการประมาณเส้นขอบโดยไม่ตีรูสิ่งกีดขวางเป็นขอบนอก, ไม่สร้างขอบปลอมจากขอบภาพ, ค่าเยื้องซ้าย/ขวา และการวาดเส้นที่ไม่แก้ mask/path ต้นฉบับ
Tests สังเคราะห์ยืนยันพฤติกรรมซอฟต์แวร์ แต่ไม่วัดความแม่นยำ outdoor จริง ซึ่งต้องใช้ภาพ/วิดีโอต้นฉบับและ ground truth

RGB baseline ยังสับสนผนังสีเดียวกับพื้น หญ้าหรือพื้นวัสดุใหม่ และไม่วัดความสูง/ความชัน/ความลึกหลุม
Optical flow อาจผิดบนพื้นไร้ texture; temporal filtering จึงไม่ใช่ตัวรับประกันความถูกต้อง
เส้นทางนี้เป็น reference สำหรับทดลองในภาพ ยังไม่มี camera calibration, footprint หน่วยเมตร, vehicle dynamics หรือคำสั่งควบคุมหุ่น

โค้ด ROS 2 เดิมเก็บไว้ที่ `legacy/RoadDetection_ros2.py` เพื่ออ้างอิงและเปรียบเทียบ ยังมีพฤติกรรมเดิมและ dependency ROS เดิม ไม่ได้ใช้ใน runner ใหม่นี้

## ตรวจคลิปจริงและเตรียมข้อมูล

เครื่องมือเพิ่มเติมต้องมี `ffmpeg` และ `ffprobe` ใน PATH:

```bash
python -m tools.review_videos --manifest clips.json --output outputs/review-01 --sample-every 2 --workers 2
```

`clips.json` เป็นรายการวิดีโอพร้อมจำนวนเฟรมและ FPS ที่ตรวจจากไฟล์จริง เช่น:

```json
[{"name":"clip1","path":"path/to/clip.mp4","frames":300,"fps":30.0}]
```

เครื่องมือประมวลผลทุกเฟรมต่อเนื่อง แยก history แต่ละคลิป ส่งออก MP4 เปรียบเทียบภาพดิบ/ผล baseline, JSONL รายเฟรม, snapshot โค้ด/config และคัดภาพต้นฉบับตามช่วงเวลาสำหรับทำ label
`--sample-every` มีผลเฉพาะการคัดภาพ ไม่ได้ข้ามเฟรมในการทดสอบ detector
จำนวนเฟรมที่มี path, ค่า heuristic และการเคลื่อนของเป้าหมายในภาพไม่ใช่ accuracy หรือคะแนนความนิ่งที่หักการเคลื่อนกล้องแล้ว
ภาพที่คัดยังไม่มี label; อย่าใช้ mask จาก baseline เป็น ground truth และเก็บคลิปจากการบันทึกเดียวกันไว้ใน recording group เดียวเมื่อแบ่ง dataset

ใช้ `python -m tools.review_probes --help` เพื่อส่งออกร่าง sparse label และวินิจฉัยเกณฑ์สีจากจุดที่ตรวจด้วยสายตา โดย class `255` หมายถึงพื้นที่ยังไม่ label ซึ่งต้อง ignore
ไฟล์วิดีโอ ผลทดลอง และ label ที่เกี่ยวข้องกับไฟล์แนบเก็บใน `outputs/` ซึ่งถูก ignore ไว้
