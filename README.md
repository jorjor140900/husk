# husk — camera-only road detection baseline

พัฒนาและทดสอบการตรวจจับพื้นที่กับเส้นนำทางจาก **ภาพ / วิดีโอ / webcam โดยไม่ต้องใช้ ROS หรือหุ่นจริง**
ยังไม่มีโมเดล AI ที่ฝึกแล้วใน repository นี้ โค้ดส่วนกลางรองรับ probability map จาก segmentation เพื่อเพิ่มโมเดลภายหลัง

ถ้าทำ label ใน CVAT แล้ว ให้เปิด [notebook ฝึก SegFormer ใน Google Colab](https://colab.research.google.com/github/jorjor140900/husk/blob/work/notebooks/train_segformer_colab.ipynb) แล้วรันช่อง 1–5 ตามลำดับ โดย upload ZIP `Segmentation mask` และวิดีโอต้นฉบับ รองรับหลายคลิปในครั้งเดียว เช่น `part1.zip` + `part1.mov` จนถึง part4 รวม 8 ไฟล์ แล้วฝึกรวมเป็นโมเดลเดียว ไม่ต้อง export ภาพจาก CVAT หรือดาวน์โหลดชุดข้อมูลจาก Codex
Colab มีโหมดใช้ฟรี แต่ GPU ขึ้นกับโควตาและทรัพยากร ถ้าไม่มี GPU notebook จะฝึกต้นแบบสั้น ๆ บน CPU ได้ โมเดล pretrained ต้องดาวน์โหลดจาก Hugging Face ระหว่างรัน ไม่ใช้โมเดลสุ่มแทนเมื่อดาวน์โหลดไม่ได้

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

## ใช้กับ ROS 2 Jazzy

เริ่มใหม่ให้เป็นสัดส่วนได้ด้วย [คู่มือสร้าง workspace ใหม่ `husky_vision_ws` และแพ็กเกจ `husky_vision`](docs/ros2-fresh-start.md)
มีคำสั่งสร้าง source และคัดลอกโมเดลให้ครบ โดยไม่รวม legacy หรือผลทดลอง

มีแพ็กเกจ `husky_drivable_area` สำหรับเปิด webcam และใช้โมเดลเดิมผ่าน ROS topics
ดู [ขั้นตอนติดตั้งใน drivable_area_ws, build, launch และเปิดภาพ](docs/ros2-jazzy.md)
ส่ง mask, overlay, reference แบบพิกเซลและ status
ขั้นถัดไปใช้ [คู่มือคาลิเบรตกล้องและแสดงเส้นหน่วยเมตรใน RViz](docs/ros2-metric-path.md)
มี node แปลงพิกัดบนพื้นราบเมื่อมี calibration และ mount ที่วัดจริง โดย node นี้ไม่ส่งคำสั่งเคลื่อนที่
เริ่มทำ control ต่อได้ด้วย [ชุด Pure Pursuit + หุ่นจำลองใน RViz](docs/ros2-control.md)
ทดลองวงจรปิดและการหยุดเมื่อข้อมูลขาดใน `/husky_demo/` โดยไม่ต้องเปิดกล้องหรือเชื่อมหุ่น
หลังผ่านชุด controller มี `lidar_demo.launch.py` สำหรับทดลองกรองคำสั่งด้วย LaserScan จำลอง
และ rectangle ที่กวาดผ่านระหว่างเคลื่อนที่/เลี้ยว ดูขั้นต่อในคู่มือเดียวกัน
สำหรับงานที่มีปลายทาง ใช้ [คู่มือเก็บแมพและนำทางด้วย Nav2 + AI](docs/ros2-goal-navigation.md)
เริ่มจากขับสำรวจเก็บแมพ แล้วระบุตำแหน่งและปลายทางใน RViz; AI เพิ่มพื้นที่ห้ามผ่านใน costmap
มีโหมดวางเส้นทางอย่างเดียวเป็นค่าเริ่มต้น และแพ็กเกจ C++ `husky_vision_nav2` สำหรับชั้น AI
สำหรับหญ้าที่โมเดลนับเป็นทาง มี [ตัวกรองสีเขียวเสริมแบบเปิดแยกสำหรับ outdoor](docs/ros2-outdoor-grass.md)
ตัดทั้ง mask/path และ AI costmap โดยยังต้องตรวจภาพจริงและข้อจำกัดสี/แสงก่อนใช้งาน
โหมด Nav2 มี `outdoor-nav2-fast.json` ข้ามงานสร้างเส้นพิกเซลและ optical flow
เพื่อลดเวลา detector โดยคง segmentation, ตัวกรองหญ้าและเกณฑ์ข้อมูลสดเดิม
ระหว่างยังไม่มีแมพ ใช้ `navigation_preflight --profile ...` ตรวจแพ็กเกจและค่าตั้งไว้ก่อน
เมื่อมีแมพและเปิด navigation แล้ว เพิ่ม `--live` เพื่อตรวจข้อมูลจริง โดยตัวตรวจไม่ส่ง goal หรือคำสั่งขับ
ระหว่างรอลงพื้นที่ ใช้ `vision_preview.launch.py` เชื่อม webcam/AI กับการดูค่า v/omega ของ Pure Pursuit
ในสมมติฐานกล้องนิ่ง มีภาพและ RViz พร้อมสถานะ โดยไม่สร้าง odometry หรือส่งคำสั่งมอเตอร์
เส้นเหลืองผ่านเกณฑ์อายุควบคุม; เส้นสีส้มแสดงผลภาพที่ช้าเพื่อวิเคราะห์ โดยไม่ให้ candidate
ตรวจความสูงและมุมก้มใน mount ให้ตรงกับการตั้งกล้องปัจจุบันก่อนใช้ระยะที่ฉายลงพื้น
ใช้ `ground_check.launch.py` แสดงกากบาทระยะบนภาพดิบแล้ววัดเทียบเครื่องหมายบนพื้น
ดู [วิธีตรวจระยะ 10/25/50 cm ในห้อง](docs/ros2-metric-path.md#6-ตรวจระยะก่อนทำ-controller)
มี [ชุดเตรียมต่อ Husky ROS1–ROS2](docs/robot-field-kit.md) ด้วย rosbridge gateway,
watchdog บนหุ่น, profile ค่าจริง, launch รวมและ offline link self-test
สร้างชุดด้วย `ros2 run husky_vision prepare_robot_kit --workspace ~/husky_vision_ws`
เริ่ม observe/disarmed; วันจริงต้องเติม topic/TF/geometry และตรวจ sensor/gate ก่อนเปิดคำสั่ง
มี [ผลตรวจ VLP16 จาก bag ของ cpr-a200-1068](docs/vlp16-field-review.md) และเครื่องมือ offline
ตรวจ ring/timing พร้อมแกน deskew รายจุด; ยังไม่เชื่อมส่วนนี้เข้า field controller

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
- `quality_score` เก็บเฉพาะใน metadata เพื่อวินิจฉัย ไม่แสดงเป็นคะแนนบนจออีก เพราะค่านี้สูงได้แม้ mask ผิด ไม่ใช่ AI confidence หรือความมั่นใจว่าขับผ่านได้
- `limited_lookahead` หมายถึงเส้นทางหยุดก่อนสุด ROI; `insufficient_lookahead` และ `start_blocked_or_too_narrow` ไม่มีเส้นทางคืนมา
- `no_smooth_reference` หมายถึงพบเส้นทางแบบจุดใน mask แต่ไม่มีเส้นโค้งในชุดที่ค้นหาซึ่งผ่านการตรวจพื้นที่ทั้งหมด จึงแสดง `NO PATH` และไม่วาดเรฟเดิมค้างไว้ ไม่ได้หมายความว่าพื้นจริงผ่านไม่ได้แน่นอน
- ค่า `processing_ms` วัดการประมวลผล mask/path และรวม inference เมื่อใช้ `--model` ไม่รวมโหลดโมเดล อ่านไฟล์ เขียนไฟล์ หรือ bridge/network
- ภาพวิดีโอที่มีขนาดคี่จะเติมขอบดำด้านขวา/ล่างหนึ่งพิกเซลสำหรับ codec; พิกัดใน JSON ยังคงอิงภาพก่อนเติมขอบ

## สิ่งที่เปลี่ยนจากเวอร์ชันเดิม

1. แยก detector ออกจาก ROS และ trackbar; import แล้วไม่มีการเปิดหน้าต่างหรือเริ่มกล้อง
2. ใช้สี Lab จาก patch ใกล้จุดเริ่มหลาย patch ร่วมกับการขยายพื้นที่จาก seed ตามสีเพื่อนบ้าน และใช้ขอบ Canny เป็นกำแพงกั้นโดยค่าเริ่มต้น ลดการลามจากพื้นขึ้นเสาหรือผนังที่สีใกล้กัน
3. ไม่เติมรูสิ่งกีดขวาง ไม่เติมเต็มทั้งแถว และไม่สร้าง mask จากลิ่มเส้นตรง
4. กรอง probability ตามเวลาโดยชดเชยการเคลื่อนที่ด้วย optical flow และตรวจ photometric consistency; หลักฐานลบของเฟรมปัจจุบันล้างพื้นที่เก่าทันที
5. ใช้เฉพาะ component ที่เชื่อมกับจุดเริ่ม ไม่มีการย้ายจุดเริ่มไปยังพื้นที่อีกฝั่งเมื่อจุดเดิมถูกบัง
6. หาเส้นทางด้วย dynamic programming บนทุกแถวของภาพ ให้น้ำหนักระยะจากขอบมากกว่าคะแนนสี และมีค่าปรับเมื่อเปลี่ยนทิศทาง จึงไม่ไล่ตามจุดสว่างหรือรอยสีเป็นหลัก
7. เลือกเส้นโค้งเรียบจากชุดจำกัด แล้วตรวจทุกจุดและพื้นที่ระหว่างจุดกับ mask ที่เว้นระยะขอบแล้ว หากไม่มีเส้นที่ผ่านการตรวจจะไม่แสดงเรฟ ไม่มีการเฉลี่ยเส้นข้ามสิ่งกีดขวาง เส้นทางเดิมที่ถูก warp เป็นเพียง soft preference

กำแพงจากขอบภาพเป็นข้อแลกเปลี่ยน: ลดการรวมสิ่งกีดขวางสีเหมือนพื้น แต่รอยต่อหรือขอบเงาที่คมและพาดเต็มทางอาจตัดพื้นที่จริงจนไม่มี path ได้เช่นกัน ไม่ได้แยกความหมายของเงา/ขอบต่างระดับ/เสาด้วย AI
ดู [บันทึกสาเหตุและผลเทียบก่อน–หลัง](docs/perception-regression.md) สำหรับตัวอย่างข้อผิดพลาดที่ยังเหลือและข้อจำกัดของผลตรวจ

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
- `use_edge_barrier`: ค่าเริ่มต้น `true` ใช้ขอบภาพกั้นการขยายพื้นที่ ค่า `false` มีไว้เปรียบเทียบพฤติกรรมสีและอาจทำให้ mask ลามเข้าเสา/ผนัง
- `edge_low_threshold`, `edge_high_threshold`: เกณฑ์ Canny ที่ความละเอียด analysis ค่าเริ่มต้น `35`, `80`; ลดแล้วกั้นขอบมากขึ้นแต่เสี่ยงตัดพื้นจริงจาก texture/แสงสะท้อน
- `region_step`: ความต่างสี Lab ระหว่างเพื่อนบ้านที่ยอมให้ขยายจาก seed ค่าเริ่มต้น `6`; ค่าสูงอาจลามเข้าอีกพื้นผิว ไม่ควรเพิ่มเพื่อบังคับให้ทุกภาพมี path
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

Tests ครอบคลุมการอ้อมสิ่งกีดขวาง, ทางแคบ, จุดเริ่มถูกบัง, การไม่กระโดดข้ามรู/มุม, เสาสีใกล้พื้นแต่มีขอบที่เห็นได้, เงาที่เปลี่ยนความสว่างค่อย ๆ, การไม่ไล่เรฟตามสี, การงดเรฟเมื่อมีแต่แนวหักมุม, การล้าง history และการอ่านเขียนภาพ/วิดีโอแบบ headless
การทดสอบเงาคมเต็มความกว้างของเดิมยังคงตรวจพฤติกรรมสีเมื่อปิด edge barrier; ค่าเริ่มต้นที่เปิด barrier อาจปฏิเสธเงาลักษณะนี้โดยตั้งใจ และมีการทดสอบว่าไม่เติมข้ามขอบเพื่อสร้างเส้นทางขึ้นมา
รวมการประมาณเส้นขอบโดยไม่ตีรูสิ่งกีดขวางเป็นขอบนอก, ไม่สร้างขอบปลอมจากขอบภาพ, ค่าเยื้องซ้าย/ขวา และการวาดเส้นที่ไม่แก้ mask/path ต้นฉบับ
Tests สังเคราะห์ยืนยันพฤติกรรมซอฟต์แวร์ แต่ไม่วัดความแม่นยำ outdoor จริง ซึ่งต้องใช้ภาพ/วิดีโอต้นฉบับและ ground truth

RGB baseline ยังสับสนผนังสีเดียวกับพื้น หญ้าหรือพื้นวัสดุใหม่ และไม่วัดความสูง/ความชัน/ความลึกหลุม
นโยบายงานนี้คือพื้นทางเดินสีขาวใต้ตึกผ่านได้ ส่วนพรมทุกสีห้ามผ่าน แต่ baseline ยังไม่จำแนกชนิดวัตถุหรือพรมได้อย่างน่าเชื่อถือ ควรใช้ label ตามนโยบายนี้เมื่อเพิ่มโมเดล segmentation
Optical flow อาจผิดบนพื้นไร้ texture; temporal filtering จึงไม่ใช่ตัวรับประกันความถูกต้อง
เส้นทางจาก `RoadDetection.py` เป็น reference สำหรับทดลองในภาพ
ส่วน ROS มี projection หน่วยเมตรแยกต่างหากที่ต้องคาลิเบรตกล้องและวัด mount ก่อนใช้
ยังไม่มี footprint หน่วยเมตรหรือ vehicle dynamics สำหรับหุ่นจริง
controller ทดลองอยู่ในชุด ROS แยกต่างหาก เริ่มจาก simulation ตามคู่มือด้านบน

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

## ใช้ annotation จาก CVAT โดยไม่ export ภาพ

ส่งออก format `Segmentation mask` แบบไม่รวมภาพ แล้วจับคู่กับวิดีโอต้นฉบับที่ upload เข้า CVAT ได้ โดยไม่ต้องใช้ inference หรือซื้อการ export พร้อมภาพ:

```bash
python -m tools.prepare_cvat_dataset \
  --annotations /path/to/part1.zip \
  --video /path/to/part1.mov \
  --recording-group front-building-session-01 \
  --output outputs/part1-labeled
```

แทน `/path/to/...` ด้วยตำแหน่งไฟล์บนเครื่อง ใช้ output directory ใหม่ในแต่ละครั้ง เครื่องมืออ่านชื่อ `frame_NNNNNN` เป็นเลขเฟรมต้นฉบับเริ่มที่ 0 ไม่เดา offset ไม่ resize เพื่อซ่อนขนาดที่ไม่ตรงกัน ต้องเปิด `contact-sheet.png` หรือ `previews/` ตรวจแนว mask กับภาพจริงก่อนฝึก หาก export เปลี่ยนเลขเฟรมหรือใช้วิดีโอคนละไฟล์ต้องแก้การจับคู่ให้ถูกก่อน

ถ้ารวมหลายคลิป ให้ตั้งชื่อ ZIP ด้านนอกให้มีชื่อหลักตรงกับวิดีโอ เช่น `part2.zip` คู่กับ `part2.mov` แล้วใช้ `--inputs` แทน `--annotations`/`--video`:

```bash
python -m tools.prepare_cvat_dataset \
  --inputs /path/to/part1.zip /path/to/part1.mov /path/to/part2.zip /path/to/part2.mov /path/to/part3.zip /path/to/part3.mov /path/to/part4.zip /path/to/part4.mov \
  --recording-group front-building-session-01 \
  --output outputs/parts1-4-labeled
```

ภาพและ mask จะแยกโฟลเดอร์ตามคลิปเพื่อไม่ให้เลขเฟรมซ้ำทับกัน ส่วน `manifest.json` ที่รากรวมทุกภาพไว้สำหรับฝึกครั้งเดียว เปลี่ยนสี label ในแต่ละ Task ได้เพราะตัวแปลงอ่าน `labelmap.txt` ของแต่ละคลิปเอง แต่ใช้ชื่อ `drivable` และ `non-drivable`/`non_drivable` ให้ตรงกัน

ผลลัพธ์มี `images/`, `masks/`, `manifest.json`, `summary.json` และภาพ preview สำหรับตรวจงาน Mask สำหรับฝึกเป็น PNG ช่องเดียว: `0=non-drivable`, `1=drivable`, `255=ignore` โดยอ่านสีจาก `labelmap.txt` ไม่แปลงสีเป็น grayscale และไม่ใช้ `SegmentationObject` เป็น class mask พื้นที่ background ที่ยังไม่ label คงเป็น ignore ไม่ถือเป็นสิ่งกีดขวาง

ใช้เฉพาะ `images/` และ `masks/` สำหรับฝึก ไม่ใช้ภาพ preview ไม่หารค่า mask ด้วย 255 และย่อ mask ด้วย nearest-neighbor เท่านั้น ชุดนี้ยังไม่มีโมเดลที่ฝึกแล้ว เครื่องมือไม่แบ่ง train/validation/test อัตโนมัติ ให้ใช้ recording group เดียวกันสำหรับคลิปจากการบันทึกครั้งเดียว และเก็บการบันทึกอีกครั้งสำหรับประเมินผลแยก

## ฝึกโมเดลและใช้กับกล้อง

แนะนำให้เริ่มด้วย notebook Colab ที่ลิงก์ด้านบน: ติดตั้ง → อัปโหลด ZIP/MOV → ตรวจภาพคู่ label → ฝึก → ดาวน์โหลด `husky-segformer.zip` โค้ดฝึกใช้ pretrained SegFormer-B0 แล้วเปลี่ยนหัวเป็นสองคลาส กำหนด `do_reduce_labels=False` เพื่อรักษาคลาส 0 และใช้ 255 เป็น ignore ข้อมูลชุดเดียวใช้ฝึกทั้งหมดโดยรายงาน training loss เท่านั้น ไม่มีคะแนน validation ที่สุ่มจากเฟรมใกล้กัน

ทำ label/export แยก Task ใน CVAT ได้ แต่ตอนฝึกแนะนำรวม part1–4 เป็น dataset เดียว แล้วได้โมเดลรุ่นใหม่ตัวเดียว ถ้าเคยฝึก part1 แล้ว เก็บรุ่นเก่าไว้เป็นสำรองและฝึกรอบใหม่จาก pretrained ด้วยข้อมูลรวม notebook ไม่ resume รุ่นเก่าอัตโนมัติ และไม่สร้างโมเดลหนึ่งตัวต่อคลิป ในรายงานการฝึกมี `samples_by_clip` ให้ตรวจว่ารอบนั้นใช้ข้อมูลคลิปใดบ้าง

สำหรับเครื่องฝึกแบบ headless หรือ venv ใหม่ สามารถรันเองได้:

```bash
python -m pip install -r requirements-segmentation.txt
python -m tools.train_segmentation --dataset outputs/part1-labeled --output outputs/husky-segformer --epochs 20
```

บน Ubuntu ที่ใช้ OpenCV แบบ GUI อยู่แล้ว ให้ติดตั้งเฉพาะ dependency ของโมเดล เพื่อคง GUI build เดิม แก้ path ZIP ให้ตรงกับไฟล์ที่ดาวน์โหลดจาก Colab:

```bash
source .venv/bin/activate
python -m pip install --index-url https://download.pytorch.org/whl/cpu 'torch==2.6.0'
python -m pip install 'transformers==4.49.0' 'safetensors>=0.4.3,<1' 'Pillow>=10,<12'
mkdir -p models
unzip ~/Downloads/husky-segformer.zip -d models
python RoadDetection.py --camera 0 --show --config config.example.json --model models/husky-segformer --device cpu
```

ส่ง `--model` เป็นโฟลเดอร์ที่แตกแล้ว ซึ่งมี config, preprocessor และ weights ครบ ใช้ `--device auto` เพื่อเลือก CUDA หากติดตั้ง PyTorch ที่รองรับ GPU ไว้แล้ว ถ้าไม่ส่ง `--model` ยังใช้ baseline เดิมโดยไม่ import torch/transformers
เมื่อใช้โมเดล จะข้ามขั้นสี/Canny และส่ง probability เข้าตัวกรองเวลาและตัวสร้าง reference เดิม ค่า processing_ms รวม inference ด้วย การโหลดโมเดลหรือฝึกด้วยโค้ดนี้ไม่ได้ยืนยันความแม่นยำกลางแจ้ง และยังไม่มีการส่งคำสั่งควบคุมหุ่น

### ลดความหน่วงเมื่อใช้กล้องกับ CPU

ทดลองลดขนาดภาพที่เข้า AI โดยตรงได้โดยไม่ฝึกใหม่ เช่น:

```bash
python RoadDetection.py --camera 2 --show --config config.example.json --model models/husky-segformer --device cpu --model-size 448 256
```

ถ้ายังช้า ลอง `--model-size 320 192` แทน ตัวเลขคือ width height และต้องหาร 32 ลงตัว หากไม่ระบุจะใช้ขนาดที่บันทึกมากับโมเดล `--max-width` หรือย่อหน้าต่างแสดงผลเพียงอย่างเดียวไม่ได้ลดขนาดที่เข้าโมเดล การลดขนาดอาจทำให้เสา/คนขนาดเล็กและขอบถนนหาย จึงต้องเปรียบเทียบภาพจริงด้วย ไม่รับประกันอัตราเร่งบนเครื่องผู้ใช้

กล้อง webcam อ่านต่อเนื่องในอีก thread โดยเก็บไว้เพียงเฟรมล่าสุด ลดการรอประมวลผลเฟรมเก่าค้างสะสม ส่วนไฟล์วิดีโอยังประมวลผลทุกเฟรมตามลำดับ การอ่านแบบล่าสุดไม่ได้ลดเวลา inference ของเฟรมที่กำลังคำนวณ หรือยืนยันว่าจะขจัดความหน่วงภายใน driver/ตัวกล้องได้ทั้งหมด
จอ webcam เพิ่ม `Compute` เป็นเวลา model+mask/path, `proc FPS` เป็น 1000/Compute ซึ่งไม่รวม I/O และ `host age` เป็นเวลาตั้งแต่อ่านเฟรมนั้นสำเร็จจน model/path เสร็จ ก่อนวาด/เขียนไฟล์ ไม่ใช่เวลาจาก sensor ถึงหน้าจอจริง ใน `frames.jsonl` มี `capture_frame_index`, `camera_frames_skipped`, `host_frame_age_ms` เพิ่มเพื่อดูช่วงเฟรมที่ข้ามเมื่อโปรแกรมช้ากว่ากล้อง

ทดสอบส่วนโมเดลด้วย environment ที่ติดตั้ง optional dependencies แล้ว: `python -m unittest discover -s tests -v` การทดสอบใช้ checkpoint ขนาดเล็กที่สร้างใน temporary directory สำหรับตรวจ forward/backward, การรักษา label, save/load และ CLI ไม่ใช่น้ำหนักสำหรับใช้งานจริง การรัน notebook บน Colab และการ fine-tune pretrained model กับข้อมูลจริงต้องตรวจแยกจาก tests เหล่านี้
