# ข้อมูลและการเทรน SegFormer

Inference ปัจจุบันโหลดโมเดลที่ fine-tune แล้วจาก `camera.model_path` ใน robot profile ไม่มีการโหลด checkpoint ใหม่แทนโดยอัตโนมัติ โมเดลและภาพฝึกจริงไม่อยู่ใน Git นี้ ต้องใช้ artifacts จากผู้จัดทำ

## เครื่องมือที่เกี่ยวข้อง

- `tools/prepare_cvat_dataset.py`: จับคู่ CVAT segmentation-mask export กับเฟรมของวิดีโอต้นฉบับ เก็บ recording group เดียวกัน ไม่สุ่มแบ่งเฟรมจากคลิปเดียวกันเป็นชุดประเมิน
- `tools/train_segmentation.py`: fine-tune SegFormer สองคลาส (`0 non_drivable`, `1 drivable`), default initial checkpoint `nvidia/segformer-b0-finetuned-ade-512-512`
- `road_navigation/segmentation.py`: โหลด processor/model ในเครื่องและคืน drivable probability ให้ runtime

ตัวอย่างจากราก repository หลังติดตั้ง dependencies ใน environment สำหรับ training:

```bash
python -m tools.prepare_cvat_dataset \
  --annotations /path/to/cvat-masks.zip --video /path/to/original-video.mp4 \
  --recording-group recording-01 --output /path/to/new-dataset
python -m tools.train_segmentation \
  --dataset /path/to/new-dataset --output /path/to/new-model \
  --device cuda --epochs 20 --batch-size 2
```

Path และ hyperparameters ในตัวอย่างเป็นวิธีใช้เครื่องมือ ไม่ใช่บันทึกยืนยันค่าที่ใช้สร้าง checkpoint ล่าสุด ต้องรายงานจำนวนภาพ/epochs/checkpoint จริงจาก training artifacts ของผู้จัดทำ การเทรนอาจดาวน์โหลด initial checkpoint และใช้ GPU

Runner นี้รายงาน training loss และใช้ข้อมูลที่ส่งให้ทั้งหมด ไม่ใช่หลักฐาน validation accuracy/IoU ต้องเตรียมชุดทดสอบจาก recording แยกต่างหาก หากต้องการรายงานผลเชิงปริมาณ ห้ามนำ training loss หรือความลื่นของ overlay มาแทนความแม่นยำ
