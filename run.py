import cv2

from ultralytics import YOLO

# Load the model
model = YOLO(r"/home/ming/Desktop/yolo/runs/detect/train3/weights/best.pt")

source = r"/home/ming/Desktop/yolo/armour/Video_1768570695685_454_1(1).avi"
#detect for zhuangjiban
results =model(
    source,
    stream=True,
)

for result in results:
    plotted = result.plot()
    cv2.imshow("results",plotted)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
