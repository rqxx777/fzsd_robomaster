#Python usage example for YOLOv8 model inference
from ultralytics import YOLO
##example
# Create a new YOLO model from scratch
model = YOLO("yolo26n.yaml")
# Load a pretrained YOLO model (recommended for training)
model = YOLO("yolo26n.pt")
# Train the model using the 'coco8.yaml' dataset for 3 epochs
results = model.train(data="coco8.yaml", epochs=3)
# Evaluate the model's performance on the validation set
results = model.val()
# Perform object detection on an image using the model
results = model("https://ultralytics.com/images/bus.jpg")
# Export the model to ONNX format
success = model.export(format="onnx")



##Modes:
###1.Train
####pretrain
model = YOLO("yolo26n.pt")  # pass any model type
results = model.train(epochs=5)

####start fram the beginning
model = YOLO("yolo26n.yaml")
results = model.train(data="coco8.yaml", epochs=5)

####resume training
model = YOLO("last.pt")
results = model.train(resume=True)


###2.Validate
####val after training
# Load a YOLO model
model = YOLO("yolo26n.yaml")
# Train the model
model.train(data="coco8.yaml", epochs=5)
# Validate on training data
model.val()

####val on other date sets
# Load a YOLO model
model = YOLO("yolo26n.yaml")
# Train the model
model.train(data="coco8.yaml", epochs=5)
# Validate on separate data
model.val(data="path/to/separate/data.yaml")


###3.Predict
####predict from the source code
import cv2
from PIL import Image

from ultralytics import YOLO

model = YOLO("model.pt")
# accepts all formats - image/dir/Path/URL/video/PIL/ndarray. 0 for webcam
results = model.predict(source="0")
results = model.predict(source="folder", show=True)  # Display preds. Accepts all YOLO predict arguments
# from PIL
im1 = Image.open("bus.jpg")
results = model.predict(source=im1, save=True)  # save plotted images
# from ndarray
im2 = cv2.imread("bus.jpg")
results = model.predict(source=im2, save=True, save_txt=True)  # save predictions as labels
# from list of PIL/ndarray
results = model.predict(source=[im1, im2])

####use the results
# results would be a list of Results object including all the predictions by default
# but be careful as it could occupy a lot memory when there're many images,
# especially the task is segmentation.
# 1. return as a list
results = model.predict(source="folder")

# results would be a generator which is more friendly to memory by setting stream=True
# 2. return as a generator
results = model.predict(source=0, stream=True)

for result in results:
    # Detection
    result.boxes.xyxy  # box with xyxy format, (N, 4)
    result.boxes.xywh  # box with xywh format, (N, 4)
    result.boxes.xyxyn  # box with xyxy format but normalized, (N, 4)
    result.boxes.xywhn  # box with xywh format but normalized, (N, 4)
    result.boxes.conf  # confidence score, (N, 1)
    result.boxes.cls  # cls, (N, 1)

    # Segmentation
    result.masks.data  # masks, (N, H, W)
    result.masks.xy  # x,y segments (pixels), List[segment] * N
    result.masks.xyn  # x,y segments (normalized), List[segment] * N

    # Classification
    result.probs  # cls prob, (num_class, )

# Each result is composed of torch.Tensor by default,
# in which you can easily use following functionality:
result = result.cuda()
result = result.cpu()
result = result.to("cpu")
result = result.numpy()


###4.Export
####export to different formats
model = YOLO("yolo26n.pt")
# export to onnx format
success = model.export(format="onnx")
model.export(format="onnx", dynamic=True)
# export to openvino format
success = model.export(format="openvino")
# export to tensorRT format
success = model.export(format="tensorrt")
# export to coreML format
success = model.export(format="coreml")
# export to paddle format
success = model.export(format="paddle")
# export to torchscript format
success = model.export(format="torchscript")
# export to saved_model format
success = model.export(format="saved_model")
# export to tfjs format
success = model.export(format="tfjs")


###5.Track
# Load a model
model = YOLO("yolo26n.pt")  # load an official detection model
model = YOLO("yolo26n-seg.pt")  # load an official segmentation model
model = YOLO("path/to/best.pt")  # load a custom model
# Track with the model
results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True)
results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True, tracker="bytetrack.yaml")


###6.Benchmark
from ultralytics.utils.benchmarks import benchmark
# Benchmark
benchmark(model="yolo26n.pt", data="coco8.yaml", imgsz=640, half=False, device=0)


##Trainer
###example
from ultralytics.models.yolo.detect import DetectionPredictor, DetectionTrainer, DetectionValidator

# trainer
trainer = DetectionTrainer(overrides={})
trainer.train()
trained_model = trainer.best

# Validator
val = DetectionValidator(args=...)
val(model=trained_model)

# predictor
pred = DetectionPredictor(overrides={})
pred(source=SOURCE, model=trained_model)

# resume from last weight
overrides["resume"] = trainer.last
trainer = DetectionTrainer(overrides=overrides)
