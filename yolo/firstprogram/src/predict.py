from ultralytics import YOLO
import cv2

# Load the trained model
model = YOLO("/home/xdc/vscode/runs/train6/weights/best.pt")  # or use your best.pt from runs directory

# Video path
video_path = "/home/xdc/vscode/armor/demoVedio.mp4"
output_path = "/home/xdc/vscode/armor/detected_demo3.mp4"

# Open the video
cap = cv2.VideoCapture(video_path)

# Get video properties
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

# Create VideoWriter for saving results
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

# Process video frame by frame
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
        
    # Perform detection
    results = model.predict(
        source=frame,
        conf=0.3,  # confidence threshold
        imgsz=640,  # inference size
        save=False,  # don't save individual frames
    )
    
    # Plot bounding boxes on frame
    annotated_frame = results[0].plot()
    
    # Display and save frame
    cv2.imshow("Armor Plate Detection", annotated_frame)
    out.write(annotated_frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"Detection complete. Result saved to: {output_path}")
