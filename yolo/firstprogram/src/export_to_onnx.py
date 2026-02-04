import os
import sys
from pathlib import Path
from ultralytics import YOLO

def main():
    # Get the directory of this script
    script_dir = Path(__file__).parent
    model_path = script_dir / ".." / "best.pt"
    
    # Convert to absolute path
    model_path = model_path.resolve()
    
    # Check if model file exists
    if not model_path.exists():
        print(f"Error: Model file not found at {model_path}")
        print("Please make sure 'best.pt' exists in the firstprogram/ directory.")
        return 1
    
    print(f"Loading model from: {model_path}")
    
    try:
        # Load the trained model
        model = YOLO(str(model_path))
        print("Model loaded successfully.")
        
        # Display model information
        print(f"Model type: {model.task}")
        print(f"Model classes: {model.names}")
        
        # Export to ONNX format
        print("\nExporting model to ONNX format...")
        
        # Export with common settings
        # - imgsz: input image size (default 640)
        # - dynamic: enable dynamic axes for batch size and image dimensions
        # - simplify: simplify ONNX model
        # - opset: ONNX opset version (default 12)
        
        success = model.export(
            format="onnx",
            imgsz=640,           # Input image size
            dynamic=True,        # Enable dynamic axes
            simplify=True,       # Simplify ONNX model
            opset=12,            # ONNX opset version
            verbose=True         # Show export details
        )
        
        if success:
            # Get the exported ONNX file path
            onnx_path = model_path.with_suffix('.onnx')
            if not onnx_path.exists():
                # Sometimes the exported file might be in a different location
                # Try to find it in the same directory as the model
                onnx_files = list(model_path.parent.glob("*.onnx"))
                if onnx_files:
                    onnx_path = onnx_files[0]
            
            print(f"\n✅ Export successful!")
            print(f"ONNX model saved to: {onnx_path}")
            print(f"File size: {onnx_path.stat().st_size / (1024*1024):.2f} MB")
            
            # Print ONNX model information
            print("\nONNX Model Information:")
            print(f"- Input shape: (batch, 3, 640, 640)")
            print(f"- Output format: YOLO detection outputs")
            print(f"- Dynamic axes enabled: Yes")
            print(f"- Opset version: 12")
            
            return 0
        else:
            print("\n❌ Export failed!")
            return 1
            
    except Exception as e:
        print(f"\n❌ Error during export: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())