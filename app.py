import os
import io
import time
import urllib.parse
from flask import Flask, request, jsonify
from PIL import Image
from ultralytics import YOLO
from supabase import create_client, Client

# ============================================================================
# CONFIGURATION - Environment variables (Set these in your Render Dashboard)
# ============================================================================
SUPABASE_URL = os.environ.get("SUPABASE_URL")
SUPABASE_KEY = os.environ.get("SUPABASE_KEY")
MODEL_PATH = os.environ.get("MODEL_PATH", "best.pt")
IMG_SIZE = 640

# Initialize Supabase Client
supabase: Client = create_client(SUPABASE_URL, SUPABASE_KEY)

# Initialize YOLO model
print("Loading YOLOv8 Model...")
try:
    model = YOLO(MODEL_PATH)
    CLASS_NAMES = model.names
    print(f"Model loaded successfully. Classes: {CLASS_NAMES}")
except Exception as e:
    print(f"Error loading model: {e}")
    model = None
    CLASS_NAMES = {}

app = Flask(__name__)

# Temporary in-memory staging dictionary for mapping asynchronous requests
pending_readings = {}

@app.route("/", methods=["GET"])
def health_check():
    return jsonify({"status": "Render Cloud API Running", "yolo_loaded": model is not None, "classes": CLASS_NAMES}), 200

@app.route("/sensor-data", methods=["POST"])
def sensor_data():
    try:
        data = request.get_json(force=True)
        reading_id = str(data.get("id"))
        
        if not reading_id:
            return jsonify({"error": "Missing reading id"}), 400

        if reading_id not in pending_readings:
            pending_readings[reading_id] = {}
            
        pending_readings[reading_id]["sensor"] = data
        
        print(f"[{reading_id}] Cloud received sensor metrics.")
        return _process_if_complete(reading_id)
        
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/upload-image", methods=["POST"])
def upload_image():
    try:
        reading_id = request.args.get("id")
        if not reading_id:
            return jsonify({"error": "Missing id parameter"}), 400
            
        image_bytes = request.get_data()
        if not image_bytes:
            return jsonify({"error": "Empty image binary"}), 400

        # Run AI Inference directly inside the cloud worker context
        prediction_payload = _run_inference(image_bytes)

        if reading_id not in pending_readings:
            pending_readings[reading_id] = {}

        pending_readings[reading_id]["image_bytes"] = image_bytes
        pending_readings[reading_id]["prediction"] = prediction_payload

        print(f"[{reading_id}] Cloud processed image. Class variant target: {prediction_payload['class']}")
        return _process_if_complete(reading_id)

    except Exception as e:
        return jsonify({"error": str(e)}), 500

def _run_inference(image_bytes):
    if model is None:
        return {"class": "unknown", "confidence": 0.0, "probabilities": {}}
        
    img = Image.open(io.BytesIO(image_bytes)).convert("RGB")
    results = model(img, imgsz=IMG_SIZE, verbose=False)
    r = results[0]

    probs = r.probs.data.cpu().numpy()
    best_id = int(r.probs.top1)
    best_conf = float(r.probs.top1conf)
    best_class = CLASS_NAMES[best_id]

    probabilities = {CLASS_NAMES[i]: float(p) for i, p in enumerate(probs)}

    return {
        "class": best_class,
        "confidence": best_conf,
        "probabilities": probabilities,
    }

def _process_if_complete(reading_id):
    entry = pending_readings.get(reading_id)
    
    # Check if both structural segments are caught
    if not entry or "sensor" not in entry or "prediction" not in entry:
        return jsonify({"status": "staged", "message": "Awaiting matching packet stream context"}), 200

    # Pop out of tracking memory matrix once both entities exist
    record = pending_readings.pop(reading_id)
    
    try:
        image_url = None
        # 1. Pipeline binary stream into Supabase Storage Bucket
        if "image_bytes" in record:
            storage_path = f"photos/{reading_id}.jpg"
            
            # Remove any existing identical paths safely
            try:
                supabase.storage.from_("photos").upload(
                    path=storage_path,
                    file=record["image_bytes"],
                    file_options={"content-type": "image/jpeg"}
                )
            except Exception:
                pass # Already uploaded or caught structural error
            
            # Construct standard deterministic public URL access signature
            image_url = f"{SUPABASE_URL}/storage/v1/object/public/photos/{storage_path}"

        sensor = record.get("sensor", {})
        prediction = record.get("prediction", {})

        # 2. Insert metrics unified straight into the database matrix row layout
        db_payload = {
            "id": int(reading_id),
            "temperature": float(sensor.get("temperature", 0)),
            "humidity": float(sensor.get("humidity", 0)),
            "pressure": float(sensor.get("pressure", 0)),
            "illuminance": float(sensor.get("illuminance", 0)),
            "uva": float(sensor.get("uva", 0)),
            "uvb": float(sensor.get("uvb", 0)),
            "uv_index": float(sensor.get("uvIndex", 0)),
            "image_url": image_url,
            "prediction": prediction.get("class"),
            "confidence": prediction.get("confidence"),
            "probabilities": prediction.get("probabilities"),
            "partial": False
        }

        supabase.table("readings").insert(db_payload).execute()
        return jsonify({"status": "complete", "synced_id": reading_id}), 200

    except Exception as db_err:
        print(f"Database sync operation breakdown error: {db_err}")
        return jsonify({"error": "Failed storage database push routine", "details": str(db_err)}), 500

if __name__ == "__main__":
    # Internal port target bound globally for deployment runtimes
    port = int(os.environ.get("PORT", 5000))
    app.run(host="0.0.0.0", port=port)