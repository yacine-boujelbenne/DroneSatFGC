"""
DroneSatFGC — Fire/Smoke Bridge Service
========================================
Deployed on Render. Sits between the MKR1000 and Supabase.

  MKR1000 --(HTTPS)--> this service --(supabase-py)--> Supabase (Postgres + Storage)

What it does, per reading:
  1. Receives sensor data (POST /sensor-data) and a photo (POST /upload-image)
     from the board — tagged with a shared "id" so they can be paired up.
  2. Runs your fire/smoke classifier on the photo, using the exact same
     prediction logic as your test[1].py (imgsz=640, r.probs.top1/top1conf).
  3. Once both pieces for a given id have arrived, uploads the image to the
     Supabase "photos" storage bucket and inserts one combined row (sensors +
     prediction + image URL) into the "readings" table. The dashboard picks
     it up from there in real time via Supabase Realtime.

Local run (for testing before you deploy):
    export SUPABASE_URL="https://xxxx.supabase.co"
    export SUPABASE_SERVICE_KEY="your-secret-key"
    pip install -r requirements.txt
    python app.py

Render deployment:
    Build command: pip install -r requirements.txt
    Start command: gunicorn app:app --bind 0.0.0.0:$PORT --workers 1 --timeout 120
    Env vars (set in the Render dashboard, not in this file): SUPABASE_URL,
    SUPABASE_SERVICE_KEY

Why --workers 1: each worker would load its own copy of the model into
memory. On Render's free tier (512MB RAM) more than one worker will likely
run out of memory. One worker is plenty for a single board syncing every
15 seconds.
"""

import io
import os
import threading
import time

from flask import Flask, request, jsonify
from PIL import Image
from ultralytics import YOLO

from dotenv import load_dotenv
from supabase import create_client

load_dotenv()  # loads a local .env file into os.environ if one exists — this
                # is for convenience when testing on your own machine. On
                # Render there is no .env file, and this line does nothing —
                # the dashboard's Environment Variables are already sitting
                # in os.environ by the time this script runs.

# ============================================================================
# CONFIG
# ============================================================================
MODEL_PATH = "best.pt"          # your trained weights, committed alongside this file
IMG_SIZE = 640                  # matches your test[1].py

SUPABASE_URL = os.environ["SUPABASE_URL"]
SUPABASE_SERVICE_KEY = os.environ["SUPABASE_SERVICE_KEY"]  # the SECRET key
    # (sb_secret_... in Supabase's newer key format) — NOT the publishable/
    # anon one. This bypasses Row Level Security so the server can always
    # write, which is exactly why it must only ever live here as a Render
    # environment variable, never in dashboard.html or any client-side code.

READINGS_TABLE = "readings"
PHOTOS_BUCKET = "photos"

# How long to wait for the "other half" of a reading (sensor data or image)
# before inserting whatever we have anyway, so a dropped packet never
# silently loses data — it just goes in incomplete instead.
PENDING_TIMEOUT_SECONDS = 20

# ============================================================================
# Supabase + model setup — both done once at import time, not per-request
# ============================================================================
supabase = create_client(SUPABASE_URL, SUPABASE_SERVICE_KEY)

print("Loading model...")
model = YOLO(MODEL_PATH)
CLASS_NAMES = model.names  # dynamic — don't hardcode, this always matches your weights
print("Classes:", CLASS_NAMES)

app = Flask(__name__)

# In-memory holding area for readings we've received half of.
# reading_id (str) -> {"sensor": {...}, "sensor_time": t, "image_bytes": b"...",
#                       "prediction": {...}, "image_time": t}
pending = {}
pending_lock = threading.Lock()


@app.route("/", methods=["GET"])
def health_check():
    return jsonify({"status": "DroneSatFGC bridge running", "classes": CLASS_NAMES})


@app.route("/sensor-data", methods=["POST"])
def sensor_data():
    data = request.get_json(force=True)
    reading_id = str(data.get("id"))

    with pending_lock:
        entry = pending.setdefault(reading_id, {})
        entry["sensor"] = data
        entry["sensor_time"] = time.time()

    print(f"[{reading_id}] sensor data received: {data}")
    _try_finalize(reading_id)
    return jsonify({"status": "ok"}), 200


@app.route("/upload-image", methods=["POST"])
def upload_image():
    reading_id = request.args.get("id", "unknown")
    image_bytes = request.get_data()

    prediction = _run_inference(image_bytes)

    with pending_lock:
        entry = pending.setdefault(reading_id, {})
        entry["image_bytes"] = image_bytes
        entry["prediction"] = prediction
        entry["image_time"] = time.time()

    print(f"[{reading_id}] image received, prediction: {prediction['class']} "
          f"({prediction['confidence']:.2%})")
    _try_finalize(reading_id)
    return jsonify({"status": "ok", "prediction": prediction["class"],
                    "confidence": prediction["confidence"]}), 200


def _run_inference(image_bytes):
    """Reuses the exact prediction logic from test[1].py, minus the GUI."""
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


def _try_finalize(reading_id):
    """If both sensor data and an image+prediction are in for this id, push it."""
    with pending_lock:
        entry = pending.get(reading_id)
        if not entry or "sensor" not in entry or "prediction" not in entry:
            return
        record = pending.pop(reading_id)

    _push_to_supabase(reading_id, record, partial=False)


def _push_to_supabase(reading_id, record, partial):
    image_url = None

    if "image_bytes" in record:
        path = f"{reading_id}.jpg"
        try:
            supabase.storage.from_(PHOTOS_BUCKET).upload(
                path,
                record["image_bytes"],
                {"content-type": "image/jpeg", "upsert": "true"},
            )
            image_url = supabase.storage.from_(PHOTOS_BUCKET).get_public_url(path)
        except Exception as exc:
            print(f"[{reading_id}] image upload failed, continuing without it: {exc}")

    sensor = record.get("sensor", {})
    prediction = record.get("prediction", {})

    row = {
        "reading_id": reading_id,
        "temperature": sensor.get("temperature"),
        "humidity": sensor.get("humidity"),
        "pressure": sensor.get("pressure"),
        "illuminance": sensor.get("illuminance"),
        "uva": sensor.get("uva"),
        "uvb": sensor.get("uvb"),
        "uv_index": sensor.get("uvIndex"),
        "image_url": image_url,
        "prediction": prediction.get("class"),
        "confidence": prediction.get("confidence"),
        "probabilities": prediction.get("probabilities"),
        "partial": partial,
    }

    try:
        supabase.table(READINGS_TABLE).insert(row).execute()
        status = "PARTIAL (timed out)" if partial else "complete"
        print(f"[{reading_id}] inserted into Supabase — {status} — "
              f"prediction={prediction.get('class', 'n/a')}")
    except Exception as exc:
        print(f"[{reading_id}] Supabase insert failed: {exc}")


def _cleanup_loop():
    """Flushes anything that's been waiting too long for its other half,
    so a lost packet doesn't just sit in memory forever."""
    while True:
        time.sleep(5)
        now = time.time()
        stale = []
        with pending_lock:
            for reading_id, entry in list(pending.items()):
                last_seen = max(entry.get("sensor_time", 0), entry.get("image_time", 0))
                if now - last_seen > PENDING_TIMEOUT_SECONDS:
                    stale.append((reading_id, pending.pop(reading_id)))
        for reading_id, entry in stale:
            print(f"[{reading_id}] timed out incomplete, pushing partial data")
            _push_to_supabase(reading_id, entry, partial=True)


threading.Thread(target=_cleanup_loop, daemon=True).start()

if __name__ == "__main__":
    # Local testing only — Render runs this via gunicorn instead, which
    # imports `app` directly and never executes this block.
    port = int(os.environ.get("PORT", 5000))
    print(f"Bridge server listening on 0.0.0.0:{port}")
    app.run(host="0.0.0.0", port=port)