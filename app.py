"""
DroneSatFGC — Fire/Smoke Bridge Service
========================================
Deployed on Render. Sits between the ESP32 and Supabase.

  ESP32 --(HTTPS)--> this service --(supabase-py)--> Supabase (Postgres + Storage)

What it does, per reading:
  1. Receives sensor data and a photo together (POST /ingest) as multipart/form-data.
     The legacy split endpoints remain available while hardware is migrated.
  2. Runs your fire/smoke classifier on the photo, using the exact same
     prediction logic as your test[1].py (imgsz=640, r.probs.top1/top1conf).
  3. Uploads the image to the Supabase "photos" storage bucket and inserts one
     combined row (sensors + prediction + image URL) into the "readings"
     table. The dashboard picks it up via Supabase Realtime.

New telemetry columns (run once in Supabase SQL editor if missing):
    alter table public.readings
      add column if not exists pm1 double precision,
      add column if not exists pm2_5 double precision,
      add column if not exists pm10 double precision,
      add column if not exists gas_raw double precision,
      add column if not exists latitude double precision,
      add column if not exists longitude double precision,
      add column if not exists sim_signal double precision,
      add column if not exists device_status jsonb,
      add column if not exists device_log text;

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
import json
import math
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
MAX_IMAGE_BYTES = 5 * 1024 * 1024
MAX_REQUEST_BYTES = MAX_IMAGE_BYTES + 64 * 1024

# ============================================================================
# Supabase + model setup — both done once at import time, not per-request
# ============================================================================
supabase = create_client(SUPABASE_URL, SUPABASE_SERVICE_KEY)

print("Loading model...")
model = YOLO(MODEL_PATH)
CLASS_NAMES = model.names  # dynamic — don't hardcode, this always matches your weights
print("Classes:", CLASS_NAMES)

app = Flask(__name__)
app.config["MAX_CONTENT_LENGTH"] = MAX_REQUEST_BYTES

# In-memory holding area for readings we've received half of.
# reading_id (str) -> {"sensor": {...}, "sensor_time": t, "image_bytes": b"...",
#                       "prediction": {...}, "image_time": t}
pending = {}
pending_lock = threading.Lock()


@app.route("/", methods=["GET"])
def health_check():
    return jsonify({"status": "DroneSatFGC bridge running", "classes": CLASS_NAMES})


@app.route("/ingest", methods=["POST"])
def ingest():
    """Receive one atomic sensor reading and JPEG in a multipart request."""
    if not request.mimetype == "multipart/form-data":
        return jsonify({"error": "Content-Type must be multipart/form-data"}), 415

    reading_id = request.form.get("reading_id", "").strip()
    valid_id = (
        reading_id
        and len(reading_id) <= 128
        and reading_id.isascii()
        and all(char.isalnum() or char in "._-" for char in reading_id)
    )
    if not valid_id:
        return jsonify({
            "error": "reading_id must contain 1-128 ASCII letters, digits, '.', '_', or '-'"
        }), 400

    image = request.files.get("image")
    image_bytes = None
    if image is not None:
        image_bytes = image.read(MAX_IMAGE_BYTES + 1)
        if not image_bytes:
            return jsonify({"error": "image is empty"}), 400
        if len(image_bytes) > MAX_IMAGE_BYTES:
            return jsonify({"error": "image exceeds 5 MiB limit"}), 413

    try:
        raw_device_status = request.form.get("device_status", "{}")
        device_status = json.loads(raw_device_status)
        if not isinstance(device_status, dict):
            raise ValueError("device_status must be a JSON object")
        allowed_components = {"wifi", "camera", "bme280", "dust", "gas", "gps", "sim"}
        allowed_states = {"ok", "no_fix", "unavailable", "disabled"}
        device_status = {
            component: state
            for component, state in device_status.items()
            if component in allowed_components
            and isinstance(state, str)
            and state in allowed_states
        }
        sensor = {
            "temperature": _optional_float("temperature"),
            "humidity": _optional_float("humidity"),
            "pressure": _optional_float("pressure"),
            "illuminance": _optional_float("illuminance"),
            "uva": _optional_float("uva"),
            "uvb": _optional_float("uvb"),
            "uvIndex": _optional_float("uv_index", "uvIndex"),
            "pm1": _optional_float("pm1"),
            "pm2_5": _optional_float("pm2_5", "pm25"),
            "pm10": _optional_float("pm10"),
            "gas_raw": _optional_float("gas_raw", "gas"),
            "latitude": _optional_float("latitude", "lat"),
            "longitude": _optional_float("longitude", "lon", "lng"),
            "sim_signal": _optional_float("sim_signal", "csq"),
            "device_status": device_status,
            "device_log": request.form.get("device_log", "")[:2000],
        }
        prediction = _run_inference(image_bytes) if image_bytes else {}
    except (ValueError, json.JSONDecodeError) as exc:
        return jsonify({"error": str(exc)}), 400
    except Exception as exc:
        print(f"[{reading_id}] inference failed: {exc}")
        return jsonify({"error": "inference failed on server"}), 502

    record = {"sensor": sensor, "prediction": prediction}
    if image_bytes:
        record["image_bytes"] = image_bytes
    row = _push_to_supabase(
        reading_id,
        record,
        partial=not bool(image_bytes),
        require_image_backup=bool(image_bytes),
    )
    if row is None:
        return jsonify({"error": "failed to persist reading"}), 502

    if prediction:
        print(f"[{reading_id}] multipart reading received, prediction: "
              f"{prediction['class']} ({prediction['confidence']:.2%})")
    else:
        print(f"[{reading_id}] telemetry-only reading received")
    return jsonify({
        "status": "ok",
        "reading_id": reading_id,
        "prediction": prediction.get("class"),
        "confidence": prediction.get("confidence"),
        "image_url": row["image_url"],
        "partial": row["partial"],
    }), 200


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
    try:
        with Image.open(io.BytesIO(image_bytes)) as source:
            if source.format != "JPEG":
                raise ValueError("image is not JPEG encoded")
            source.verify()
        with Image.open(io.BytesIO(image_bytes)) as source:
            img = source.convert("RGB")
    except Exception as exc:
        raise ValueError("image is not a valid JPEG") from exc

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


def _optional_float(*field_names):
    for name in field_names:
        value = request.form.get(name)
        if value is None or value.strip() == "":
            continue
        try:
            number = float(value)
        except ValueError as exc:
            raise ValueError(f"{name} must be numeric") from exc
        if not math.isfinite(number):
            raise ValueError(f"{name} must be finite")
        return number
    return None


def _try_finalize(reading_id):
    """If both sensor data and an image+prediction are in for this id, push it."""
    with pending_lock:
        entry = pending.get(reading_id)
        if not entry or "sensor" not in entry or "prediction" not in entry:
            return
        record = pending.pop(reading_id)

    _push_to_supabase(reading_id, record, partial=False)


def _persist_reading_row(reading_id, row):
    existing = (
        supabase.table(READINGS_TABLE)
        .select("id")
        .eq("reading_id", reading_id)
        .limit(1)
        .execute()
    )
    if existing.data:
        (
            supabase.table(READINGS_TABLE)
            .update(row)
            .eq("id", existing.data[0]["id"])
            .execute()
        )
    else:
        supabase.table(READINGS_TABLE).insert(row).execute()


def _push_to_supabase(reading_id, record, partial, require_image_backup=False):
    image_url = None
    storage_failed = False

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
            storage_failed = True
            print(f"[{reading_id}] image upload failed: {exc}")

    if storage_failed and require_image_backup:
        return None

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
        "pm1": sensor.get("pm1"),
        "pm2_5": sensor.get("pm2_5"),
        "pm10": sensor.get("pm10"),
        "gas_raw": sensor.get("gas_raw"),
        "latitude": sensor.get("latitude"),
        "longitude": sensor.get("longitude"),
        "sim_signal": sensor.get("sim_signal"),
        "device_status": sensor.get("device_status"),
        "device_log": sensor.get("device_log"),
        "image_url": image_url,
        "prediction": prediction.get("class"),
        "confidence": prediction.get("confidence"),
        "probabilities": prediction.get("probabilities"),
        "partial": partial or storage_failed,
    }

    try:
        _persist_reading_row(reading_id, row)
        status = "PARTIAL" if row["partial"] else "complete"
        print(f"[{reading_id}] inserted into Supabase — {status} — "
              f"prediction={prediction.get('class', 'n/a')}")
        return row
    except Exception as exc:
        # Free-tier / schema lag: retry without optional debug columns.
        message = str(exc).lower()
        if "device_status" in message or "device_log" in message:
            fallback = dict(row)
            fallback.pop("device_status", None)
            fallback.pop("device_log", None)
            try:
                _persist_reading_row(reading_id, fallback)
                print(f"[{reading_id}] inserted without device_status/device_log "
                      f"(add those columns in Supabase for full diagnostics)")
                return fallback
            except Exception as nested:
                print(f"[{reading_id}] Supabase insert failed: {nested}")
                return None
        print(f"[{reading_id}] Supabase insert failed: {exc}")
        return None


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