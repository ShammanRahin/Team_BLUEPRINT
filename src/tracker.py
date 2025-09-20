#!/usr/bin/env python3
import time
import threading
import cv2
import numpy as np
import serial
from flask import Flask, Response, jsonify

# ---------------- CONFIG ----------------
MAIN_SIZE   = (640, 480)   # camera resolution
PROC_SIZE   = (320, 240)   # detection resolution (smaller = faster)
SERIAL_PORT = "/dev/ttyUSB0"   # or /dev/ttyACM0 (check with dmesg | grep tty)
BAUD_RATE   = 115200
SEND_HZ     = 25
STREAM_JPEG_QUALITY = 50
MIN_AREA_PROC = 250
DRAW_OVERLAY = True
# ----------------------------------------

# HSV ranges
RED1_LO, RED1_HI = np.array([0,120,80]),   np.array([10,255,255])
RED2_LO, RED2_HI = np.array([170,120,80]), np.array([180,255,255])
YEL_LO,  YEL_HI  = np.array([20,120,100]), np.array([35,255,255])

# Globals
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
latest_overlay = None
latest_error = 0
latest_color = 2   # 1=red, 0=yellow, 2=none
latest_area  = 0
have_viewer = False
lock = threading.Lock()

# Serial init
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    time.sleep(2)
    print(f"[SERIAL] Connected {SERIAL_PORT}")
except Exception as e:
    print(f"[SERIAL] Serial disabled: {e}")
    ser = None

def open_camera():
    try:
        from picamera2 import Picamera2
        picam2 = Picamera2()
        cfg = picam2.create_video_configuration(main={"size": MAIN_SIZE})
        picam2.configure(cfg)
        picam2.start()
        print("[CAM] PiCamera2 main", MAIN_SIZE)
        class PiCam:
            def read(self):
                f = picam2.capture_array()
                return True, cv2.cvtColor(f, cv2.COLOR_RGB2BGR)
            def release(self): picam2.stop()
        return PiCam()
    except Exception as e:
        print(f"[CAM] Fallback to /dev/video0: {e}")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, MAIN_SIZE[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, MAIN_SIZE[1])
        if not cap.isOpened():
            raise RuntimeError("No camera found")
        return cap

def find_largest(mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, area = None, 0
    for c in cnts:
        a = cv2.contourArea(c)
        if a > area and a >= MIN_AREA_PROC:
            best, area = c, a
    return best, area

def process_loop():
    global latest_overlay, latest_error, latest_color, latest_area
    cam = open_camera()
    w_main, h_main = MAIN_SIZE
    w_p, h_p = PROC_SIZE
    scale_x = w_main / float(w_p)
    scale_y = h_main / float(h_p)
    center_main = w_main // 2

    last_send = 0.0
    send_interval = 1.0 / SEND_HZ

    while True:
        ok, frame_bgr = cam.read()
        if not ok:
            time.sleep(0.01)
            continue

        small = cv2.resize(frame_bgr, PROC_SIZE, interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(hsv, RED1_LO, RED1_HI)
        red_mask |= cv2.inRange(hsv, RED2_LO, RED2_HI)
        yel_mask = cv2.inRange(hsv, YEL_LO, YEL_HI)

        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        yel_mask = cv2.morphologyEx(yel_mask, cv2.MORPH_OPEN, kernel, iterations=1)

        r_cnt, r_area = find_largest(red_mask)
        y_cnt, y_area = find_largest(yel_mask)

        if r_area >= y_area and r_cnt is not None:
            best_cnt, color_code, best_area = r_cnt, 1, r_area  # red
        elif y_cnt is not None:
            best_cnt, color_code, best_area = y_cnt, 0, y_area  # yellow
        else:
            best_cnt, color_code, best_area = None, 2, 0         # none

        error = 0
        overlay = frame_bgr

        if best_cnt is not None:
            x, y, w, h = cv2.boundingRect(best_cnt)
            cx = int((x + w // 2) * scale_x)
            error = cx - center_main

            if DRAW_OVERLAY and have_viewer:
                cv2.rectangle(overlay,
                              (int(x*scale_x), int(y*scale_y)),
                              (int((x+w)*scale_x), int((y+h)*scale_y)),
                              (0,255,0), 2)
                cv2.putText(overlay,
                            f"c:{color_code} err:{error} area:{int(best_area)}",
                            (10,30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0,255,0), 2)

        now = time.time()
        if ser and now - last_send >= send_interval:
            last_send = now
            try:
                ser.write(f"{color_code} {error} {int(best_area)}\n".encode("utf-8"))
            except Exception:
                pass

        with lock:
            latest_overlay = overlay if DRAW_OVERLAY else None
            latest_error = int(error)
            latest_color = color_code
            latest_area  = int(best_area)

def gen_mjpeg():
    global have_viewer
    have_viewer = True
    try:
        while True:
            with lock:
                frame = latest_overlay.copy() if latest_overlay is not None else None
            if frame is None:
                time.sleep(0.05)
                continue
            ok, jpg = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), STREAM_JPEG_QUALITY])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
                       jpg.tobytes() + b"\r\n")
            time.sleep(0.03)
    finally:
        have_viewer = False

# Flask
app = Flask(_name_)

@app.route("/")
def index():
    return "<h2>Tracker</h2><p>/status (JSON) or /stream (MJPEG)</p>"

@app.route("/status")
def status():
    with lock:
        return jsonify({
            "color": latest_color,   # 1=red, 0=yellow, 2=none
            "error": latest_error,
            "area":  latest_area
        })

@app.route("/stream")
def stream():
    return Response(gen_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# Main
if _name_ == "_main_":
    t = threading.Thread(target=process_loop, daemon=True)
    t.start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
