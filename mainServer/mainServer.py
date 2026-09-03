import sys
import cv2
import socket
import struct
import math
import os
import re
import time
import threading
import mysql.connector
import numpy as np

import easyocr
import torch

from ultralytics import YOLO

from datetime import datetime

from PyQt6 import uic
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap

# =========================================
# 통신 모듈
# =========================================
from AI_network import AIServer
from Camera_network import CameraReceiver
import ESP32_network
from ESP32_network import ESP32Server

from PyQt6.QtWidgets import (
    QApplication,
    QDialog,
    QLabel,
    QVBoxLayout,
    QTableWidgetItem,
    QHeaderView,
    QMessageBox,
    QAbstractItemView
)


# =========================================
# ESP32 UDP 설정
# 카메라 방향 제어용 서보 ESP32
# =========================================
ESP32_IP = "192.168.0.2"
ESP32_PORT = 4210

# 서보모터 제어 범위
# servo1 = 좌/우(PAN), servo2 = 상/하(TILT)
SERVO_MIN_ANGLE = 50
SERVO_MAX_ANGLE = 130
SERVO_INITIAL_PAN = 100
SERVO_INITIAL_TILT = 100
SERVO_STEP = 2
SERVO_INTERVAL_MS = 100


# =========================================
# ESP32-CAM UDP 영상 수신 설정
#
# ESP32-CAM은 Wi-Fi로 JPEG 프레임을
# 여러 UDP 패킷으로 나누어 5005번 포트에 전송한다.
# 이 영상은 관리자 UI의 lblCameraView에만 표시한다.
# =========================================
ESP32_CAM_UDP_IP = "0.0.0.0"
ESP32_CAM_UDP_PORT = 5005

ESP32_CAM_MAGIC_NUMBER = 0x43414D31
ESP32_CAM_HEADER_FORMAT = "<IIHHH"
ESP32_CAM_HEADER_SIZE = struct.calcsize(
    ESP32_CAM_HEADER_FORMAT
)


class ESP32CamUDPReceiver(threading.Thread):

    def __init__(
        self,
        ip=ESP32_CAM_UDP_IP,
        port=ESP32_CAM_UDP_PORT
    ):
        super().__init__(
            name="ESP32CamUDPReceiver",
            daemon=True
        )

        self.ip = ip
        self.port = port

        self.stop_event = threading.Event()
        self.frame_lock = threading.Lock()

        self.latest_frame = None
        self.last_error = ""
        self.last_sender = ""

        self.frames = {}
        self.sock = None

    def get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None

            return self.latest_frame.copy()

    def get_status_text(self):
        if self.last_error:
            return self.last_error

        if self.latest_frame is None:
            return (
                "ESP32-CAM UDP 영상 수신 대기 중... "
                f"(PORT {self.port})"
            )

        return ""

    def stop(self):
        self.stop_event.set()

        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass

    def _cleanup_old_frames(self):
        now = time.monotonic()

        old_ids = [
            frame_id
            for frame_id, frame_info in self.frames.items()
            if now - frame_info["time"] > 1.0
        ]

        for frame_id in old_ids:
            self.frames.pop(
                frame_id,
                None
            )

        if len(self.frames) > 10:
            ordered_ids = sorted(
                self.frames,
                key=lambda frame_id:
                self.frames[frame_id]["time"]
            )

            for frame_id in ordered_ids[:-5]:
                self.frames.pop(
                    frame_id,
                    None
                )

    def run(self):
        try:
            self.sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM
            )

            self.sock.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_RCVBUF,
                1024 * 1024
            )

            self.sock.bind(
                (
                    self.ip,
                    self.port
                )
            )

            self.sock.settimeout(
                0.2
            )

            print(
                f"[ESP32-CAM] UDP 영상 수신 시작: "
                f"{self.ip}:{self.port}"
            )

            while not self.stop_event.is_set():

                try:
                    packet, addr = self.sock.recvfrom(
                        2048
                    )

                except socket.timeout:
                    self._cleanup_old_frames()
                    continue

                except OSError as error:
                    if self.stop_event.is_set():
                        break

                    self.last_error = (
                        "ESP32-CAM UDP 수신 오류: "
                        f"{error}"
                    )
                    print(
                        "[ESP32-CAM UDP ERROR]",
                        error
                    )
                    break

                if len(packet) < ESP32_CAM_HEADER_SIZE:
                    continue

                try:
                    (
                        magic,
                        frame_id,
                        chunk_index,
                        total_chunks,
                        payload_size
                    ) = struct.unpack(
                        ESP32_CAM_HEADER_FORMAT,
                        packet[:ESP32_CAM_HEADER_SIZE]
                    )

                except struct.error:
                    continue

                if magic != ESP32_CAM_MAGIC_NUMBER:
                    continue

                if total_chunks <= 0:
                    continue

                if chunk_index >= total_chunks:
                    continue

                payload = packet[
                    ESP32_CAM_HEADER_SIZE:
                    ESP32_CAM_HEADER_SIZE + payload_size
                ]

                if len(payload) != payload_size:
                    continue

                self.last_sender = addr[0]

                frame_info = self.frames.get(
                    frame_id
                )

                if (
                    frame_info is None
                    or frame_info["total"] != total_chunks
                ):
                    frame_info = {
                        "total": total_chunks,
                        "chunks": {},
                        "time": time.monotonic()
                    }

                    self.frames[frame_id] = (
                        frame_info
                    )

                frame_info["chunks"][
                    chunk_index
                ] = payload

                if (
                    len(frame_info["chunks"])
                    != total_chunks
                ):
                    self._cleanup_old_frames()
                    continue

                if not all(
                    index in frame_info["chunks"]
                    for index in range(total_chunks)
                ):
                    self._cleanup_old_frames()
                    continue

                jpeg_data = b"".join(
                    frame_info["chunks"][index]
                    for index in range(total_chunks)
                )

                np_data = np.frombuffer(
                    jpeg_data,
                    dtype=np.uint8
                )

                frame = cv2.imdecode(
                    np_data,
                    cv2.IMREAD_COLOR
                )

                self.frames.pop(
                    frame_id,
                    None
                )

                if frame is None:
                    continue

                with self.frame_lock:
                    self.latest_frame = frame

                self.last_error = ""

                self._cleanup_old_frames()

        except OSError as error:
            if not self.stop_event.is_set():
                if getattr(error, "errno", None) == 98:
                    self.last_error = (
                        "UDP 5005 포트가 이미 사용 중입니다. "
                        "별도로 실행한 esp_32.py가 있다면 종료하세요."
                    )
                else:
                    self.last_error = (
                        "ESP32-CAM UDP 소켓 오류: "
                        f"{error}"
                    )

                print(
                    "[ESP32-CAM UDP SOCKET ERROR]",
                    error
                )

        except Exception as error:
            if not self.stop_event.is_set():
                self.last_error = (
                    "ESP32-CAM UDP 수신기 오류: "
                    f"{error}"
                )

                print(
                    "[ESP32-CAM UDP FATAL]",
                    error
                )

        finally:
            if self.sock is not None:
                try:
                    self.sock.close()
                except Exception:
                    pass

                self.sock = None

            print(
                "[ESP32-CAM] UDP 영상 수신기 종료"
            )


# =========================================
# 주차면 번호판 자동 인식 설정
# =========================================
PARKING_CAMERA_DEVICE = "/dev/video2"
MONITOR_CAMERA_DEVICE = "/dev/video0"
PARKING_MODEL_PATH = "models/license_plate_detector.pt"

PARKING_CAPTURE_WIDTH = 1280
PARKING_CAPTURE_HEIGHT = 720
PARKING_CAPTURE_FPS = 30

PARKING_ROIS = [
    (425, 95, 565, 165),
    (575, 95, 700, 165),
    (705, 95, 825, 165),
    (270, 310, 490, 450),
    (525, 310, 710, 450),
    (735, 310, 960, 450)
]

PARKING_SLOT_NAMES = [
    "A1", "A2", "A3",
    "B1", "B2", "B3"
]

PARKING_SCAN_DELAY = 0.05
PARKING_CONFIRM_HITS = 1
PARKING_CLEAR_MISSES = 3
PARKING_YOLO_CONFIDENCE = 0.20
PARKING_OCR_CONFIDENCE = 0.30


pending_exit_vehicles = []
pending_exit_lock = threading.Lock()


def pack_slot_flags_binary(flags):

    normalized_flags = [
        1
        if int(flag) != 0
        else 0
        for flag in flags
    ]

    if len(normalized_flags) != 6:
        raise ValueError(
            "주차자리 정보는 반드시 6개여야 합니다."
        )

    return bytes(
        normalized_flags
    )


ESP32_network.pack_slot_flags = (
    pack_slot_flags_binary
)


def connect_db():

    return mysql.connector.connect(
        host="localhost",
        user="root",
        password="2314",
        database="project01"
    )


class ParkingPlateWorker(threading.Thread):

    def __init__(self):
        super().__init__(
            name="ParkingPlateWorker",
            daemon=True
        )

        self.stop_event = threading.Event()
        self.frame_lock = threading.Lock()
        self.latest_frame = None

        self.current_roi_index = 0
        self.camera_ready = False
        self.models_ready = False
        self.last_error = ""
        self.entered_last4_cache = set()
        self.entered_last4_cache_time = 0.0

        self.slot_states = []

        for slot_name in PARKING_SLOT_NAMES:
            self.slot_states.append({
                "slot": slot_name,
                "candidate_last4": "",
                "candidate_hits": 0,
                "missing_hits": 0,
                "confirmed_last4": "",
                "confirmed_car_number": "",
                "plate_box": None,
                "observed_last4": "",
                "yolo_confidence": 0.0,
                "ocr_confidence": 0.0,
                "last_match_attempt": 0.0
            })

    @staticmethod
    def _digits_only(value):
        return re.sub(
            r"[^0-9]",
            "",
            str(value or "")
        )

    @classmethod
    def _last_four(cls, value):
        digits = cls._digits_only(value)

        if len(digits) < 4:
            return ""

        return digits[-4:]

    @staticmethod
    def _scaled_rois(frame_width, frame_height):
        scale_x = frame_width / PARKING_CAPTURE_WIDTH
        scale_y = frame_height / PARKING_CAPTURE_HEIGHT

        scaled = []

        for x1, y1, x2, y2 in PARKING_ROIS:
            scaled.append((
                int(x1 * scale_x),
                int(y1 * scale_y),
                int(x2 * scale_x),
                int(y2 * scale_y)
            ))

        return scaled

    def get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None

            return self.latest_frame.copy()

    def get_status_text(self):
        if self.last_error:
            return self.last_error

        if not self.models_ready:
            return "Loading YOLO/OCR models..."

        if not self.camera_ready:
            return "Opening /dev/video2..."

        return ""

    def stop(self):
        self.stop_event.set()

    def _load_existing_assignments(self):
        conn = None
        cursor = None

        try:
            conn = connect_db()
            cursor = conn.cursor()
            cursor.execute(
                """
                SELECT car_number, parking_location
                FROM car_management
                WHERE parking_location IS NOT NULL
                """
            )

            state_by_slot = {
                state["slot"]: state
                for state in self.slot_states
            }

            for car_number, location in cursor.fetchall():
                slot_name = str(location or "").strip().upper()
                state = state_by_slot.get(slot_name)

                if state is None:
                    continue

                state["confirmed_car_number"] = str(car_number)
                state["confirmed_last4"] = self._last_four(car_number)

        except mysql.connector.Error as error:
            print("[PARKING OCR DB INIT ERROR]", error)

        finally:
            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def _find_entered_vehicle(self, cursor, last_four):
        cursor.execute(
            """
            SELECT car_number, parking_location
            FROM car_management
            ORDER BY entry_time DESC, id DESC
            """
        )

        matches = []

        for car_number, location in cursor.fetchall():
            if self._last_four(car_number) == last_four:
                matches.append((
                    str(car_number),
                    location
                ))

        return matches

    def _get_entered_last_four(self):
        now = time.monotonic()

        if now - self.entered_last4_cache_time < 1.0:
            return self.entered_last4_cache

        conn = None
        cursor = None

        try:
            conn = connect_db()
            cursor = conn.cursor()
            cursor.execute(
                """
                SELECT car_number
                FROM car_management
                """
            )

            entered = set()

            for (car_number,) in cursor.fetchall():
                last_four = self._last_four(car_number)

                if last_four:
                    entered.add(last_four)

            self.entered_last4_cache = entered
            self.entered_last4_cache_time = now

        except mysql.connector.Error as error:
            print("[PARKING OCR DB CACHE ERROR]", error)

        finally:
            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

        return self.entered_last4_cache

    def _assign_slot(self, slot_name, last_four):
        conn = None
        cursor = None

        try:
            conn = connect_db()
            conn.start_transaction()
            cursor = conn.cursor()

            matches = self._find_entered_vehicle(
                cursor,
                last_four
            )

            if not matches:
                conn.rollback()
                print(
                    f"[PARKING OCR] {slot_name}: "
                    f"입차 목록에 *{last_four} 차량 없음"
                )
                return None

            if len(matches) > 1:
                conn.rollback()
                print(
                    f"[PARKING OCR] {slot_name}: "
                    f"뒷자리 *{last_four} 중복 - 자동 배정 보류"
                )
                return None

            car_number, old_location = matches[0]

            cursor.execute(
                """
                UPDATE car_management
                SET parking_location = NULL
                WHERE LOWER(TRIM(parking_location)) = %s
                  AND car_number <> %s
                """,
                (
                    slot_name.lower(),
                    car_number
                )
            )

            cursor.execute(
                """
                UPDATE car_management
                SET parking_location = %s
                WHERE car_number = %s
                """,
                (
                    slot_name.lower(),
                    car_number
                )
            )

            conn.commit()

            if str(old_location or "").strip().upper() != slot_name:
                print(
                    f"[PARKING OCR] {car_number} -> {slot_name} 배정"
                )

            return car_number

        except mysql.connector.Error as error:
            if conn is not None:
                conn.rollback()

            print("[PARKING OCR DB ASSIGN ERROR]", error)
            return None

        finally:
            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def _clear_slot(self, slot_name, expected_car_number):
        if not expected_car_number:
            return

        conn = None
        cursor = None

        try:
            conn = connect_db()
            cursor = conn.cursor()
            cursor.execute(
                """
                UPDATE car_management
                SET parking_location = NULL
                WHERE car_number = %s
                  AND LOWER(TRIM(parking_location)) = %s
                """,
                (
                    expected_car_number,
                    slot_name.lower()
                )
            )
            conn.commit()

            if cursor.rowcount > 0:
                print(
                    f"[PARKING OCR] {slot_name} 비움: "
                    f"{expected_car_number} 위치값 해제"
                )

        except mysql.connector.Error as error:
            if conn is not None:
                conn.rollback()

            print("[PARKING OCR DB CLEAR ERROR]", error)

        finally:
            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    @staticmethod
    def _open_camera():
        cap = cv2.VideoCapture(
            PARKING_CAMERA_DEVICE,
            cv2.CAP_V4L2
        )

        if not cap.isOpened():
            return cap

        cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )
        cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            PARKING_CAPTURE_WIDTH
        )
        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            PARKING_CAPTURE_HEIGHT
        )
        cap.set(
            cv2.CAP_PROP_FPS,
            PARKING_CAPTURE_FPS
        )
        cap.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1
        )

        return cap

    @staticmethod
    def _prepare_ocr_image(plate_image, threshold=False):
        if plate_image.size == 0:
            return None

        gray = cv2.cvtColor(
            plate_image,
            cv2.COLOR_BGR2GRAY
        )

        target_height = 96
        scale = max(
            target_height / max(gray.shape[0], 1),
            2.0
        )

        enlarged = cv2.resize(
            gray,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC
        )

        clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )
        enhanced = clahe.apply(enlarged)

        if threshold:
            enhanced = cv2.adaptiveThreshold(
                enhanced,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                7
            )

        return cv2.copyMakeBorder(
            enhanced,
            12,
            12,
            18,
            18,
            cv2.BORDER_CONSTANT,
            value=255
        )

    def _read_last_four(self, reader, plate_image):
        if plate_image.size == 0:
            return "", 0.0

        plate_width = plate_image.shape[1]
        right_start = int(plate_width * 0.36)
        last_digits_image = plate_image[:, right_start:]

        if last_digits_image.size == 0:
            return "", 0.0

        for use_threshold in (False, True):
            prepared = self._prepare_ocr_image(
                last_digits_image,
                threshold=use_threshold
            )

            if prepared is None:
                continue

            try:
                results = reader.readtext(
                    prepared,
                    detail=1,
                    paragraph=False,
                    decoder="greedy",
                    allowlist="0123456789",
                    min_size=5,
                    text_threshold=0.25,
                    low_text=0.15,
                    link_threshold=0.20,
                    canvas_size=1280,
                    mag_ratio=1.0
                )
            except Exception as error:
                print("[PARKING OCR READ ERROR]", error)
                return "", 0.0

            ordered = sorted(
                results,
                key=lambda item: min(
                    point[0]
                    for point in item[0]
                )
            )

            joined_digits = ""
            confidences = []

            for _, text, confidence in ordered:
                digits = self._digits_only(text)

                if not digits:
                    continue

                joined_digits += digits
                confidences.append(float(confidence))

            if len(joined_digits) >= 4:
                confidence = (
                    sum(confidences) / len(confidences)
                    if confidences
                    else 0.0
                )

                if confidence >= PARKING_OCR_CONFIDENCE:
                    return joined_digits[-4:], confidence

        return "", 0.0

    def _handle_plate_seen(self, roi_index, last_four):
        state = self.slot_states[roi_index]
        state["missing_hits"] = 0

        if not last_four:
            return

        if last_four not in self._get_entered_last_four():
            state["candidate_last4"] = ""
            state["candidate_hits"] = 0
            return

        if state["candidate_last4"] == last_four:
            state["candidate_hits"] += 1
        else:
            state["candidate_last4"] = last_four
            state["candidate_hits"] = 1

        if state["candidate_hits"] < PARKING_CONFIRM_HITS:
            return

        if state["confirmed_last4"] == last_four:
            return

        now = time.monotonic()

        if now - state["last_match_attempt"] < 2.0:
            return

        state["last_match_attempt"] = now

        car_number = self._assign_slot(
            state["slot"],
            last_four
        )

        if car_number is None:
            return

        state["confirmed_last4"] = last_four
        state["confirmed_car_number"] = car_number

    def _handle_plate_missing(self, roi_index):
        state = self.slot_states[roi_index]
        state["plate_box"] = None
        state["observed_last4"] = ""
        state["yolo_confidence"] = 0.0
        state["ocr_confidence"] = 0.0
        state["missing_hits"] += 1

        if state["missing_hits"] < PARKING_CLEAR_MISSES:
            return

        self._clear_slot(
            state["slot"],
            state["confirmed_car_number"]
        )

        state["candidate_last4"] = ""
        state["candidate_hits"] = 0
        state["missing_hits"] = 0
        state["confirmed_last4"] = ""
        state["confirmed_car_number"] = ""
        state["last_match_attempt"] = 0.0

    def _process_one_roi(
        self,
        frame,
        scaled_rois,
        model,
        reader,
        device,
        use_half
    ):
        roi_index = self.current_roi_index
        x1, y1, x2, y2 = scaled_rois[roi_index]
        roi_image = frame[y1:y2, x1:x2]

        if roi_image.size == 0:
            self._handle_plate_missing(roi_index)
            return

        try:
            with torch.inference_mode():
                prediction = model.predict(
                    source=roi_image,
                    conf=PARKING_YOLO_CONFIDENCE,
                    imgsz=640,
                    device=device,
                    half=use_half,
                    max_det=3,
                    verbose=False
                )[0]
        except Exception as error:
            print("[PARKING YOLO ERROR]", error)
            return

        boxes = prediction.boxes

        if boxes is None or len(boxes) == 0:
            self._handle_plate_missing(roi_index)
            return

        best_index = int(
            boxes.conf.argmax().item()
        )
        yolo_confidence = float(
            boxes.conf[best_index].item()
        )
        px1, py1, px2, py2 = (
            boxes.xyxy[best_index]
            .detach()
            .cpu()
            .tolist()
        )

        px1 = int(px1)
        py1 = int(py1)
        px2 = int(px2)
        py2 = int(py2)

        pad_x = max(
            int((px2 - px1) * 0.10),
            2
        )
        pad_y = max(
            int((py2 - py1) * 0.20),
            2
        )

        crop_x1 = max(px1 - pad_x, 0)
        crop_y1 = max(py1 - pad_y, 0)
        crop_x2 = min(px2 + pad_x, roi_image.shape[1])
        crop_y2 = min(py2 + pad_y, roi_image.shape[0])

        plate_image = roi_image[
            crop_y1:crop_y2,
            crop_x1:crop_x2
        ]

        self.slot_states[roi_index]["plate_box"] = (
            x1 + crop_x1,
            y1 + crop_y1,
            x1 + crop_x2,
            y1 + crop_y2
        )
        self.slot_states[roi_index]["yolo_confidence"] = (
            yolo_confidence
        )

        last_four, confidence = self._read_last_four(
            reader,
            plate_image
        )

        self.slot_states[roi_index]["observed_last4"] = (
            last_four
        )
        self.slot_states[roi_index]["ocr_confidence"] = (
            float(confidence)
        )

        self._handle_plate_seen(
            roi_index,
            last_four
        )

    def _draw_overlay(self, frame, scaled_rois):
        output = frame.copy()

        for index, (x1, y1, x2, y2) in enumerate(scaled_rois):
            state = self.slot_states[index]

            if state["confirmed_last4"]:
                color = (0, 0, 255)
                result_text = state["confirmed_last4"]
            elif state["candidate_last4"]:
                color = (0, 220, 255)
                result_text = state["candidate_last4"]
            elif state["observed_last4"]:
                color = (255, 180, 0)
                result_text = state["observed_last4"]
            else:
                color = (0, 255, 0)
                result_text = "----"

            if index == self.current_roi_index:
                color = (0, 255, 255)

            cv2.rectangle(
                output,
                (x1, y1),
                (x2, y2),
                color,
                2
            )
            cv2.putText(
                output,
                f"{state['slot']} {result_text}",
                (x1 + 5, y1 + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                color,
                2,
                cv2.LINE_AA
            )

            plate_box = state["plate_box"]

            if plate_box is not None:
                bx1, by1, bx2, by2 = plate_box
                cv2.rectangle(
                    output,
                    (bx1, by1),
                    (bx2, by2),
                    (255, 0, 255),
                    2
                )

        cv2.rectangle(
            output,
            (985, 18),
            (1270, 210),
            (255, 255, 255),
            2
        )
        cv2.rectangle(
            output,
            (987, 20),
            (1268, 208),
            (20, 20, 20),
            -1
        )
        cv2.putText(
            output,
            "PARKING OCR",
            (1000, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        for index, state in enumerate(self.slot_states):
            value = (
                state["confirmed_last4"]
                or state["candidate_last4"]
                or state["observed_last4"]
                or "----"
            )
            cv2.putText(
                output,
                (
                    f"{state['slot']}: {value} "
                    f"Y{state['yolo_confidence']:.2f} "
                    f"O{state['ocr_confidence']:.2f}"
                ),
                (1000, 75 + index * 21),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (0, 255, 255)
                if index == self.current_roi_index
                else (210, 210, 210),
                1,
                cv2.LINE_AA
            )

        return output

    def run(self):
        cap = None

        try:
            model_path = PARKING_MODEL_PATH

            if not os.path.isabs(model_path):
                model_path = os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    model_path
                )

            if not os.path.isfile(model_path):
                self.last_error = (
                    "Plate model not found: "
                    f"{model_path}"
                )
                print("[PARKING OCR]", self.last_error)
                return

            gpu_available = torch.cuda.is_available()
            device = 0 if gpu_available else "cpu"
            use_half = bool(gpu_available)

            print(
                "[PARKING OCR] 장치:",
                "CUDA GPU" if gpu_available else "CPU"
            )
            print("[PARKING OCR] YOLO 모델 로딩 중")
            model = YOLO(model_path)

            print("[PARKING OCR] EasyOCR 숫자 모델 로딩 중")
            reader = easyocr.Reader(
                ["en"],
                gpu=gpu_available,
                verbose=False
            )

            self.models_ready = True
            self._load_existing_assignments()

            cap = self._open_camera()

            if not cap.isOpened():
                self.last_error = (
                    f"Cannot open camera: {PARKING_CAMERA_DEVICE}"
                )
                print("[PARKING OCR]", self.last_error)
                return

            self.camera_ready = True
            print(
                f"[PARKING OCR] 카메라 연결: "
                f"{PARKING_CAMERA_DEVICE}"
            )

            while not self.stop_event.is_set():
                ret, frame = cap.read()

                if not ret:
                    self.last_error = "Camera frame read failed"
                    self.stop_event.wait(0.10)
                    continue

                self.last_error = ""
                frame_height, frame_width = frame.shape[:2]
                scaled_rois = self._scaled_rois(
                    frame_width,
                    frame_height
                )

                self._process_one_roi(
                    frame,
                    scaled_rois,
                    model,
                    reader,
                    device,
                    use_half
                )

                detection_frame = self._draw_overlay(
                    frame,
                    scaled_rois
                )

                with self.frame_lock:
                    self.latest_frame = detection_frame

                self.current_roi_index = (
                    self.current_roi_index + 1
                ) % len(PARKING_ROIS)

                self.stop_event.wait(
                    PARKING_SCAN_DELAY
                )

        except Exception as error:
            self.last_error = f"Parking OCR stopped: {error}"
            print("[PARKING OCR FATAL]", error)

        finally:
            self.camera_ready = False

            if cap is not None and cap.isOpened():
                cap.release()

            if torch.cuda.is_available():
                try:
                    torch.cuda.empty_cache()
                except Exception:
                    pass

            print("[PARKING OCR] 작업자 종료")


class ParkingDetectionWindow(QDialog):

    def __init__(self, worker):
        super().__init__()

        self.worker = worker

        self.setWindowTitle(
            "포켓3 실시간 주차면 번호판 검출"
        )
        self.resize(1280, 720)

        self.detection_view = QLabel(
            "포켓3 검출 모델을 준비하고 있습니다."
        )
        self.detection_view.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )
        self.detection_view.setMinimumSize(
            640,
            360
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.detection_view)

        self.detection_timer = QTimer(self)
        self.detection_timer.timeout.connect(
            self.update_detection_view
        )
        self.detection_timer.start(33)

    def update_detection_view(self):
        frame = self.worker.get_latest_frame()

        if frame is None:
            status = self.worker.get_status_text()

            self.detection_view.setText(
                status
                or "포켓3 검출 프레임을 기다리고 있습니다."
            )
            return

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )
        height, width, channels = rgb_frame.shape
        bytes_per_line = channels * width

        image = QImage(
            rgb_frame.data,
            width,
            height,
            bytes_per_line,
            QImage.Format.Format_RGB888
        ).copy()

        pixmap = QPixmap.fromImage(image)
        pixmap = pixmap.scaled(
            self.detection_view.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        self.detection_view.setPixmap(pixmap)

    def closeEvent(self, event):
        self.detection_timer.stop()
        event.accept()


def get_table_names(cursor):

    cursor.execute("SHOW TABLES")

    return [
        row[0]
        for row in cursor.fetchall()
    ]


def get_column_names(
    cursor,
    table_name
):

    cursor.execute(
        f"SHOW COLUMNS FROM `{table_name}`"
    )

    return [
        row[0]
        for row in cursor.fetchall()
    ]


def check_eco_vehicle(
    cursor,
    model
):

    tables = get_table_names(
        cursor
    )

    keywords = [
        "eco",
        "electric",
        "ev",
        "environment",
        "green",
        "전기",
        "친환경"
    ]

    model_columns = [
        "model",
        "model_name",
        "car_model",
        "vehicle_model"
    ]

    for table in tables:

        table_lower = table.lower()

        if not any(
            keyword in table_lower
            for keyword in keywords
        ):
            continue

        columns = get_column_names(
            cursor,
            table
        )

        for column in model_columns:

            if column not in columns:
                continue

            cursor.execute(
                f"""
                SELECT COUNT(*)
                FROM `{table}`
                WHERE REPLACE(
                    LOWER(`{column}`),
                    ' ',
                    ''
                )
                =
                REPLACE(
                    LOWER(%s),
                    ' ',
                    ''
                )
                """,
                (model,)
            )

            count = cursor.fetchone()[0]

            if count > 0:
                return "Y"

    return "N"


def check_disabled_vehicle(
    cursor,
    car_number
):

    tables = get_table_names(
        cursor
    )

    keywords = [
        "disabled",
        "disability",
        "handicap",
        "장애"
    ]

    car_columns = [
        "car_number",
        "vehicle_number",
        "plate_number"
    ]

    for table in tables:

        table_lower = table.lower()

        if not any(
            keyword in table_lower
            for keyword in keywords
        ):
            continue

        columns = get_column_names(
            cursor,
            table
        )

        for column in car_columns:

            if column not in columns:
                continue

            cursor.execute(
                f"""
                SELECT COUNT(*)
                FROM `{table}`
                WHERE REPLACE(
                    `{column}`,
                    ' ',
                    ''
                )
                =
                REPLACE(
                    %s,
                    ' ',
                    ''
                )
                """,
                (car_number,)
            )

            count = cursor.fetchone()[0]

            if count > 0:
                return "Y"

    return "N"


def candidate_list_to_flags(candidates):

    flags = []

    for candidate in candidates:

        if candidate is None:
            flags.append(False)
            continue

        normalized = (
            str(candidate)
            .strip()
            .lower()
        )

        flags.append(
            normalized
            not in (
                "",
                "null",
                "none"
            )
        )

    return flags


def get_db_slot_flags():

    parking_spaces = [
        "a1",
        "a2",
        "a3",
        "b1",
        "b2",
        "b3"
    ]

    conn = None
    cursor = None

    try:

        conn = connect_db()
        cursor = conn.cursor()

        cursor.execute(
            """
            SELECT parking_location
            FROM car_management
            WHERE parking_location IS NOT NULL
            """
        )

        occupied_spaces = set()

        for row in cursor.fetchall():

            parking_location = row[0]

            if parking_location is None:
                continue

            location = (
                str(parking_location)
                .strip()
                .lower()
            )

            if location in parking_spaces:
                occupied_spaces.add(
                    location
                )

        flags = [
            1
            if space in occupied_spaces
            else 0
            for space in parking_spaces
        ]

        print(
            "[DB SLOT FLAGS]",
            flags
        )

        return flags

    except mysql.connector.Error as e:

        print(
            "[DB SLOT ERROR]",
            e
        )

        return None

    finally:

        if cursor is not None:
            cursor.close()

        if conn is not None:
            conn.close()


def broadcast_db_slot_flags(esp32):

    flags = get_db_slot_flags()

    if flags is None:
        return

    esp32.broadcast_slot_flags(
        flags
    )


def record_entry(
    vehicle_number,
    vehicle_type,
    detected_at,
    event_id
):

    conn = None
    cursor = None

    try:

        conn = connect_db()
        cursor = conn.cursor()

        cursor.execute(
            """
            SELECT COUNT(*)
            FROM car_management
            WHERE car_number = %s
            """,
            (
                vehicle_number,
            )
        )

        car_count = (
            cursor.fetchone()[0]
        )

        if car_count > 0:

            print(
                "[DB ENTRY SKIP]",
                "이미 입차된 차량:",
                vehicle_number
            )

            return

        cursor.execute(
            """
            SELECT COUNT(*)
            FROM membership
            WHERE car_number = %s
              AND expiration_time >= CURDATE()
            """,
            (
                vehicle_number,
            )
        )

        member_count = (
            cursor.fetchone()[0]
        )

        if member_count > 0:
            member = "Y"
        else:
            member = "N"

        eco = check_eco_vehicle(
            cursor,
            vehicle_type
        )

        disabled = (
            check_disabled_vehicle(
                cursor,
                vehicle_number
            )
        )

        cursor.execute(
            """
            INSERT INTO car_management
            (
                car_number,
                entry_time,
                member,
                eco,
                disabled,
                model,
                parking_location
            )
            VALUES
            (
                %s,
                %s,
                %s,
                %s,
                %s,
                %s,
                NULL
            )
            """,
            (
                vehicle_number,
                detected_at,
                member,
                eco,
                disabled,
                vehicle_type
            )
        )

        conn.commit()

        print(
            "[DB ENTRY]",
            "event =", event_id,
            "/ car =", vehicle_number,
            "/ model =", vehicle_type,
            "/ time =", detected_at,
            "/ member =", member,
            "/ eco =", eco,
            "/ disabled =", disabled
        )

    except mysql.connector.Error as e:

        print(
            "[DB ENTRY ERROR]",
            e
        )

    finally:

        if cursor is not None:
            cursor.close()

        if conn is not None:
            conn.close()


def get_exit_fee(
    vehicle_number
):

    conn = None
    cursor = None

    try:

        conn = connect_db()
        cursor = conn.cursor()

        cursor.execute(
            """
            SELECT
                entry_time,
                member,
                eco,
                disabled
            FROM car_management
            WHERE car_number = %s
            """,
            (
                vehicle_number,
            )
        )

        vehicle_data = (
            cursor.fetchone()
        )

        if vehicle_data is None:

            raise ValueError(
                f"{vehicle_number} 차량의 입차 기록을 찾을 수 없습니다."
            )

        (
            entry_time,
            member,
            eco,
            disabled
        ) = vehicle_data

        if isinstance(
            entry_time,
            str
        ):

            entry_time = (
                datetime.strptime(
                    entry_time,
                    "%Y-%m-%d %H:%M:%S"
                )
            )

        exit_time = datetime.now()

        parked_seconds = max(
            0,
            math.ceil(
                (
                    exit_time
                    -
                    entry_time
                ).total_seconds()
            )
        )

        basic_fee = (
            parked_seconds
            *
            10
        )

        member = str(
            member
            or
            "N"
        ).strip().upper()

        eco = str(
            eco
            or
            "N"
        ).strip().upper()

        disabled = str(
            disabled
            or
            "N"
        ).strip().upper()

        if member == "Y":

            final_fee = 0

        elif (
            eco == "Y"
            or
            disabled == "Y"
        ):

            final_fee = (
                basic_fee
                //
                2
            )

        else:

            final_fee = basic_fee

        print(
            "[EXIT FEE]",
            "car =", vehicle_number,
            "/ seconds =", parked_seconds,
            "/ fee =", final_fee
        )

        with pending_exit_lock:

            if (
                vehicle_number
                not in
                pending_exit_vehicles
            ):

                pending_exit_vehicles.append(
                    vehicle_number
                )

        return final_fee

    finally:

        if cursor is not None:
            cursor.close()

        if conn is not None:
            conn.close()


def delete_confirmed_exit_vehicle():

    with pending_exit_lock:

        if not pending_exit_vehicles:

            print(
                "[DB EXIT SKIP]",
                "삭제할 출차 대기 차량이 없습니다."
            )

            return

        vehicle_number = (
            pending_exit_vehicles[0]
        )

    conn = None
    cursor = None

    try:

        conn = connect_db()
        cursor = conn.cursor()

        cursor.execute(
            """
            DELETE FROM car_management
            WHERE car_number = %s
            """,
            (
                vehicle_number,
            )
        )

        conn.commit()

        with pending_exit_lock:

            if (
                pending_exit_vehicles
                and
                pending_exit_vehicles[0]
                ==
                vehicle_number
            ):

                pending_exit_vehicles.pop(0)

        print(
            "[DB EXIT DELETE]",
            vehicle_number
        )

    except mysql.connector.Error as e:

        print(
            "[DB EXIT DELETE ERROR]",
            e
        )

    finally:

        if cursor is not None:
            cursor.close()

        if conn is not None:
            conn.close()


class ESP32ServerWithExitDelete(
    ESP32Server
):

    def _process_exit_confirmed(
        self,
        client,
        request_id,
        payload
    ):

        super()._process_exit_confirmed(
            client,
            request_id,
            payload
        )

        delete_confirmed_exit_vehicle()


def start_network_servers():

    entry_camera = CameraReceiver(
        port=5001
    )

    exit_camera = CameraReceiver(
        port=5002
    )

    esp32 = ESP32ServerWithExitDelete(
        entry_camera_receiver=entry_camera,
        exit_camera_receiver=exit_camera,
        ai_requester=None,
        fee_provider=get_exit_fee,
        entry_recorder=record_entry
    )

    ai = AIServer(
        timeout=30,
        on_candidate_list=
        lambda _candidates: None
    )

    esp32.ai_requester = (
        ai.request_ocr
    )

    entry_camera.start()
    exit_camera.start()
    ai.start()
    esp32.start()

    print(
        "[MAIN] entry camera UDP : 5001"
    )

    print(
        "[MAIN] exit camera UDP  : 5002"
    )

    print(
        "[MAIN] AI server TCP    : 6000"
    )

    print(
        "[MAIN] ESP32 TCP        : 5000"
    )

    print(
        "[MAIN] network servers started"
    )

    return (
        entry_camera,
        exit_camera,
        ai,
        esp32
    )


class Admin(QDialog):

    def __init__(self):
        super().__init__()

        uic.loadUi(
            "main_admin.ui",
            self
        )

        self.current_vehicle_columns = []
        self.current_member_columns = []

        self.btnVehicle.clicked.connect(
            self.show_vehicle_page
        )
        self.btnMember.clicked.connect(
            self.show_member_page
        )
        self.btnCamera.clicked.connect(
            self.show_camera_page
        )

        self.btnVehicleSearch.clicked.connect(
            self.search_vehicle
        )
        self.btnVehicleReset.clicked.connect(
            self.reset_vehicle_search
        )
        self.editVehicleSearch.returnPressed.connect(
            self.search_vehicle
        )
        self.btnVehicleAdd.clicked.connect(
            self.toggle_vehicle_input
        )
        self.btnVehicleExit.clicked.connect(
            self.delete_vehicle
        )
        self.btnEntrySave.clicked.connect(
            self.add_vehicle
        )
        self.btnEntryCancel.clicked.connect(
            self.cancel_vehicle_input
        )

        self.vehicleInputFrame.setVisible(
            False
        )

        self.tblVehicles.verticalHeader().setVisible(
            False
        )
        self.tblVehicles.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.tblVehicles.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        self.tblVehicles.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.tblVehicles.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )

        self.btnMemberSearch.clicked.connect(
            self.search_member
        )
        self.btnMemberReset.clicked.connect(
            self.reset_member_search
        )
        self.editMemberSearch.returnPressed.connect(
            self.search_member
        )
        self.btnMemberAdd.clicked.connect(
            self.toggle_member_input
        )
        self.btnMemberDelete.clicked.connect(
            self.delete_member
        )
        self.btnMemberSave.clicked.connect(
            self.add_member
        )
        self.btnMemberCancel.clicked.connect(
            self.cancel_member_input
        )

        self.memberInputFrame.setVisible(
            False
        )

        self.tblMembers.verticalHeader().setVisible(
            False
        )
        self.tblMembers.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.tblMembers.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        self.tblMembers.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )
        self.tblMembers.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )

        # =====================================
        # 카메라 제어
        # =====================================
        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.pan_angle = SERVO_INITIAL_PAN
        self.tilt_angle = SERVO_INITIAL_TILT
        self.current_command = None

        self.servo_timer = QTimer(
            self
        )

        self.servo_timer.timeout.connect(
            self.send_servo_command
        )

        self.servo_timer.setInterval(
            SERVO_INTERVAL_MS
        )

        self.btnCameraLeft.pressed.connect(
            self.start_left
        )
        self.btnCameraLeft.released.connect(
            self.stop_servo_command
        )

        self.btnCameraRight.pressed.connect(
            self.start_right
        )
        self.btnCameraRight.released.connect(
            self.stop_servo_command
        )

        self.btnCameraUp.pressed.connect(
            self.start_up
        )
        self.btnCameraUp.released.connect(
            self.stop_servo_command
        )

        self.btnCameraDown.pressed.connect(
            self.start_down
        )
        self.btnCameraDown.released.connect(
            self.stop_servo_command
        )

        self.send_servo_angles()

        self.esp32_camera_receiver = (
            ESP32CamUDPReceiver()
        )

        self.esp32_camera_receiver.start()

        self.camera_timer = QTimer(
            self
        )

        self.camera_timer.timeout.connect(
            self.update_camera
        )

        self.refresh_timer = QTimer(
            self
        )

        self.refresh_timer.timeout.connect(
            self.refresh_vehicle_page
        )

        self.refresh_timer.start(
            1000
        )

        self.mainStackedWidget.setCurrentWidget(
            self.pageVehicle
        )

        self.load_all_vehicles()
        self.load_all_members()
        self.load_parking_status()

    def show_vehicle_page(self):

        self.stop_servo_command()
        self.stop_camera()

        self.mainStackedWidget.setCurrentWidget(
            self.pageVehicle
        )

        self.load_all_vehicles()
        self.load_parking_status()

    def show_member_page(self):

        self.stop_servo_command()
        self.stop_camera()

        self.mainStackedWidget.setCurrentWidget(
            self.pageMember
        )

        self.load_all_members()

    def show_camera_page(self):

        self.mainStackedWidget.setCurrentWidget(
            self.pageCamera
        )

        self.start_camera()

    def refresh_vehicle_page(self):

        if (
            self.mainStackedWidget.currentWidget()
            ==
            self.pageVehicle
        ):

            if self.editVehicleSearch.text().strip() == "":
                self.load_all_vehicles()

            self.load_parking_status()

    def load_all_vehicles(self):

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT *
                FROM car_management
                ORDER BY id
                """
            )

            rows = cursor.fetchall()

            self.show_table_data(
                self.tblVehicles,
                cursor,
                rows,
                "vehicle"
            )

        except mysql.connector.Error as e:

            print(
                "[VEHICLE DB ERROR]",
                e
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def search_vehicle(self):

        keyword = (
            self.editVehicleSearch
            .text()
            .strip()
        )

        if keyword == "":
            self.load_all_vehicles()
            return

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT *
                FROM car_management
                WHERE car_number LIKE %s
                ORDER BY id
                """,
                (
                    f"%{keyword}%",
                )
            )

            rows = cursor.fetchall()

            self.show_table_data(
                self.tblVehicles,
                cursor,
                rows,
                "vehicle"
            )

        except mysql.connector.Error as e:

            QMessageBox.critical(
                self,
                "DB 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def reset_vehicle_search(self):

        self.editVehicleSearch.clear()
        self.load_all_vehicles()

    def toggle_vehicle_input(self):

        is_visible = self.vehicleInputFrame.isVisible()

        self.vehicleInputFrame.setVisible(
            not is_visible
        )

        if not is_visible:
            self.editEntryCarNumber.setFocus()

    def cancel_vehicle_input(self):

        self.editEntryCarNumber.clear()
        self.editEntryModel.clear()

        self.comboEntryLocation.setCurrentIndex(
            0
        )

        self.vehicleInputFrame.setVisible(
            False
        )

    def add_vehicle(self):

        car_number = (
            self.editEntryCarNumber
            .text()
            .strip()
        )

        model = (
            self.editEntryModel
            .text()
            .strip()
        )

        if self.comboEntryLocation.currentIndex() == 0:
            location = ""
        else:
            location = (
                self.comboEntryLocation
                .currentText()
                .strip()
                .lower()
            )

        if car_number == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "차량번호를 입력해주세요."
            )
            return

        if model == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "차종을 입력해주세요."
            )
            return

        if location == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "주차위치를 선택해주세요."
            )
            return

        parking_spaces = [
            "a1",
            "a2",
            "a3",
            "b1",
            "b2",
            "b3"
        ]

        if location not in parking_spaces:

            QMessageBox.warning(
                self,
                "주차위치 오류",
                "주차위치는 A1, A2, A3, "
                "B1, B2, B3 중 하나를 선택해주세요."
            )
            return

        entry_time = datetime.now().strftime(
            "%Y-%m-%d %H:%M:%S"
        )

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT COUNT(*)
                FROM car_management
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            car_count = cursor.fetchone()[0]

            if car_count > 0:

                QMessageBox.warning(
                    self,
                    "차량 추가",
                    "이미 입차되어 있는 차량입니다."
                )
                return

            cursor.execute(
                """
                SELECT car_number
                FROM car_management
                WHERE LOWER(parking_location) = %s
                """,
                (
                    location,
                )
            )

            occupied = cursor.fetchone()

            if occupied is not None:

                QMessageBox.warning(
                    self,
                    "주차위치 오류",
                    f"{location.upper()} 위치에는 "
                    f"이미 {occupied[0]} 차량이 있습니다."
                )
                return

            cursor.execute(
                """
                SELECT COUNT(*)
                FROM membership
                WHERE car_number = %s
                  AND expiration_time >= CURDATE()
                """,
                (
                    car_number,
                )
            )

            member_count = cursor.fetchone()[0]

            if member_count > 0:
                member = "Y"
            else:
                member = "N"

            eco = check_eco_vehicle(
                cursor,
                model
            )

            disabled = check_disabled_vehicle(
                cursor,
                car_number
            )

            cursor.execute(
                """
                INSERT INTO car_management
                (
                    car_number,
                    entry_time,
                    member,
                    eco,
                    disabled,
                    model,
                    parking_location
                )
                VALUES
                (
                    %s,
                    %s,
                    %s,
                    %s,
                    %s,
                    %s,
                    %s
                )
                """,
                (
                    car_number,
                    entry_time,
                    member,
                    eco,
                    disabled,
                    model,
                    location
                )
            )

            conn.commit()

            QMessageBox.information(
                self,
                "차량 추가",
                "차량이 추가되었습니다.\n\n"
                f"차량번호 : {car_number}\n"
                f"차종 : {model}\n"
                f"주차위치 : {location.upper()}\n"
                f"입차시간 : {entry_time}\n"
                f"회원 : {member}\n"
                f"친환경 : {eco}\n"
                f"장애인 : {disabled}"
            )

            self.cancel_vehicle_input()
            self.editVehicleSearch.clear()
            self.load_all_vehicles()
            self.load_parking_status()

        except mysql.connector.Error as e:

            QMessageBox.critical(
                self,
                "DB 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def delete_vehicle(self):

        selected_row = self.tblVehicles.currentRow()

        if selected_row < 0:

            QMessageBox.warning(
                self,
                "차량 출차",
                "출차할 차량을 선택해주세요."
            )
            return

        if "car_number" not in self.current_vehicle_columns:

            QMessageBox.warning(
                self,
                "출차 오류",
                "car_number 컬럼을 찾을 수 없습니다."
            )
            return

        car_number_column = (
            self.current_vehicle_columns.index(
                "car_number"
            )
        )

        item = self.tblVehicles.item(
            selected_row,
            car_number_column
        )

        if item is None:
            return

        car_number = item.text()

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT
                    entry_time,
                    member,
                    eco,
                    disabled
                FROM car_management
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            vehicle_data = cursor.fetchone()

            if vehicle_data is None:

                QMessageBox.warning(
                    self,
                    "출차 오류",
                    "해당 차량의 입차 기록을 찾을 수 없습니다."
                )

                self.load_all_vehicles()
                return

            (
                entry_time,
                member,
                eco,
                disabled
            ) = vehicle_data

            if isinstance(entry_time, str):

                entry_time = datetime.strptime(
                    entry_time,
                    "%Y-%m-%d %H:%M:%S"
                )

            exit_time = datetime.now()

            parked_seconds = max(
                0,
                math.ceil(
                    (
                        exit_time
                        -
                        entry_time
                    ).total_seconds()
                )
            )

            basic_fee = parked_seconds * 10

            member = str(member or "N").strip().upper()
            eco = str(eco or "N").strip().upper()
            disabled = str(disabled or "N").strip().upper()

            if member == "Y":

                discount_text = "회원 차량 전액 면제"
                final_fee = 0

            elif eco == "Y" or disabled == "Y":

                discount_text = "친환경/장애인 차량 50% 할인"
                final_fee = basic_fee // 2

            else:

                discount_text = "할인 없음"
                final_fee = basic_fee

            answer = QMessageBox.question(
                self,
                "출차 요금 정산",
                f"차량번호 : {car_number}\n"
                f"입차시간 : "
                f"{entry_time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"출차시간 : "
                f"{exit_time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"주차시간 : {parked_seconds}초\n"
                f"기본요금 : {basic_fee:,}원\n"
                f"할인내용 : {discount_text}\n"
                f"최종요금 : {final_fee:,}원\n\n"
                "정산 후 차량을 출차 처리하시겠습니까?",
                QMessageBox.StandardButton.Yes
                |
                QMessageBox.StandardButton.No
            )

            if answer != QMessageBox.StandardButton.Yes:
                return

            cursor.execute(
                """
                DELETE FROM car_management
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            conn.commit()

            QMessageBox.information(
                self,
                "출차 완료",
                f"{car_number} 차량의 출차 처리가 완료되었습니다.\n"
                f"최종요금 : {final_fee:,}원"
            )

            self.editVehicleSearch.clear()
            self.load_all_vehicles()
            self.load_parking_status()

        except (
            mysql.connector.Error,
            ValueError,
            TypeError
        ) as e:

            QMessageBox.critical(
                self,
                "출차 처리 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def load_all_members(self):

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT *
                FROM membership
                """
            )

            rows = cursor.fetchall()

            self.show_table_data(
                self.tblMembers,
                cursor,
                rows,
                "member"
            )

        except mysql.connector.Error as e:

            print(
                "[MEMBER DB ERROR]",
                e
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def search_member(self):

        keyword = (
            self.editMemberSearch
            .text()
            .strip()
        )

        if keyword == "":
            self.load_all_members()
            return

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            search_word = f"%{keyword}%"

            cursor.execute(
                """
                SELECT *
                FROM membership
                WHERE car_number LIKE %s
                   OR name LIKE %s
                """,
                (
                    search_word,
                    search_word
                )
            )

            rows = cursor.fetchall()

            self.show_table_data(
                self.tblMembers,
                cursor,
                rows,
                "member"
            )

        except mysql.connector.Error as e:

            QMessageBox.critical(
                self,
                "DB 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def reset_member_search(self):

        self.editMemberSearch.clear()
        self.load_all_members()

    def toggle_member_input(self):

        is_visible = self.memberInputFrame.isVisible()

        self.memberInputFrame.setVisible(
            not is_visible
        )

        if not is_visible:
            self.editMemberName.setFocus()

    def cancel_member_input(self):

        self.editMemberName.clear()
        self.editMemberCarNumber.clear()
        self.editMemberModel.clear()

        self.memberInputFrame.setVisible(
            False
        )

    def add_member(self):

        name = (
            self.editMemberName
            .text()
            .strip()
        )

        car_number = (
            self.editMemberCarNumber
            .text()
            .strip()
        )

        model = (
            self.editMemberModel
            .text()
            .strip()
        )

        expiration_time = (
            self.dateMemberExpiration
            .date()
            .toString(
                "yyyy-MM-dd"
            )
        )

        if name == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "이름을 입력해주세요."
            )
            return

        if car_number == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "차량번호를 입력해주세요."
            )
            return

        if model == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "차종을 입력해주세요."
            )
            return

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT COUNT(*)
                FROM membership
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            count = cursor.fetchone()[0]

            if count > 0:

                QMessageBox.warning(
                    self,
                    "회원 추가",
                    "이미 등록된 차량번호입니다."
                )
                return

            cursor.execute(
                """
                INSERT INTO membership
                (
                    name,
                    car_number,
                    model,
                    expiration_time
                )
                VALUES
                (
                    %s,
                    %s,
                    %s,
                    %s
                )
                """,
                (
                    name,
                    car_number,
                    model,
                    expiration_time
                )
            )

            conn.commit()

            QMessageBox.information(
                self,
                "회원 추가",
                "회원이 추가되었습니다."
            )

            self.cancel_member_input()
            self.editMemberSearch.clear()
            self.load_all_members()

        except mysql.connector.Error as e:

            QMessageBox.critical(
                self,
                "DB 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def delete_member(self):

        selected_row = self.tblMembers.currentRow()

        if selected_row < 0:

            QMessageBox.warning(
                self,
                "회원 삭제",
                "삭제할 회원을 선택해주세요."
            )
            return

        if "car_number" not in self.current_member_columns:

            QMessageBox.warning(
                self,
                "회원 삭제",
                "car_number 컬럼을 찾을 수 없습니다."
            )
            return

        car_number_column = (
            self.current_member_columns.index(
                "car_number"
            )
        )

        item = self.tblMembers.item(
            selected_row,
            car_number_column
        )

        if item is None:
            return

        car_number = item.text()

        answer = QMessageBox.question(
            self,
            "회원 삭제",
            f"{car_number} 회원을 삭제하시겠습니까?",
            QMessageBox.StandardButton.Yes
            |
            QMessageBox.StandardButton.No
        )

        if answer != QMessageBox.StandardButton.Yes:
            return

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                DELETE FROM membership
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            conn.commit()

            QMessageBox.information(
                self,
                "회원 삭제",
                "회원이 삭제되었습니다."
            )

            self.editMemberSearch.clear()
            self.load_all_members()

        except mysql.connector.Error as e:

            QMessageBox.critical(
                self,
                "DB 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def show_table_data(
        self,
        table,
        cursor,
        rows,
        table_type
    ):

        column_names = [
            column[0]
            for column in cursor.description
        ]

        if table_type == "vehicle":
            self.current_vehicle_columns = column_names
        else:
            self.current_member_columns = column_names

        table.setColumnCount(
            len(column_names)
        )

        table.setRowCount(
            len(rows)
        )

        header_names = {
            "id": "번호",
            "car_number": "차량번호",
            "entry_time": "입차시간",
            "member": "회원",
            "eco": "친환경",
            "disabled": "장애인",
            "model": "차종",
            "parking_location": "위치",
            "name": "회원이름",
            "expiration_time": "만료일"
        }

        korean_headers = [
            header_names.get(
                column,
                column
            )
            for column in column_names
        ]

        table.setHorizontalHeaderLabels(
            korean_headers
        )

        for row_index, row in enumerate(rows):

            for column_index, value in enumerate(row):

                text = "" if value is None else str(value)

                item = QTableWidgetItem(
                    text
                )

                item.setTextAlignment(
                    Qt.AlignmentFlag.AlignCenter
                )

                item.setToolTip(
                    text
                )

                table.setItem(
                    row_index,
                    column_index,
                    item
                )

        header = table.horizontalHeader()

        header.setSectionResizeMode(
            QHeaderView.ResizeMode.Interactive
        )

        if table_type == "vehicle":

            vehicle_widths = {
                "id": 42,
                "car_number": 95,
                "entry_time": 145,
                "member": 52,
                "eco": 58,
                "disabled": 58,
                "model": 80,
                "parking_location": 58
            }

            for index, column in enumerate(column_names):
                table.setColumnWidth(
                    index,
                    vehicle_widths.get(column, 80)
                )

        else:

            member_widths = {
                "id": 55,
                "name": 140,
                "car_number": 170,
                "model": 170,
                "expiration_time": 140
            }

            for index, column in enumerate(column_names):
                table.setColumnWidth(
                    index,
                    member_widths.get(column, 120)
                )

            header.setStretchLastSection(
                True
            )

    # =========================================
    # 카메라 / 서보 제어
    # =========================================
    def start_left(self):

        self.current_command = "LEFT"
        self.send_servo_command()
        self.servo_timer.start()

    def start_right(self):

        self.current_command = "RIGHT"
        self.send_servo_command()
        self.servo_timer.start()

    def start_up(self):

        self.current_command = "UP"
        self.send_servo_command()
        self.servo_timer.start()

    def start_down(self):

        self.current_command = "DOWN"
        self.send_servo_command()
        self.servo_timer.start()

    def stop_servo_command(self):

        self.servo_timer.stop()
        self.current_command = None

    def send_servo_angles(self):

        self.pan_angle = max(
            SERVO_MIN_ANGLE,
            min(
                SERVO_MAX_ANGLE,
                int(self.pan_angle)
            )
        )

        self.tilt_angle = max(
            SERVO_MIN_ANGLE,
            min(
                SERVO_MAX_ANGLE,
                int(self.tilt_angle)
            )
        )

        message = (
            f"{self.pan_angle},{self.tilt_angle}"
        )

        try:

            self.udp_socket.sendto(
                message.encode(),
                (
                    ESP32_IP,
                    ESP32_PORT
                )
            )

            print(
                "UDP SERVO SEND:",
                message
            )

        except Exception as e:

            print(
                "UDP SERVO ERROR:",
                e
            )

    def send_servo_command(self):

        if self.current_command is None:
            return

        old_pan = self.pan_angle
        old_tilt = self.tilt_angle

        if self.current_command == "LEFT":
            self.pan_angle -= SERVO_STEP

        elif self.current_command == "RIGHT":
            self.pan_angle += SERVO_STEP

        elif self.current_command == "UP":
            self.tilt_angle -= SERVO_STEP

        elif self.current_command == "DOWN":
            self.tilt_angle += SERVO_STEP

        self.pan_angle = max(
            SERVO_MIN_ANGLE,
            min(
                SERVO_MAX_ANGLE,
                self.pan_angle
            )
        )

        self.tilt_angle = max(
            SERVO_MIN_ANGLE,
            min(
                SERVO_MAX_ANGLE,
                self.tilt_angle
            )
        )

        if (
            self.pan_angle == old_pan
            and
            self.tilt_angle == old_tilt
        ):
            return

        self.send_servo_angles()

    def start_camera(self):

        if not self.camera_timer.isActive():
            self.camera_timer.start(
                33
            )

        frame = (
            self.esp32_camera_receiver
            .get_latest_frame()
        )

        if frame is None:
            self.lblCameraView.clear()
            self.lblCameraView.setText(
                self.esp32_camera_receiver
                .get_status_text()
            )
            self.lblCameraView.setAlignment(
                Qt.AlignmentFlag.AlignCenter
            )

    def update_camera(self):

        frame = (
            self.esp32_camera_receiver
            .get_latest_frame()
        )

        if frame is None:

            status = (
                self.esp32_camera_receiver
                .get_status_text()
            )

            if status:
                self.lblCameraView.clear()
                self.lblCameraView.setText(
                    status
                )
                self.lblCameraView.setAlignment(
                    Qt.AlignmentFlag.AlignCenter
                )

            return

        rgb_frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        height, width, channel = (
            rgb_frame.shape
        )

        bytes_per_line = (
            channel * width
        )

        image = QImage(
            rgb_frame.data,
            width,
            height,
            bytes_per_line,
            QImage.Format.Format_RGB888
        ).copy()

        pixmap = QPixmap.fromImage(
            image
        )

        pixmap = pixmap.scaled(
            self.lblCameraView.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        self.lblCameraView.setPixmap(
            pixmap
        )

        self.lblCameraView.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

    def stop_camera(self):

        if self.camera_timer.isActive():
            self.camera_timer.stop()

        self.lblCameraView.clear()

        self.lblCameraView.setText(
            "카메라 화면"
        )

        self.lblCameraView.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

    def load_parking_status(self):

        parking_status = {
            "a1": None,
            "a2": None,
            "a3": None,
            "b1": None,
            "b2": None,
            "b3": None
        }

        parking_disabled = {
            "a1": None,
            "a2": None,
            "a3": None,
            "b1": None,
            "b2": None,
            "b3": None
        }

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT
                    car_number,
                    parking_location,
                    disabled
                FROM car_management
                WHERE parking_location IS NOT NULL
                """
            )

            rows = cursor.fetchall()

            for car_number, parking_location, disabled in rows:

                if parking_location is None:
                    continue

                location = parking_location.strip().lower()

                if location in parking_status:
                    parking_status[location] = car_number
                    parking_disabled[location] = disabled

        except mysql.connector.Error as e:

            print(
                "[PARKING DB ERROR]",
                e
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

        slot_data = (
            (self.slotP1, "A1", parking_status["a1"], parking_disabled["a1"]),
            (self.slotP2, "A2", parking_status["a2"], parking_disabled["a2"]),
            (self.slotP3, "A3", parking_status["a3"], parking_disabled["a3"]),
            (self.slotP4, "B1", parking_status["b1"], parking_disabled["b1"]),
            (self.slotP5, "B2", parking_status["b2"], parking_disabled["b2"]),
            (self.slotP6, "B3", parking_status["b3"], parking_disabled["b3"])
        )

        for button, location, car_number, disabled in slot_data:

            self.set_parking_space(
                button,
                location,
                car_number,
                disabled
            )

    def set_parking_space(
        self,
        button,
        location,
        car_number,
        disabled=None
    ):

        if car_number is not None:

            button.setText(
                f"{location}\n{car_number}"
            )

            is_disabled_violation = (
                location == "A1"
                and str(disabled or "N").strip().upper() != "Y"
            )

            if is_disabled_violation:

                button.setStyleSheet(
                    """
                    QPushButton {
                        background-color: #F1C40F;
                        color: #1F2937;
                        border: 2px solid #B7950B;
                        border-radius: 12px;
                        padding: 14px;
                        font-size: 16px;
                        font-weight: bold;
                    }
                    """
                )

                button.setToolTip(
                    "장애인 구역에 장애인 미등록 차량이 주차되어 있습니다."
                )

            else:

                button.setStyleSheet(
                    """
                    QPushButton {
                        background-color: #E74C3C;
                        color: white;
                        border: 2px solid #922B21;
                        border-radius: 12px;
                        padding: 14px;
                        font-size: 16px;
                        font-weight: bold;
                    }
                    """
                )

                button.setToolTip("")

        else:

            button.setText(
                f"{location}\n빈자리"
            )

            button.setStyleSheet(
                """
                QPushButton {
                    background-color: #2563EB;
                    color: white;
                    border: none;
                    border-radius: 12px;
                    padding: 14px;
                    font-size: 16px;
                    font-weight: bold;
                }
                """
            )

            button.setToolTip("")

    def closeEvent(
        self,
        event
    ):

        self.refresh_timer.stop()
        self.stop_servo_command()
        self.stop_camera()

        if hasattr(
            self,
            "esp32_camera_receiver"
        ):
            self.esp32_camera_receiver.stop()
            self.esp32_camera_receiver.join(
                timeout=2.0
            )

        try:
            self.udp_socket.close()
        except:
            pass

        event.accept()


class Membership(QDialog):

    def __init__(
        self,
        admin_window
    ):

        super().__init__()

        self.admin_window = admin_window

        uic.loadUi(
            "membership.ui",
            self
        )

        self.search_button.clicked.connect(
            self.search_member
        )

        self.add_button.clicked.connect(
            self.add_member
        )

        self.delete_button.clicked.connect(
            self.delete_member
        )

        self.home_button.clicked.connect(
            self.go_home
        )

        self.search.returnPressed.connect(
            self.search_member
        )

        self.result.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )

        self.result.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )

        self.result.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )

        self.result.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )

        self.load_all_members()

    def go_home(self):

        self.admin_window.show()
        self.close()

    def load_all_members(self):

        conn = connect_db()
        cursor = conn.cursor()

        cursor.execute(
            """
            SELECT *
            FROM membership
            """
        )

        rows = cursor.fetchall()

        self.show_member_data(
            cursor,
            rows
        )

        cursor.close()
        conn.close()

    def search_member(self):

        keyword = (
            self.search
            .text()
            .strip()
        )

        if keyword == "":

            self.load_all_members()
            return

        conn = connect_db()
        cursor = conn.cursor()

        search_word = (
            f"%{keyword}%"
        )

        cursor.execute(
            """
            SELECT *
            FROM membership
            WHERE car_number LIKE %s
               OR name LIKE %s
            """,
            (
                search_word,
                search_word
            )
        )

        rows = cursor.fetchall()

        self.show_member_data(
            cursor,
            rows
        )

        cursor.close()
        conn.close()

    def add_member(self):

        name = (
            self.name
            .text()
            .strip()
        )

        car_number = (
            self.car_number
            .text()
            .strip()
        )

        model = (
            self.model
            .text()
            .strip()
        )

        expiration_time = (
            self.ex_date
            .date()
            .toString(
                "yyyy-MM-dd"
            )
        )

        if name == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "이름을 입력해주세요."
            )

            return

        if car_number == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "차량번호를 입력해주세요."
            )

            return

        if model == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "모델명을 입력해주세요."
            )

            return

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT COUNT(*)
                FROM membership
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            count = (
                cursor.fetchone()[0]
            )

            if count > 0:

                QMessageBox.warning(
                    self,
                    "회원 추가",
                    "이미 등록된 차량번호입니다."
                )

                return

            cursor.execute(
                """
                INSERT INTO membership
                (
                    name,
                    car_number,
                    model,
                    expiration_time
                )
                VALUES
                (
                    %s,
                    %s,
                    %s,
                    %s
                )
                """,
                (
                    name,
                    car_number,
                    model,
                    expiration_time
                )
            )

            conn.commit()

            QMessageBox.information(
                self,
                "회원 추가",
                "회원이 추가되었습니다."
            )

            self.name.clear()
            self.car_number.clear()
            self.model.clear()
            self.search.clear()

            self.load_all_members()

        except mysql.connector.Error as e:

            QMessageBox.critical(
                self,
                "DB 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def delete_member(self):

        selected_row = (
            self.result.currentRow()
        )

        if selected_row < 0:

            QMessageBox.warning(
                self,
                "회원 삭제",
                "삭제할 회원을 선택해주세요."
            )

            return

        car_number_column = -1

        for column in range(
            self.result.columnCount()
        ):

            header = (
                self.result
                .horizontalHeaderItem(
                    column
                )
            )

            if (
                header is not None
                and
                header.text()
                ==
                "car_number"
            ):

                car_number_column = (
                    column
                )

                break

        if car_number_column == -1:
            return

        item = self.result.item(
            selected_row,
            car_number_column
        )

        if item is None:
            return

        car_number = (
            item.text()
        )

        answer = QMessageBox.question(
            self,
            "회원 삭제",
            f"{car_number} 회원을 삭제하시겠습니까?",
            QMessageBox.StandardButton.Yes
            |
            QMessageBox.StandardButton.No
        )

        if (
            answer
            !=
            QMessageBox.StandardButton.Yes
        ):
            return

        conn = connect_db()
        cursor = conn.cursor()

        cursor.execute(
            """
            DELETE FROM membership
            WHERE car_number = %s
            """,
            (
                car_number,
            )
        )

        conn.commit()

        cursor.close()
        conn.close()

        QMessageBox.information(
            self,
            "회원 삭제",
            "회원이 삭제되었습니다."
        )

        self.search.clear()

        self.load_all_members()

    def show_member_data(
        self,
        cursor,
        rows
    ):

        column_names = [
            column[0]
            for column
            in cursor.description
        ]

        self.result.setColumnCount(
            len(column_names)
        )

        self.result.setRowCount(
            len(rows)
        )

        self.result.setHorizontalHeaderLabels(
            column_names
        )

        for row_index, row in enumerate(
            rows
        ):

            for (
                column_index,
                value
            ) in enumerate(
                row
            ):

                text = (
                    ""
                    if value is None
                    else str(value)
                )

                item = QTableWidgetItem(
                    text
                )

                item.setTextAlignment(
                    Qt.AlignmentFlag.AlignCenter
                )

                self.result.setItem(
                    row_index,
                    column_index,
                    item
                )


class CarSearch(QDialog):

    def __init__(
        self,
        admin_window
    ):

        super().__init__()

        self.admin_window = admin_window

        uic.loadUi(
            "car_management.ui",
            self
        )

        self.current_columns = []

        self.search_button.clicked.connect(
            self.search_vehicle
        )

        self.delete_button.clicked.connect(
            self.delete_vehicle
        )

        self.delete_button_2.clicked.connect(
            self.add_vehicle
        )

        self.home_button.clicked.connect(
            self.go_home
        )

        self.search.returnPressed.connect(
            self.search_vehicle
        )

        self.result.verticalHeader().setVisible(
            False
        )

        self.result.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )

        self.result.setWordWrap(
            False
        )

        self.result.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )

        self.result.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )

        self.result.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers
        )

        table_font = (
            self.result.font()
        )

        table_font.setPointSize(
            8
        )

        self.result.setFont(
            table_font
        )

        header_font = (
            self.result
            .horizontalHeader()
            .font()
        )

        header_font.setPointSize(
            8
        )

        self.result.horizontalHeader().setFont(
            header_font
        )

        self.load_all_data()

        self.refresh_timer = QTimer(
            self
        )

        self.refresh_timer.timeout.connect(
            self.load_all_data
        )

        self.refresh_timer.start(
            1000
        )

    def go_home(self):

        self.refresh_timer.stop()

        self.admin_window.show()
        self.close()

    def load_all_data(self):

        conn = connect_db()
        cursor = conn.cursor()

        cursor.execute(
            """
            SELECT *
            FROM car_management
            ORDER BY id
            """
        )

        rows = cursor.fetchall()

        self.show_data(
            cursor,
            rows
        )

        cursor.close()
        conn.close()

    def search_vehicle(self):

        keyword = (
            self.search
            .text()
            .strip()
        )

        if keyword == "":

            self.load_all_data()
            return

        conn = connect_db()
        cursor = conn.cursor()

        search_word = (
            f"%{keyword}%"
        )

        cursor.execute(
            """
            SELECT *
            FROM car_management
            WHERE car_number LIKE %s
            ORDER BY id
            """,
            (
                search_word,
            )
        )

        rows = cursor.fetchall()

        self.show_data(
            cursor,
            rows
        )

        cursor.close()
        conn.close()

    def add_vehicle(self):

        car_number = (
            self.car_number
            .text()
            .strip()
        )

        model = (
            self.model
            .text()
            .strip()
        )

        location = (
            self.location
            .text()
            .strip()
            .lower()
        )

        if car_number == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "차량번호를 입력해주세요."
            )

            return

        if model == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "모델명을 입력해주세요."
            )

            return

        if location == "":

            QMessageBox.warning(
                self,
                "입력 오류",
                "주차위치를 입력해주세요."
            )

            return

        parking_spaces = [
            "a1",
            "a2",
            "a3",
            "b1",
            "b2",
            "b3"
        ]

        if location not in parking_spaces:

            QMessageBox.warning(
                self,
                "주차위치 오류",
                "주차위치는 A1, A2, A3, "
                "B1, B2, B3 중 하나를 입력해주세요."
            )

            return

        input_time = (
            self.in_time
            .time()
            .toString(
                "HH:mm:ss"
            )
        )

        today = (
            datetime.now()
            .strftime(
                "%Y-%m-%d"
            )
        )

        entry_time = (
            f"{today} {input_time}"
        )

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT COUNT(*)
                FROM car_management
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            car_count = (
                cursor.fetchone()[0]
            )

            if car_count > 0:

                QMessageBox.warning(
                    self,
                    "차량 추가",
                    "이미 입차되어 있는 차량입니다."
                )

                return

            cursor.execute(
                """
                SELECT car_number
                FROM car_management
                WHERE LOWER(parking_location) = %s
                """,
                (
                    location,
                )
            )

            occupied = (
                cursor.fetchone()
            )

            if occupied is not None:

                QMessageBox.warning(
                    self,
                    "주차위치 오류",
                    f"{location.upper()} 위치에는 "
                    f"이미 {occupied[0]} 차량이 있습니다."
                )

                return

            cursor.execute(
                """
                SELECT COUNT(*)
                FROM membership
                WHERE car_number = %s
                  AND expiration_time >= CURDATE()
                """,
                (
                    car_number,
                )
            )

            member_count = (
                cursor.fetchone()[0]
            )

            if member_count > 0:
                member = "Y"
            else:
                member = "N"

            eco = check_eco_vehicle(
                cursor,
                model
            )

            disabled = (
                check_disabled_vehicle(
                    cursor,
                    car_number
                )
            )

            cursor.execute(
                """
                INSERT INTO car_management
                (
                    car_number,
                    entry_time,
                    member,
                    eco,
                    disabled,
                    model,
                    parking_location
                )
                VALUES
                (
                    %s,
                    %s,
                    %s,
                    %s,
                    %s,
                    %s,
                    %s
                )
                """,
                (
                    car_number,
                    entry_time,
                    member,
                    eco,
                    disabled,
                    model,
                    location
                )
            )

            conn.commit()

            QMessageBox.information(
                self,
                "차량 추가",
                "차량이 추가되었습니다.\n\n"
                f"차량번호 : {car_number}\n"
                f"모델 : {model}\n"
                f"주차위치 : {location.upper()}\n"
                f"입차시간 : {entry_time}\n"
                f"회원 : {member}\n"
                f"친환경 : {eco}\n"
                f"장애인 : {disabled}"
            )

            self.car_number.clear()
            self.model.clear()
            self.location.clear()
            self.search.clear()

            self.load_all_data()

        except mysql.connector.Error as e:

            QMessageBox.critical(
                self,
                "DB 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def delete_vehicle(self):

        selected_row = (
            self.result.currentRow()
        )

        if selected_row < 0:

            QMessageBox.warning(
                self,
                "차량 삭제",
                "삭제할 차량을 선택해주세요."
            )

            return

        if (
            "car_number"
            not in
            self.current_columns
        ):

            QMessageBox.warning(
                self,
                "삭제 오류",
                "car_number 컬럼을 찾을 수 없습니다."
            )

            return

        car_number_column = (
            self.current_columns.index(
                "car_number"
            )
        )

        item = self.result.item(
            selected_row,
            car_number_column
        )

        if item is None:
            return

        car_number = (
            item.text()
        )

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT
                    entry_time,
                    member,
                    eco,
                    disabled
                FROM car_management
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            vehicle_data = (
                cursor.fetchone()
            )

            if vehicle_data is None:

                QMessageBox.warning(
                    self,
                    "출차 오류",
                    "해당 차량의 입차 기록을 찾을 수 없습니다."
                )

                self.load_all_data()
                return

            (
                entry_time,
                member,
                eco,
                disabled
            ) = vehicle_data

            if isinstance(
                entry_time,
                str
            ):

                entry_time = (
                    datetime.strptime(
                        entry_time,
                        "%Y-%m-%d %H:%M:%S"
                    )
                )

            exit_time = datetime.now()

            parked_seconds = max(
                0,
                math.ceil(
                    (
                        exit_time
                        -
                        entry_time
                    ).total_seconds()
                )
            )

            basic_fee = (
                parked_seconds
                *
                10
            )

            member = str(
                member
                or
                "N"
            ).strip().upper()

            eco = str(
                eco
                or
                "N"
            ).strip().upper()

            disabled = str(
                disabled
                or
                "N"
            ).strip().upper()

            if member == "Y":

                discount_text = (
                    "회원 차량 전액 면제"
                )

                final_fee = 0

            elif (
                eco == "Y"
                or
                disabled == "Y"
            ):

                discount_text = (
                    "친환경/장애인 차량 50% 할인"
                )

                final_fee = (
                    basic_fee
                    //
                    2
                )

            else:

                discount_text = (
                    "할인 없음"
                )

                final_fee = basic_fee

            answer = QMessageBox.question(
                self,
                "출차 요금 정산",
                f"차량번호 : {car_number}\n"
                f"입차시간 : "
                f"{entry_time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"출차시간 : "
                f"{exit_time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                f"주차시간 : {parked_seconds}초\n"
                f"기본요금 : {basic_fee:,}원\n"
                f"할인내용 : {discount_text}\n"
                f"최종요금 : {final_fee:,}원\n\n"
                "정산 후 차량을 출차 처리하시겠습니까?",
                QMessageBox.StandardButton.Yes
                |
                QMessageBox.StandardButton.No
            )

            if (
                answer
                !=
                QMessageBox.StandardButton.Yes
            ):
                return

            cursor.execute(
                """
                DELETE FROM car_management
                WHERE car_number = %s
                """,
                (
                    car_number,
                )
            )

            conn.commit()

            QMessageBox.information(
                self,
                "출차 완료",
                f"{car_number} 차량의 출차 처리가 완료되었습니다.\n"
                f"최종요금 : {final_fee:,}원"
            )

            self.search.clear()
            self.load_all_data()

        except (
            mysql.connector.Error,
            ValueError,
            TypeError
        ) as e:

            QMessageBox.critical(
                self,
                "출차 처리 오류",
                str(e)
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

    def show_data(
        self,
        cursor,
        rows
    ):

        self.current_columns = [
            column[0]
            for column
            in cursor.description
        ]

        display_name_map = {

            "id":
                "번호",

            "car_number":
                "차량번호",

            "entry_time":
                "입차시간",

            "member":
                "회원",

            "eco":
                "친환경",

            "disabled":
                "장애",

            "model":
                "모델",

            "parking_location":
                "위치"
        }

        display_names = [

            display_name_map.get(
                column,
                column
            )

            for column
            in self.current_columns
        ]

        self.result.setColumnCount(
            len(
                self.current_columns
            )
        )

        self.result.setRowCount(
            len(rows)
        )

        self.result.setHorizontalHeaderLabels(
            display_names
        )

        header = (
            self.result
            .horizontalHeader()
        )

        header.setMinimumSectionSize(
            10
        )

        for column_index in range(
            len(
                self.current_columns
            )
        ):

            header.setSectionResizeMode(
                column_index,
                QHeaderView.ResizeMode.Fixed
            )

        if "id" in self.current_columns:

            index = (
                self.current_columns.index(
                    "id"
                )
            )

            self.result.setColumnWidth(
                index,
                34
            )

        if "member" in self.current_columns:

            index = (
                self.current_columns.index(
                    "member"
                )
            )

            self.result.setColumnWidth(
                index,
                44
            )

        if "eco" in self.current_columns:

            index = (
                self.current_columns.index(
                    "eco"
                )
            )

            self.result.setColumnWidth(
                index,
                50
            )

        if "disabled" in self.current_columns:

            index = (
                self.current_columns.index(
                    "disabled"
                )
            )

            self.result.setColumnWidth(
                index,
                44
            )

        if (
            "parking_location"
            in
            self.current_columns
        ):

            index = (
                self.current_columns.index(
                    "parking_location"
                )
            )

            self.result.setColumnWidth(
                index,
                20
            )

        if "entry_time" in self.current_columns:

            index = (
                self.current_columns.index(
                    "entry_time"
                )
            )

            self.result.setColumnWidth(
                index,
                175
            )

        if "car_number" in self.current_columns:

            index = (
                self.current_columns.index(
                    "car_number"
                )
            )

            self.result.setColumnWidth(
                index,
                115
            )

        if "model" in self.current_columns:

            index = (
                self.current_columns.index(
                    "model"
                )
            )

            self.result.setColumnWidth(
                index,
                70
            )

        for row_index, row in enumerate(
            rows
        ):

            for (
                column_index,
                value
            ) in enumerate(
                row
            ):

                if (
                    self.current_columns[
                        column_index
                    ]
                    ==
                    "id"
                ):

                    text = str(
                        row_index + 1
                    )

                else:

                    if value is None:
                        text = ""
                    else:
                        text = str(value)

                item = QTableWidgetItem(
                    text
                )

                item.setTextAlignment(
                    Qt.AlignmentFlag.AlignCenter
                )

                item.setToolTip(
                    text
                )

                self.result.setItem(
                    row_index,
                    column_index,
                    item
                )


class ParkingManagement(QDialog):

    def __init__(
        self,
        admin_window
    ):

        super().__init__()

        self.admin_window = (
            admin_window
        )

        uic.loadUi(
            "parking_management.ui",
            self
        )

        self.home_button.clicked.connect(
            self.go_home
        )

        self.udp_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.current_command = None

        self.servo_timer = QTimer(
            self
        )

        self.servo_timer.timeout.connect(
            self.send_servo_command
        )

        self.servo_timer.setInterval(
            100
        )

        self.left_button.pressed.connect(
            self.start_left
        )

        self.left_button.released.connect(
            self.stop_servo_command
        )

        self.right_button.pressed.connect(
            self.start_right
        )

        self.right_button.released.connect(
            self.stop_servo_command
        )

        self.cap = None

        self.camera_timer = QTimer(
            self
        )

        self.camera_timer.timeout.connect(
            self.update_camera
        )

        self.parking_refresh_timer = QTimer(
            self
        )

        self.parking_refresh_timer.timeout.connect(
            self.load_parking_status
        )

        self.parking_refresh_timer.start(
            1000
        )

        self.load_parking_status()

    def start_left(self):

        self.current_command = "LEFT"

        self.send_servo_command()

        self.servo_timer.start()

    def start_right(self):

        self.current_command = "RIGHT"

        self.send_servo_command()

        self.servo_timer.start()

    def stop_servo_command(self):

        self.servo_timer.stop()

        self.current_command = None

    def send_servo_command(self):

        if self.current_command is None:
            return

        try:

            self.udp_socket.sendto(
                self.current_command.encode(),
                (
                    ESP32_IP,
                    ESP32_PORT
                )
            )

            print(
                "UDP SEND:",
                self.current_command
            )

        except Exception as e:

            print(
                "UDP ERROR:",
                e
            )

    def start_camera(self):

        if self.cap is not None and self.cap.isOpened():
            if not self.camera_timer.isActive():
                self.camera_timer.start(33)

            return

        self.cap = cv2.VideoCapture(
            MONITOR_CAMERA_DEVICE,
            cv2.CAP_V4L2
        )

        if self.cap.isOpened():
            self.camera_timer.start(33)

        else:

            self.cap.release()
            self.cap = None

            self.camera_view.setText(
                f"모니터링 카메라 연결 실패: {MONITOR_CAMERA_DEVICE}"
            )

            self.camera_view.setAlignment(
                Qt.AlignmentFlag.AlignCenter
            )

    def update_camera(self):

        if self.cap is None:
            return

        ret, frame = self.cap.read()

        if not ret:
            return

        frame = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        height, width, channel = (
            frame.shape
        )

        bytes_per_line = (
            channel * width
        )

        image = QImage(
            frame.data,
            width,
            height,
            bytes_per_line,
            QImage.Format.Format_RGB888
        )

        pixmap = QPixmap.fromImage(
            image
        )

        pixmap = pixmap.scaled(
            self.camera_view.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation
        )

        self.camera_view.setPixmap(
            pixmap
        )

        self.camera_view.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )

    def stop_camera(self):

        if self.camera_timer.isActive():

            self.camera_timer.stop()

        if self.cap is not None:

            if self.cap.isOpened():
                self.cap.release()

            self.cap = None

    def go_home(self):

        self.stop_servo_command()
        self.stop_camera()
        self.parking_refresh_timer.stop()

        try:
            self.udp_socket.close()
        except:
            pass

        self.admin_window.show()
        self.close()

    def closeEvent(
        self,
        event
    ):

        self.stop_servo_command()
        self.stop_camera()
        self.parking_refresh_timer.stop()

        try:

            self.udp_socket.close()

        except:

            pass

        event.accept()

    def load_parking_status(self):

        parking_status = {
            "a1": None,
            "a2": None,
            "a3": None,
            "b1": None,
            "b2": None,
            "b3": None
        }

        conn = None
        cursor = None

        try:

            conn = connect_db()
            cursor = conn.cursor()

            cursor.execute(
                """
                SELECT
                    car_number,
                    parking_location
                FROM car_management
                WHERE parking_location
                      IS NOT NULL
                """
            )

            rows = (
                cursor.fetchall()
            )

            for (
                car_number,
                parking_location
            ) in rows:

                if parking_location is None:
                    continue

                location = (
                    parking_location
                    .strip()
                    .lower()
                )

                if location in parking_status:

                    parking_status[
                        location
                    ] = car_number

        except mysql.connector.Error as e:

            print(
                "[PARKING DB ERROR]",
                e
            )

        finally:

            if cursor is not None:
                cursor.close()

            if conn is not None:
                conn.close()

        self.set_parking_space(
            self.A1,
            self.A1_N,
            parking_status["a1"]
        )

        self.set_parking_space(
            self.A2,
            self.A2_N,
            parking_status["a2"]
        )

        self.set_parking_space(
            self.A3,
            self.A3_N,
            parking_status["a3"]
        )

        self.set_parking_space(
            self.B1,
            self.B1_N,
            parking_status["b1"]
        )

        self.set_parking_space(
            self.B2,
            self.B2_N,
            parking_status["b2"]
        )

        self.set_parking_space(
            self.B3,
            self.B3_N,
            parking_status["b3"]
        )

    def set_parking_space(
        self,
        frame,
        number_label,
        car_number
    ):

        if car_number is not None:

            frame.setStyleSheet(
                """
                QFrame {
                    background-color: #E74C3C;
                    border: 2px solid #922B21;
                    border-radius: 8px;
                }
                """
            )

            number_label.setText(
                str(car_number)
            )

            number_label.setStyleSheet(
                """
                QLabel {
                    background-color: transparent;
                    border: none;
                    color: white;
                    font-size: 15px;
                    font-weight: bold;
                }
                """
            )

        else:

            frame.setStyleSheet(
                """
                QFrame {
                    background-color: #3498DB;
                    border: 2px solid #1F618D;
                    border-radius: 8px;
                }
                """
            )

            number_label.setText(
                ""
            )

            number_label.setStyleSheet(
                """
                QLabel {
                    background-color: transparent;
                    border: none;
                }
                """
            )

        number_label.setAlignment(
            Qt.AlignmentFlag.AlignCenter
        )


if __name__ == "__main__":

    app = QApplication(
        sys.argv
    )

    network_servers = (
        start_network_servers()
    )

    parking_plate_worker = (
        ParkingPlateWorker()
    )

    parking_plate_worker.start()

    slot_sync_timer = QTimer()

    slot_sync_timer.timeout.connect(
        lambda:
        broadcast_db_slot_flags(
            network_servers[3]
        )
    )

    slot_sync_timer.start(
        1000
    )

    window = Admin()

    window.show()

    detection_window = ParkingDetectionWindow(
        parking_plate_worker
    )

    detection_window.show()

    exit_code = app.exec()

    parking_plate_worker.stop()

    parking_plate_worker.join(
        timeout=5.0
    )

    sys.exit(exit_code)