"""ESP32-CAM과 메인 서버 사이의 UDP 영상 통신 모듈."""

import os
import socket
import struct
import threading
from datetime import datetime


# ESP32-CAM UDP 패킷의 헤더: frame_id(4) + packet_id(2) + total_packets(2)
CAMERA_HEADER_FORMAT = "!IHH"
CAMERA_HEADER_SIZE = struct.calcsize(CAMERA_HEADER_FORMAT)
CAMERA_HOST = os.getenv("CAMERA_HOST", "0.0.0.0")
CAMERA_PORT = int(os.getenv("CAMERA_PORT", "5001"))


class CameraReceiver:
    """ESP32-CAM JPEG 조각을 재조립하고 최신 프레임을 제공한다."""

    def __init__(self, host=CAMERA_HOST, port=CAMERA_PORT):
        """카메라 UDP 수신 주소와 내부 프레임 저장 공간을 초기화한다."""
        self.host = host
        self.port = port

        # 완성된 JPEG 한 장을 보관하는 변수다.
        self._latest_frame = None
        self._frame_lock = threading.Lock()

    def get_latest_frame(self):
        """OCR 요청에 사용할 최신 JPEG 정보를 반환한다."""
        with self._frame_lock:
            if self._latest_frame is None:
                return None

            return dict(self._latest_frame)

    def run(self):
        """UDP 패킷을 계속 수신해 JPEG 프레임을 재조립한다."""
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp.bind((self.host, self.port))
        print(f"[CAMERA] UDP {self.host}:{self.port}")

        # frame_id별로 아직 완성되지 않은 JPEG 조각을 저장한다.
        frame_buffers = {}

        while True:
            data, address = udp.recvfrom(65535)

            if len(data) < CAMERA_HEADER_SIZE:
                continue

            frame_id, packet_id, total_packets = struct.unpack(
                CAMERA_HEADER_FORMAT,
                data[:CAMERA_HEADER_SIZE],
            )
            payload = data[CAMERA_HEADER_SIZE:]

            if total_packets == 0 or packet_id >= total_packets:
                continue

            frame = frame_buffers.setdefault(
                frame_id,
                {"total": total_packets, "parts": {}},
            )
            frame["parts"][packet_id] = payload

            if len(frame["parts"]) != frame["total"]:
                continue

            image_data = b"".join(
                frame["parts"][index] for index in range(frame["total"])
            )

            with self._frame_lock:
                self._latest_frame = {
                    "frame_id": frame_id,
                    "image_data": image_data,
                    "received_at": datetime.now().astimezone().strftime(
                        "%Y-%m-%d %H:%M:%S"
                    ),
                    "camera_ip": address[0],
                }

            # 한 프레임이 완성되면 이전 조각은 다음 프레임을 위해 비운다.
            frame_buffers.clear()
            print(f"[CAMERA] frame={frame_id}, bytes={len(image_data)}")

    def start(self):
        """카메라 수신 루프를 데몬 스레드로 시작한다."""
        thread = threading.Thread(target=self.run, daemon=True)
        thread.start()
        return thread

