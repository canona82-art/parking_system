"""웹캠 2대를 입차·출차 UDP 카메라로 사용하는 송신기.

/dev/video0은 메인 서버 UDP 5001(입차)로,
/dev/video2는 메인 서버 UDP 5002(출차)로 전송한다.

메인 서버 CameraReceiver가 사용하는 기존 ESP32-CAM 프로토콜을
그대로 사용하므로 메인 서버의 수신 코드는 변경하지 않아도 된다.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from pathlib import Path

import cv2


UDP_PAYLOAD_SIZE = 1400
HEADER_FORMAT = "!IHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

DEFAULT_SERVER_IP = "192.168.0.220"
DEFAULT_ENTRY_PORT = 5001
DEFAULT_EXIT_PORT = 5002
V4L_BY_ID_DIR = Path("/dev/v4l/by-id")
BUILTIN_CAMERA_MARKERS = ("SunplusIT_Inc_HD_Webcam",)
DEFAULT_EXIT_CAMERA = "/dev/video2"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 10.0
DEFAULT_JPEG_QUALITY = 85


class CameraSender(threading.Thread):
    """웹캠 한 대의 프레임을 지정된 UDP 포트로 전송한다."""

    def __init__(
        self,
        name: str,
        camera_index: int | str,
        server_ip: str,
        server_port: int,
        width: int,
        height: int,
        fps: float,
        jpeg_quality: int,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(name=name, daemon=True)
        self.camera_index = camera_index
        self.destination = (server_ip, server_port)
        self.width = width
        self.height = height
        self.target_fps = fps
        self.jpeg_quality = jpeg_quality
        self.stop_event = stop_event
        self.frame_id = 0
        self.frame_lock = threading.Lock()
        self.latest_frame = None

    def run(self) -> None:
        # int, str 상관없이 Linux에서는 CAP_V4L2 백엔드를 명시하는 것이 안정적입니다.
        if isinstance(self.camera_index, str):
            camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        else:
            camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            if not camera.isOpened():
                print(
                    f"[{self.name}] 웹캠을 열 수 없습니다: "
                    f"index={self.camera_index}"
                )
                return

            camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if self.target_fps > 0:
                camera.set(cv2.CAP_PROP_FPS, self.target_fps)

            print(
                f"[{self.name}] camera={self.camera_index} "
                f"-> {self.destination[0]}:{self.destination[1]}"
            )

            interval = (
                1.0 / self.target_fps
                if self.target_fps > 0
                else 0.0
            )
            next_send_time = time.monotonic()

            while not self.stop_event.is_set():
                success, frame = camera.read()

                if not success:
                    print(f"[{self.name}] 프레임을 읽지 못했습니다.")
                    time.sleep(0.01)
                    continue

                self.send_frame(frame, udp)
                with self.frame_lock:
                    self.latest_frame = frame.copy()

                if interval > 0:
                    next_send_time += interval
                    wait_time = next_send_time - time.monotonic()

                    if wait_time > 0:
                        time.sleep(wait_time)
                    else:
                        next_send_time = time.monotonic()

        except Exception as exc:
            print(
                f"[{self.name}] 송신 오류: "
                f"{type(exc).__name__}: {exc}"
            )

        finally:
            camera.release()
            udp.close()
            print(f"[{self.name}] stopped")

    def get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def send_frame(self, frame, udp: socket.socket) -> None:
        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )

        if not success:
            print(f"[{self.name}] JPEG 인코딩 실패")
            return

        jpeg = encoded.tobytes()
        total_packets = (
            len(jpeg) + UDP_PAYLOAD_SIZE - 1
        ) // UDP_PAYLOAD_SIZE

        if total_packets == 0 or total_packets > 0xFFFF:
            print(
                f"[{self.name}] JPEG 크기 오류: "
                f"{len(jpeg)} bytes"
            )
            return

        current_frame_id = self.frame_id & 0xFFFFFFFF

        for packet_id in range(total_packets):
            start = packet_id * UDP_PAYLOAD_SIZE
            end = start + UDP_PAYLOAD_SIZE

            header = struct.pack(
                HEADER_FORMAT,
                current_frame_id,
                packet_id,
                total_packets,
            )

            udp.sendto(
                header + jpeg[start:end],
                self.destination,
            )

        self.frame_id = (self.frame_id + 1) & 0xFFFFFFFF


def find_external_cameras() -> list[int | str]:
    """사용 가능한 실제 비디오 캡처 장치(인덱스)를 탐색하여 반환한다."""
    available_cameras = []

    # 1. 먼저 /dev/v4l/by-id 경로를 시도
    if V4L_BY_ID_DIR.exists():
        camera_paths = sorted(
            str(path)
            for path in V4L_BY_ID_DIR.glob("*")
            if "index0" in path.name
            and not any(marker in path.name for marker in BUILTIN_CAMERA_MARKERS)
        )
        if camera_paths:
            return camera_paths[:2]

    # 2. by-id 탐색 실패 시 /dev/video* 번호(0~9)를 순회하며 열리는 카메라 탐색
    for idx in range(10):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            # 실제로 프레임이 읽히는 장치만 추가
            if ret:
                available_cameras.append(idx)

    if not available_cameras:
        raise RuntimeError(
            "사용 가능한 웹캠을 찾지 못했습니다. "
            "USB 연결 상태나 ls /dev/video* 목록을 확인하세요."
        )

    return available_cameras[:2]


def main() -> None:
    stop_event = threading.Event()
    
    # 실제 연결된 카메라를 자동 탐색
    try:
        cams = find_external_cameras()
    except RuntimeError as e:
        print(f"카메라 탐색 실패: {e}")
        return

    entry_camera = cams[0]
    exit_camera = cams[1] if len(cams) > 1 else None

    entry_sender = CameraSender(
        name="ENTRY",
        camera_index=entry_camera,
        server_ip=DEFAULT_SERVER_IP,
        server_port=DEFAULT_ENTRY_PORT,
        width=DEFAULT_WIDTH,
        height=DEFAULT_HEIGHT,
        fps=DEFAULT_FPS,
        jpeg_quality=DEFAULT_JPEG_QUALITY,
        stop_event=stop_event,
    )

    exit_sender = None
    if exit_camera is not None:
        exit_sender = CameraSender(
            name="EXIT",
            camera_index=exit_camera,
            server_ip=DEFAULT_SERVER_IP,
            server_port=DEFAULT_EXIT_PORT,
            width=DEFAULT_WIDTH,
            height=DEFAULT_HEIGHT,
            fps=DEFAULT_FPS,
            jpeg_quality=DEFAULT_JPEG_QUALITY,
            stop_event=stop_event,
        )

    entry_sender.start()
    if exit_sender is not None:
        exit_sender.start()
    else:
        print("외장 카메라 1대만 감지됨: 입차 송신만 실행합니다.")

    # ... 이하 UI 루프 동일 ...

    print("웹캠 송신기 실행 중. q 또는 Ctrl+C로 종료하세요.")

    try:
        while not stop_event.is_set():
            senders = [entry_sender]
            if exit_sender is not None:
                senders.append(exit_sender)

            for sender in senders:
                frame = sender.get_latest_frame()
                if frame is not None:
                    cv2.imshow(f"{sender.name} Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                stop_event.set()
                break

            time.sleep(0.001)
    except KeyboardInterrupt:
        print("종료 요청을 받았습니다.")
        stop_event.set()
    finally:
        stop_event.set()
        entry_sender.join(timeout=2.0)
        if exit_sender is not None:
            exit_sender.join(timeout=2.0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
