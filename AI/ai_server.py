"""메인 서버와 연결되는 OCR AI 서버 통신 클라이언트."""

import os
import socket
import sys
import threading
from pathlib import Path

# mainServer와 공통 프로토콜 모듈을 공유한다.
PROTOCOL_DIR = Path(__file__).resolve().parent

if str(PROTOCOL_DIR) not in sys.path:
    sys.path.insert(0, str(PROTOCOL_DIR))

from network_basic import (  # noqa: E402
    AI_CANDIDATE_LIST,
    EVENT,
    OCR_REQUEST,
    OCR_RESULT,
    REQUEST,
    RESPONSE,
    STATUS_ERROR,
    STATUS_OK,
    pack_ai_candidate_list,
    pack_ocr_result,
    recv_packet,
    send_packet,
    unpack_ocr_request,
)

MAIN_SERVER_HOST = os.getenv("MAIN_SERVER_HOST", "192.168.0.220")
MAIN_SERVER_PORT = int(os.getenv("MAIN_SERVER_PORT", "6000"))
AI_CONNECT_TIMEOUT = float(os.getenv("AI_CONNECT_TIMEOUT", "5"))
AI_RECONNECT_DELAY = float(os.getenv("AI_RECONNECT_DELAY", "2"))


class AIServerClient:
    """메인 서버에 접속해 OCR 요청과 후보 리스트 이벤트를 처리한다."""

    def __init__(
        self,
        ocr_handler=None,
        host=MAIN_SERVER_HOST,
        port=MAIN_SERVER_PORT,
        connect_timeout=AI_CONNECT_TIMEOUT,
        reconnect_delay=AI_RECONNECT_DELAY,
    ):
        self.ocr_handler = ocr_handler
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.reconnect_delay = reconnect_delay

        self._socket = None
        self._socket_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._event_id = 0
        self._event_id_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._connected = threading.Event()

    def start(self):
        """메인 서버 재접속 루프를 데몬 스레드로 시작한다."""
        thread = threading.Thread(target=self.run_forever, daemon=True)
        thread.start()
        return thread

    def stop(self):
        """재접속을 중지하고 현재 연결을 닫는다."""
        self._stop_event.set()
        with self._socket_lock:
            current = self._socket
            self._socket = None
            self._connected.clear()

        if current is not None:
            try:
                current.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            current.close()

    def run_forever(self):
        """메인 서버에 연결된 동안 패킷을 계속 수신한다."""
        while not self._stop_event.is_set():
            client = None
            try:
                client = socket.create_connection(
                    (self.host, self.port),
                    timeout=self.connect_timeout,
                )
                client.settimeout(None)
                self._set_socket(client)
                print(f"[AI] Connected to main server {self.host}:{self.port}")
                self._receive_loop(client)
            except (ConnectionError, OSError, ValueError) as error:
                if not self._stop_event.is_set():
                    print("[AI] Main server connection error:", error)
            finally:
                if client is not None:
                    self._clear_socket(client)

            self._stop_event.wait(self.reconnect_delay)

    def _set_socket(self, client):
        with self._socket_lock:
            previous = self._socket
            self._socket = client
            self._connected.set()

        if previous is not None and previous is not client:
            try:
                previous.close()
            except OSError:
                pass

    def _clear_socket(self, client):
        with self._socket_lock:
            if self._socket is not client:
                return
            self._socket = None
            self._connected.clear()

        try:
            client.close()
        except OSError:
            pass

    def _receive_loop(self, client):
        """메인 서버의 OCR 요청을 각 작업 스레드로 넘긴다."""
        while not self._stop_event.is_set():
            version, message_type, command, request_id, payload = recv_packet(client)

            if version != 1 or message_type != REQUEST or command != OCR_REQUEST:
                raise ValueError("Invalid OCR request packet")

            threading.Thread(
                target=self._handle_ocr_request,
                args=(client, request_id, payload),
                daemon=True,
            ).start()

    def _handle_ocr_request(self, client, request_id, payload):
        """OCR 콜백 결과를 공통 OCR_RESULT 패킷으로 반환한다."""
        try:
            direction, image_data = unpack_ocr_request(payload)
            if self.ocr_handler is None:
                raise RuntimeError("ocr_handler is not configured")

            print(f"\n[AI SERVER] 메인 서버로부터 '{direction}' 이미지 수신 완료. 분석 시작...")
            
            result = self.ocr_handler(direction, image_data)
            vehicle_number = str(result.get("vehicle_number", "") or "")
            vehicle_type = str(result.get("vehicle_type", "") or "")

            if not vehicle_number:
                self._send(
                    client, RESPONSE, OCR_RESULT, request_id,
                    pack_ocr_result(STATUS_ERROR, "", "")
                )
                print(f"[AI SERVER] ❌ 번호판 인식 실패 (차량 없음 또는 인식 불가)")
                return

            self._send(
                client, RESPONSE, OCR_RESULT, request_id,
                pack_ocr_result(STATUS_OK, vehicle_number, vehicle_type)
            )
            
            # 터미널에서 한눈에 보이도록 강조된 로그 출력
            print("=" * 50)
            print(f" 🟢 [인식 성공] 위치: {direction.upper()}")
            print(f" 🚗 차종: {vehicle_type}")
            print(f" 📝 번호판: {vehicle_number}")
            print("=" * 50 + "\n")

        except Exception as error:
            print(f"[AI SERVER ERROR] {error}")
            try:
                self._send(
                    client, RESPONSE, OCR_RESULT, request_id,
                    pack_ocr_result(STATUS_ERROR, "", "")
                )
            except OSError:
                pass

    def _send(self, client, message_type, command, request_id, payload):
        """한 TCP 연결에 패킷이 섞이지 않도록 직렬화해 전송한다."""
        with self._socket_lock:
            if self._socket is not client:
                raise OSError("Main server connection is no longer active")

        with self._send_lock:
            send_packet(client, message_type, command, request_id, payload)

    def _next_event_id(self):
        with self._event_id_lock:
            self._event_id = (self._event_id + 1) & 0xFFFFFFFF
            if self._event_id == 0:
                self._event_id = 1
            return self._event_id

    def publish_candidates(self, candidates, trigger=False):
        """트리거 bool이 False일 때 6칸 후보 리스트를 메인 서버로 보낸다."""
        if trigger:
            return False

        if not self._connected.wait(self.connect_timeout):
            raise RuntimeError("Main server is not connected")

        with self._socket_lock:
            client = self._socket

        if client is None:
            raise RuntimeError("Main server is not connected")

        self._send(
            client,
            EVENT,
            AI_CANDIDATE_LIST,
            self._next_event_id(),
            pack_ai_candidate_list(candidates),
        )
        print("[AI CANDIDATES]", list(candidates))
        return True


if __name__ == "__main__":
    from ocr_adapter import ocr_handler

    # GPU 자원 사전 로드
    from ocr_adapter import get_ai_resources
    get_ai_resources()

    ai = AIServerClient(ocr_handler=ocr_handler)
    ai.start()

    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        ai.stop()