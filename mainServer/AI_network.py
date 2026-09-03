"""메인 서버와 지속 연결하는 OCR AI 서버 통신 모듈."""

import os
import queue
import socket
import threading

from network_basic import (
    AI_CANDIDATE_LIST,
    OCR_REQUEST,
    OCR_RESULT,
    REQUEST,
    RESPONSE,
    EVENT,
    STATUS_OK,
    pack_ocr_request,
    recv_packet,
    send_packet,
    unpack_ai_candidate_list,
    unpack_ocr_result,
)


# AI 서버가 메인 서버로 접속할 주소와 포트다.
AI_BIND_HOST = os.getenv("AI_BIND_HOST", "0.0.0.0")
AI_SERVER_PORT = int(os.getenv("AI_SERVER_PORT", "6000"))
AI_SERVER_TIMEOUT = float(os.getenv("AI_SERVER_TIMEOUT", "5"))


class AIServer:
    """AI 서버 한 대와의 지속 TCP 연결을 관리한다.

    같은 연결에서 메인 서버가 OCR 요청을 보내고, AI 서버가 OCR 응답 또는
    실시간 후보 리스트 이벤트를 보낼 수 있다.
    """

    def __init__(
        self,
        host=AI_BIND_HOST,
        port=AI_SERVER_PORT,
        timeout=AI_SERVER_TIMEOUT,
        on_candidate_list=None,
    ):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.on_candidate_list = on_candidate_list

        self._client = None
        self._client_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._connected = threading.Event()
        self._pending = {}
        self._pending_lock = threading.Lock()

    def start(self):
        """AI 서버 접속 대기 루프를 데몬 스레드로 시작한다."""
        thread = threading.Thread(target=self._listen, daemon=True)
        thread.start()
        return thread

    def _listen(self):
        """AI 서버의 접속을 받고 연결별 수신 루프를 실행한다."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(1)
        print(f"[AI] TCP {self.host}:{self.port}")

        while True:
            client, address = server.accept()
            self._replace_client(client)
            print("[AI CONNECTED]", address)
            self._read_client(client, address)

    def _replace_client(self, client):
        """재접속 시 이전 AI 연결을 교체한다."""
        with self._client_lock:
            previous = self._client
            self._client = client
            self._connected.set()

        if previous is not None:
            try:
                previous.close()
            except OSError:
                pass

    def _read_client(self, client, address):
        """OCR 응답과 AI의 실시간 후보 이벤트를 분배한다."""
        try:
            while True:
                version, message_type, command, request_id, payload = recv_packet(
                    client
                )

                if version != 1:
                    raise ValueError("invalid AI packet version")

                if message_type == RESPONSE and command == OCR_RESULT:
                    with self._pending_lock:
                        waiter = self._pending.get(request_id)

                    if waiter is not None:
                        try:
                            waiter.put_nowait(payload)
                        except queue.Full:
                            pass
                    continue

                if message_type == EVENT and command == AI_CANDIDATE_LIST:
                    candidates = unpack_ai_candidate_list(payload)
                    if self.on_candidate_list is not None:
                        self.on_candidate_list(candidates)
                    continue

                raise ValueError(
                    f"unknown AI packet: type={message_type}, command=0x{command:04x}"
                )
        except (ConnectionError, OSError, ValueError) as error:
            print("[AI DISCONNECTED]", address, error)
        finally:
            self._clear_client(client)

    def _clear_client(self, client):
        """현재 연결을 닫고 대기 중인 OCR 요청을 실패 처리한다."""
        with self._client_lock:
            if self._client is client:
                self._client = None
                self._connected.clear()

        try:
            client.close()
        except OSError:
            pass

        with self._pending_lock:
            waiters = list(self._pending.values())

        for waiter in waiters:
            try:
                waiter.put_nowait(ConnectionError("AI server disconnected"))
            except queue.Full:
                pass

    def request_ocr(self, direction, image_data, request_id):
        """AI 서버에 JPEG OCR을 요청하고 결과를 반환한다."""
        if not image_data:
            raise RuntimeError("camera image is not ready")

        if not self._connected.wait(self.timeout):
            raise RuntimeError("AI server is not connected")

        waiter = queue.Queue(maxsize=1)
        with self._pending_lock:
            self._pending[request_id] = waiter

        try:
            with self._client_lock:
                client = self._client

            if client is None:
                raise RuntimeError("AI server is not connected")

            with self._send_lock:
                send_packet(
                    client,
                    REQUEST,
                    OCR_REQUEST,
                    request_id,
                    pack_ocr_request(direction, image_data),
                )

            result = waiter.get(timeout=self.timeout)
            if isinstance(result, Exception):
                raise result
        except queue.Empty as error:
            raise RuntimeError("AI OCR response timed out") from error
        finally:
            with self._pending_lock:
                self._pending.pop(request_id, None)

        status, vehicle_number, vehicle_type = unpack_ocr_result(result)
        if status != STATUS_OK:
            raise RuntimeError("AI OCR returned an error")

        return {
            "vehicle_number": vehicle_number,
            "vehicle_type": vehicle_type,
        }
