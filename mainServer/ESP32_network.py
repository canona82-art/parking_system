"""ESP32 IO 보드와 두 대의 ESP32-CAM을 연결하는 통신 모듈."""

import os
import socket
import threading
from datetime import datetime

from network_basic import (
    AI_SLOT_FLAGS,
    ENTRY_RECOGNIZED,
    EXIT_FARE_INFO,
    REQUEST,
    RESPONSE,
    EVENT,
    SENSOR1_ENTRY,
    SENSOR2_ENTRY_CONFIRMED,
    SENSOR3_EXIT,
    SENSOR4_EXIT_CONFIRMED,
    STATUS_ERROR,
    STATUS_OK,
    pack_entry_result,
    pack_exit_result,
    pack_slot_flags,
    pack_status,
    recv_packet,
    send_packet,
    unpack_event_id,
)


ESP32_HOST = os.getenv("ESP32_HOST", "0.0.0.0")
ESP32_PORT = int(os.getenv("ESP32_PORT", "5000"))


class ESP32Server:
    """ESP32 센서 요청과 AI의 주차 상태 이벤트를 처리한다."""

    def __init__(
        self,
        entry_camera_receiver,
        exit_camera_receiver,
        ai_requester,
        fee_provider=None,
        entry_recorder=None,
        host=ESP32_HOST,
        port=ESP32_PORT,
    ):
        self.entry_camera_receiver = entry_camera_receiver
        self.exit_camera_receiver = exit_camera_receiver
        self.ai_requester = ai_requester
        self.fee_provider = fee_provider or (lambda _vehicle: 0)
        self.entry_recorder = entry_recorder or (
            lambda _vehicle, _vehicle_type, _detected_at, _event_id: None
        )
        self.host = host
        self.port = port

        self._event_id = 0
        self._event_id_lock = threading.Lock()
        self._clients = {}
        self._clients_lock = threading.Lock()
        self._slot_flags = None
        self._slot_flags_lock = threading.Lock()

    def start(self):
        """ESP32 접속 대기 루프를 데몬 스레드로 시작한다."""
        thread = threading.Thread(target=self._listen, daemon=True)
        thread.start()
        return thread

    def _listen(self):
        """ESP32 접속을 받고 보드별 처리 스레드를 만든다."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(5)
        print(f"[ESP32] TCP {self.host}:{self.port}")

        while True:
            client, address = server.accept()
            threading.Thread(
                target=self._handle_client,
                args=(client, address),
                daemon=True,
            ).start()

    def _next_event_id(self):
        """입차·출차 및 상태 이벤트를 식별할 번호를 발급한다."""
        with self._event_id_lock:
            self._event_id = (self._event_id + 1) & 0xFFFFFFFF
            if self._event_id == 0:
                self._event_id = 1
            return self._event_id

    def _register_client(self, client):
        """ESP32 소켓과 동시 송신 방지용 Lock을 등록한다."""
        send_lock = threading.Lock()
        with self._clients_lock:
            self._clients[client] = send_lock

        with self._slot_flags_lock:
            flags = None if self._slot_flags is None else list(self._slot_flags)

        # 재접속한 ESP32에도 마지막 AI 상태를 즉시 전달한다.
        if flags is not None:
            self._send_to_client(
                client,
                EVENT,
                AI_SLOT_FLAGS,
                self._next_event_id(),
                pack_slot_flags(flags),
            )

    def _remove_client(self, client):
        with self._clients_lock:
            self._clients.pop(client, None)
        try:
            client.close()
        except OSError:
            pass

    def _send_to_client(self, client, message_type, command, request_id, payload):
        """한 ESP32 소켓에 Lock을 사용해 패킷을 보낸다."""
        with self._clients_lock:
            send_lock = self._clients.get(client)

        if send_lock is None:
            raise OSError("ESP32 client is not registered")

        with send_lock:
            send_packet(client, message_type, command, request_id, payload)

    def broadcast_slot_flags(self, flags):
        """AI 후보 리스트에서 계산한 6개의 bool을 연결된 ESP32에 보낸다."""
        payload = pack_slot_flags(flags)
        with self._slot_flags_lock:
            self._slot_flags = list(flags)

        dead_clients = []
        with self._clients_lock:
            clients = list(self._clients)

        for client in clients:
            try:
                self._send_to_client(
                    client,
                    EVENT,
                    AI_SLOT_FLAGS,
                    self._next_event_id(),
                    payload,
                )
            except OSError:
                dead_clients.append(client)

        for client in dead_clients:
            self._remove_client(client)

        print("[AI SLOT FLAGS]", list(flags))

    @staticmethod
    def _current_time():
        return datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S")

    def _process_entry_sensor(self, client, request_id):
        """센서1 당시의 입차 카메라 최신 프레임을 OCR 서버로 보낸다."""
        event_id = self._next_event_id()
        detected_at = self._current_time()

        try:
            frame = self.entry_camera_receiver.get_latest_frame()
            result = self.ai_requester(
                "entry",
                frame["image_data"] if frame else None,
                request_id,
            )
            vehicle = result["vehicle_number"]
            vehicle_type = result.get("vehicle_type", "")
            self.entry_recorder(vehicle, vehicle_type, detected_at, event_id)

            self._send_to_client(
                client,
                RESPONSE,
                ENTRY_RECOGNIZED,
                request_id,
                pack_entry_result(STATUS_OK, event_id, vehicle, detected_at),
            )
            print(
                "[ENTRY OCR]",
                vehicle,
                vehicle_type,
                detected_at,
            )
        except (OSError, RuntimeError, ValueError, KeyError, TypeError) as error:
            print("[ENTRY OCR ERROR]", error)
            self._send_to_client(
                client,
                RESPONSE,
                ENTRY_RECOGNIZED,
                request_id,
                pack_entry_result(STATUS_ERROR, event_id, "", detected_at),
            )

    def _process_entry_confirmed(self, client, request_id, payload):
        """센서2 요청에 처리 완료 응답을 보낸다."""
        unpack_event_id(payload)
        self._send_to_client(
            client,
            RESPONSE,
            SENSOR2_ENTRY_CONFIRMED,
            request_id,
            pack_status(STATUS_OK),
        )

    def _process_exit_sensor(self, client, request_id):
        """센서3 당시의 출차 카메라 최신 프레임을 OCR 서버로 보낸다."""
        event_id = self._next_event_id()

        try:
            frame = self.exit_camera_receiver.get_latest_frame()
            result = self.ai_requester(
                "exit",
                frame["image_data"] if frame else None,
                request_id,
            )
            vehicle = result["vehicle_number"]
            fee = int(self.fee_provider(vehicle))

            self._send_to_client(
                client,
                RESPONSE,
                EXIT_FARE_INFO,
                request_id,
                pack_exit_result(STATUS_OK, event_id, vehicle, fee),
            )
            print("[EXIT OCR]", vehicle, fee)
        except (OSError, RuntimeError, ValueError, KeyError, TypeError) as error:
            print("[EXIT OCR ERROR]", error)
            self._send_to_client(
                client,
                RESPONSE,
                EXIT_FARE_INFO,
                request_id,
                pack_exit_result(STATUS_ERROR, event_id, "", 0),
            )

    def _process_exit_confirmed(self, client, request_id, payload):
        """센서4 요청에 처리 완료 응답을 보낸다."""
        unpack_event_id(payload)
        self._send_to_client(
            client,
            RESPONSE,
            SENSOR4_EXIT_CONFIRMED,
            request_id,
            pack_status(STATUS_OK),
        )

    def _handle_client(self, client, address):
        """ESP32 센서 명령을 알맞은 처리 함수로 분배한다."""
        self._register_client(client)
        print("[ESP32 CONNECTED]", address)

        try:
            while True:
                version, message_type, command, request_id, payload = recv_packet(
                    client
                )

                if version != 1 or message_type != REQUEST:
                    raise ValueError("invalid ESP32 request")

                if command == SENSOR1_ENTRY and not payload:
                    self._process_entry_sensor(client, request_id)
                elif command == SENSOR2_ENTRY_CONFIRMED:
                    self._process_entry_confirmed(client, request_id, payload)
                elif command == SENSOR3_EXIT and not payload:
                    self._process_exit_sensor(client, request_id)
                elif command == SENSOR4_EXIT_CONFIRMED:
                    self._process_exit_confirmed(client, request_id, payload)
                else:
                    raise ValueError(f"unknown ESP32 command: 0x{command:04x}")
        except (ConnectionError, OSError, ValueError) as error:
            print("[ESP32 DISCONNECTED]", address, error)
        finally:
            self._remove_client(client)
