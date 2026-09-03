"""프로젝트 전체에서 공유하는 바이너리 통신 규격."""

import struct


# 공통 패킷의 버전과 메시지 종류
VERSION = 1
REQUEST = 1
RESPONSE = 2
EVENT = 3

# ESP32 센서 명령
SENSOR1_ENTRY = 0x10
SENSOR2_ENTRY_CONFIRMED = 0x11
SENSOR3_EXIT = 0x12
SENSOR4_EXIT_CONFIRMED = 0x13

# ESP32에 보내는 서버 응답
ENTRY_RECOGNIZED = 0x20
EXIT_FARE_INFO = 0x21
AI_SLOT_FLAGS = 0x22

# OCR AI 서버 명령
OCR_REQUEST = 0x40
OCR_RESULT = 0x41
AI_CANDIDATE_LIST = 0x42

# PyQt GUI 명령
GUI_GET_STATUS = 0x50
GUI_STATUS = 0x51
GUI_EVENT = 0x52

# GUI 이벤트 종류
ENTRY_SENSOR1 = 1
ENTRY_RECOGNIZED_EVENT = 2
ENTRY_CONFIRMED = 3
EXIT_SENSOR3 = 4
EXIT_INFO = 5
EXIT_CONFIRMED = 6
COMMUNICATION_ERROR = 255

# 응답 상태
STATUS_OK = 0
STATUS_ERROR = 1

# 고정 길이 필드. 모든 클라이언트가 같은 크기로 읽을 수 있게 한다.
VEHICLE_NUMBER_SIZE = 32
VEHICLE_TYPE_SIZE = 32
TIMESTAMP_SIZE = 20
AI_SLOT_COUNT = 6

# version, message_type, command, request_id, data_size
HEADER_FORMAT = "!BBHII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MAX_PAYLOAD_SIZE = 8 * 1024 * 1024


def recv_exact(sock, size):
    """TCP에서 지정한 크기의 데이터를 모두 수신한다."""
    data = bytearray()

    while len(data) < size:
        chunk = sock.recv(size - len(data))

        if not chunk:
            raise ConnectionError("peer disconnected")

        data.extend(chunk)

    return bytes(data)


def recv_packet(sock):
    """공통 헤더와 payload를 읽어 튜플로 반환한다."""
    header = recv_exact(sock, HEADER_SIZE)
    version, message_type, command, request_id, data_size = struct.unpack(
        HEADER_FORMAT,
        header,
    )

    if data_size > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload too large: {data_size}")

    payload = recv_exact(sock, data_size)
    return version, message_type, command, request_id, payload


def send_packet(sock, message_type, command, request_id, payload=b""):
    """공통 헤더를 붙여 하나의 TCP 패킷을 전송한다."""
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload too large: {len(payload)}")

    header = struct.pack(
        HEADER_FORMAT,
        VERSION,
        message_type,
        command,
        request_id,
        len(payload),
    )
    sock.sendall(header + payload)


def encode_text(value, size):
    """문자열을 UTF-8 고정 길이 필드로 변환한다."""
    encoded = str(value or "").encode("utf-8")
    # UTF-8 문자의 중간에서 잘리지 않도록 한다.
    encoded = encoded[: size - 1].decode("utf-8", errors="ignore").encode("utf-8")
    return encoded.ljust(size, b"\x00")


def decode_text(value):
    """고정 길이 문자열의 padding을 제거하고 UTF-8로 복원한다."""
    return value.split(b"\x00", 1)[0].decode("utf-8", errors="replace")


def pack_status(status):
    """성공 또는 실패 상태를 payload로 만든다."""
    return struct.pack("!B", status)


def pack_event_id(event_id):
    """센서 확정 메시지용 event_id를 payload로 만든다."""
    return struct.pack("!I", event_id)


def unpack_event_id(payload):
    """센서 확정 payload에서 event_id를 읽는다."""
    if len(payload) != 4:
        raise ValueError("event_id payload must be 4 bytes")

    return struct.unpack("!I", payload)[0]


def pack_entry_result(status, event_id, vehicle_number, recognized_at):
    """입차 차량번호와 인식 시간을 ESP32 응답 payload로 만든다."""
    return (
        struct.pack("!BI", status, event_id)
        + encode_text(vehicle_number, VEHICLE_NUMBER_SIZE)
        + encode_text(recognized_at, TIMESTAMP_SIZE)
    )


def pack_exit_result(status, event_id, vehicle_number, fee):
    """출차 차량번호와 요금을 ESP32 응답 payload로 만든다."""
    return (
        struct.pack("!BI", status, event_id)
        + encode_text(vehicle_number, VEHICLE_NUMBER_SIZE)
        + struct.pack("!I", fee)
    )


def pack_ocr_request(direction, image_data):
    """OCR 요청 payload를 방향 1바이트와 JPEG로 구성한다."""
    direction_code = 1 if direction == "entry" else 2
    return struct.pack("!B", direction_code) + image_data


def unpack_ocr_request(payload):
    """AI 서버가 OCR 요청에서 방향과 JPEG를 분리한다."""
    if len(payload) < 2:
        raise ValueError("OCR request must contain direction and image")

    direction_code = payload[0]
    if direction_code == 1:
        direction = "entry"
    elif direction_code == 2:
        direction = "exit"
    else:
        raise ValueError("invalid OCR direction")

    return direction, payload[1:]


def unpack_ocr_result(payload):
    """OCR 결과에서 상태, 차량번호, 차량 종류를 읽는다."""
    expected_size = 1 + VEHICLE_NUMBER_SIZE + VEHICLE_TYPE_SIZE

    if len(payload) != expected_size:
        raise ValueError("invalid OCR result size")

    status = payload[0]
    plate_start = 1
    type_start = plate_start + VEHICLE_NUMBER_SIZE

    return (
        status,
        decode_text(payload[plate_start:type_start]),
        decode_text(payload[type_start:]),
    )


def pack_ocr_result(status, vehicle_number, vehicle_type=""):
    """OCR 팀이 반환할 결과 payload를 구성한다."""
    return (
        struct.pack("!B", status)
        + encode_text(vehicle_number, VEHICLE_NUMBER_SIZE)
        + encode_text(vehicle_type, VEHICLE_TYPE_SIZE)
    )


def pack_gui_status(total, occupied, available):
    """PyQt GUI용 주차 현황 payload를 구성한다."""
    return struct.pack("!HHH", total, occupied, available)


def pack_gui_event(event_type, event_id, vehicle_number, event_time, fee):
    """PyQt GUI용 이벤트 payload를 구성한다."""
    return (
        struct.pack("!BI", event_type, event_id)
        + encode_text(vehicle_number, VEHICLE_NUMBER_SIZE)
        + encode_text(event_time, TIMESTAMP_SIZE)
        + struct.pack("!I", fee)
    )


def pack_ai_candidate_list(candidates):
    """AI 서버의 6칸 차량번호 후보 리스트를 고정 길이 payload로 만든다."""
    if len(candidates) != AI_SLOT_COUNT:
        raise ValueError(f"candidate list must contain {AI_SLOT_COUNT} items")

    return b"".join(
        encode_text("" if value is None else value, VEHICLE_NUMBER_SIZE)
        for value in candidates
    )


def unpack_ai_candidate_list(payload):
    """AI 서버의 후보 리스트를 6개의 문자열 또는 None으로 복원한다."""
    expected_size = AI_SLOT_COUNT * VEHICLE_NUMBER_SIZE

    if len(payload) != expected_size:
        raise ValueError("invalid AI candidate list size")

    candidates = []
    for index in range(AI_SLOT_COUNT):
        start = index * VEHICLE_NUMBER_SIZE
        value = decode_text(payload[start:start + VEHICLE_NUMBER_SIZE])
        candidates.append(value or None)
    return candidates


def pack_slot_flags(flags):
    """6개의 점유 여부를 ESP32로 보낼 0/1 payload로 만든다."""
    if len(flags) != AI_SLOT_COUNT:
        raise ValueError(f"slot flag list must contain {AI_SLOT_COUNT} items")

    return bytes(1 if flag else 0 for flag in flags)


def unpack_slot_flags(payload):
    """ESP32 수신용 점유 여부 payload를 bool 리스트로 복원한다."""
    if len(payload) != AI_SLOT_COUNT:
        raise ValueError("invalid slot flag payload size")

    if any(value not in (0, 1) for value in payload):
        raise ValueError("slot flags must be 0 or 1")

    return [bool(value) for value in payload]
