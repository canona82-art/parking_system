#include "parking.h"

// ============================================================
// 설정과 공유 변수의 실제 정의
// ============================================================

const char* WIFI_SSID = "addinedu_201class_4-2.4G";
const char* WIFI_PASSWORD = "201class4!";
const char* SERVER_IP = "192.168.0.220"; // 파이썬 서버 IP 주소
const uint16_t SERVER_PORT = 5000;

// 초음파 센서 핀
const uint8_t SENSOR1_TRIG_PIN = 13;
const uint8_t SENSOR1_ECHO_PIN = 34;
const uint8_t SENSOR2_TRIG_PIN = 27;
const uint8_t SENSOR2_ECHO_PIN = 39;
const uint8_t SENSOR3_TRIG_PIN = 26;
const uint8_t SENSOR3_ECHO_PIN = 36;
const uint8_t SENSOR4_TRIG_PIN = 14;
const uint8_t SENSOR4_ECHO_PIN = 35;
const float SENSOR_DETECT_DISTANCE_CM = 5.0f;

WiFiClient serverClient;

// 프로토콜 상수
const uint8_t VERSION = 1;
const uint8_t REQUEST = 1;
const uint8_t RESPONSE = 2;
const uint8_t EVENT = 3;

const uint16_t SENSOR1_ENTRY = 0x10;
const uint16_t SENSOR2_ENTRY_CONFIRMED = 0x11;
const uint16_t SENSOR3_EXIT = 0x12;
const uint16_t SENSOR4_EXIT_CONFIRMED = 0x13;
const uint16_t ENTRY_RECOGNIZED = 0x20;
const uint16_t EXIT_FARE_INFO = 0x21;
const uint16_t AI_SLOT_FLAGS = 0x22;

const uint8_t STATUS_OK = 0;
const uint8_t STATUS_ERROR = 1;
const uint8_t VEHICLE_NUMBER_SIZE = 32;
const uint8_t TIMESTAMP_SIZE = 20;
const uint8_t HEADER_SIZE = 12;
const uint8_t AI_SLOT_COUNT = 6;

// 전역 상태 변수
uint32_t requestId = 0;
uint32_t entryEventId = 0;
uint32_t exitEventId = 0;

char recognizedVehicle[VEHICLE_NUMBER_SIZE] = {0};
char recognizedTime[TIMESTAMP_SIZE + 1] = {0};
char exitVehicle[VEHICLE_NUMBER_SIZE] = {0};
uint32_t exitFee = 0;

bool previousSensor1Detected = false;
bool previousSensor2Detected = false;
bool previousSensor3Detected = false;
bool previousSensor4Detected = false;
bool entryConfirmationSent = false;
bool exitConfirmationSent = false;
bool aiSlotFlags[6] = {false};

// ============================================================
// Big Endian 변환
// ============================================================
void writeUint16(uint8_t* buffer, uint16_t value) {
    buffer[0] = (value >> 8) & 0xFF;
    buffer[1] = value & 0xFF;
}

void writeUint32(uint8_t* buffer, uint32_t value) {
    buffer[0] = (value >> 24) & 0xFF;
    buffer[1] = (value >> 16) & 0xFF;
    buffer[2] = (value >> 8) & 0xFF;
    buffer[3] = value & 0xFF;
}

uint32_t readUint32(const uint8_t* buffer) {
    return ((uint32_t)buffer[0] << 24)
        | ((uint32_t)buffer[1] << 16)
        | ((uint32_t)buffer[2] << 8)
        | buffer[3];
}

// ============================================================
// 초음파 센서 계측
// ============================================================
float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return -1.0f;
    return (duration * 0.0343f) / 2.0f;
}

bool isUltrasonicTriggered(uint8_t trigPin, uint8_t echoPin, bool& previousDetected) {
    float distance = readDistanceCm(trigPin, echoPin);
    bool detected = (distance > 0.0f && distance <= SENSOR_DETECT_DISTANCE_CM);
    bool triggered = detected && !previousDetected;
    previousDetected = detected;
    return triggered;
}

// ============================================================
// 패킷 송수신
// ============================================================
bool sendPacket(uint8_t messageType, uint16_t command, uint32_t requestIdValue, const uint8_t* payload, uint32_t payloadSize) {
    if (!serverClient.connected()) {
        Serial.println("[TCP-TX ERROR] Server not connected, cannot send.");
        return false;
    }

    serverClient.setNoDelay(true);

    uint8_t header[12] = {0};
    header[0] = VERSION;
    header[1] = messageType;
    writeUint16(&header[2], command);
    writeUint32(&header[4], requestIdValue);
    writeUint32(&header[8], payloadSize);

    serverClient.write(header, 12);
    if (payloadSize > 0 && payload != nullptr) {
        serverClient.write(payload, payloadSize);
    }
    serverClient.flush();
    
    Serial.printf("[TCP-TX] MsgType: 0x%02X, CMD: 0x%04X, ReqId: %u, Size: %u\n", 
                  messageType, command, requestIdValue, payloadSize);
    return true;
}

bool receiveExact(uint8_t* buffer, size_t size, uint32_t timeoutMs) {
    size_t received = 0;
    uint32_t startedAt = millis();
    while (received < size) {
        if (millis() - startedAt > timeoutMs) return false;
        if (serverClient.available()) {
            int count = serverClient.read(buffer + received, size - received);
            if (count > 0) received += count;
        }
        delay(1);
    }
    return true;
}

void copyFixedText(char* destination, const uint8_t* source, size_t size) {
    memcpy(destination, source, size);
    destination[size - 1] = '\0';
}

bool receivePacket() {
    uint8_t header[12] = {0};
    if (!receiveExact(header, 12, 5000)) {
        Serial.println("[TCP-RX ERROR] Header receive timeout or incomplete.");
        return false;
    }

    uint8_t version = header[0];
    uint8_t messageType = header[1];
    uint16_t command = ((uint16_t)header[2] << 8) | header[3];
    uint32_t responseRequestId = readUint32(&header[4]);
    uint32_t dataSize = readUint32(&header[8]);

    Serial.printf("[TCP-RX] MsgType: 0x%02X, CMD: 0x%04X, ReqId: %u, DataSize: %u\n",
                  messageType, command, responseRequestId, dataSize);

    if (version != VERSION || dataSize > 256) {
        Serial.printf("[TCP-RX ERROR] Version mismatch(%d) or payload too large(%d)\n", version, dataSize);
        return false;
    }

    uint8_t payload[256] = {0};
    if (dataSize > 0 && !receiveExact(payload, dataSize, 5000)) {
        Serial.println("[TCP-RX ERROR] Payload receive timeout.");
        return false;
    }

    // 1. 비동기 AI 슬롯 이벤트 처리 (CMD: 0x0022)
    if (messageType == EVENT && command == AI_SLOT_FLAGS) {
        if (dataSize == AI_SLOT_COUNT) {
            for (uint8_t i = 0; i < AI_SLOT_COUNT; i++) {
                aiSlotFlags[i] = (payload[i] == 1);
            }
            Serial.println("[TCP-RX] AI Slot Flags updated successfully.");
            return true;
        }
        return false;
    }

    if (messageType != RESPONSE) return false;

    // 2. 입차 차량 정보 응답 수신 (CMD: 0x0020, 57바이트 완벽 대응)
    if (command == ENTRY_RECOGNIZED) {
        uint8_t status = payload[0];
        uint32_t rawEventId = readUint32(&payload[1]);

        Serial.printf("[TCP-RX CHECK] ENTRY_RECOGNIZED -> Status: 0x%02X (%d), Raw EventId: %u, DataSize: %u\n", 
                      status, status, rawEventId, dataSize);

        // Status가 0(STATUS_OK)이거나 혹시 모를 ASCII '0'(48)인 경우 모두 허용
        // 최소 5바이트(Status 1B + EventID 4B) 이상이면 파싱 진행
        if ((status == STATUS_OK || status == 0x30) && dataSize >= 5) {
            // 만약 서버에서 EventID를 0으로 주었더라도 식별 가능하도록 1로 보정
            entryEventId = (rawEventId == 0) ? 1 : rawEventId;
            entryConfirmationSent = false;

            if (dataSize >= 37) {
                copyFixedText(recognizedVehicle, &payload[5], VEHICLE_NUMBER_SIZE);
            }
            if (dataSize >= 57) {
                copyFixedText(recognizedTime, &payload[5 + VEHICLE_NUMBER_SIZE], TIMESTAMP_SIZE);
            }

            Serial.printf("[TCP-RX SUCCESS] 입차 정보 파싱 성공! -> EventId=%u, Vehicle=%s, Time=%s\n", 
                          entryEventId, recognizedVehicle, recognizedTime);
        } else {
            Serial.printf("[TCP-RX REJECT] Status 에러 응답 수신 -> Status: %d\n", status);
        }
    } 
    // 3. 출차 정산 정보 응답 수신 (CMD: 0x0021)
    else if (command == EXIT_FARE_INFO) {
        uint8_t status = payload[0];
        uint32_t rawEventId = readUint32(&payload[1]);

        Serial.printf("[TCP-RX CHECK] EXIT_FARE_INFO -> Status: 0x%02X (%d), Raw EventId: %u, DataSize: %u\n", 
                      status, status, rawEventId, dataSize);

        if ((status == STATUS_OK || status == 0x30) && dataSize >= 5) {
            exitEventId = (rawEventId == 0) ? 1 : rawEventId;
            exitConfirmationSent = false;

            if (dataSize >= 37) {
                copyFixedText(exitVehicle, &payload[5], VEHICLE_NUMBER_SIZE);
            }
            if (dataSize >= 41) {
                exitFee = readUint32(&payload[5 + VEHICLE_NUMBER_SIZE]);
            }

            Serial.printf("[TCP-RX SUCCESS] 출차 정보 파싱 성공! -> EventId=%u, Vehicle=%s, Fee=%u\n", 
                          exitEventId, exitVehicle, exitFee);
        } else {
            Serial.printf("[TCP-RX REJECT] Status 에러 응답 수신 -> Status: %d\n", status);
        }
    }
    // 4. 통과 완료 확정 응답 수신 (CMD: 0x0011, 0x0013)
    else if ((command == SENSOR2_ENTRY_CONFIRMED || command == SENSOR4_EXIT_CONFIRMED) && dataSize >= 1) {
        if (payload[0] == STATUS_OK || payload[0] == 0x30) {
            if (command == SENSOR2_ENTRY_CONFIRMED) {
                entryEventId = 0;
                entryConfirmationSent = false;
                Serial.println("[TCP-RX] SENSOR2 Confirmed OK");
            } else {
                exitEventId = 0;
                exitConfirmationSent = false;
                Serial.println("[TCP-RX] SENSOR4 Confirmed OK");
            }
        }
    }
    return true;
}

// ============================================================
// 센서 이벤트 요청
// ============================================================
void sendSensor1Entry() {
    requestId++;
    sendPacket(REQUEST, SENSOR1_ENTRY, requestId, nullptr, 0);
}

void sendSensor2EntryConfirmed() {
    uint8_t payload[4] = {0};
    writeUint32(payload, entryEventId);
    requestId++;
    sendPacket(REQUEST, SENSOR2_ENTRY_CONFIRMED, requestId, payload, 4);
    entryConfirmationSent = true;
}

void sendSensor3Exit() {
    requestId++;
    sendPacket(REQUEST, SENSOR3_EXIT, requestId, nullptr, 0);
}

void sendSensor4ExitConfirmed() {
    uint8_t payload[4] = {0};
    writeUint32(payload, exitEventId);
    requestId++;
    sendPacket(REQUEST, SENSOR4_EXIT_CONFIRMED, requestId, payload, 4);
    exitConfirmationSent = true;
}

// ============================================================
// 연결 관리
// ============================================================
void connectWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi connected");
}

bool connectServer() {
    serverClient.stop();
    Serial.printf("[TCP] Connecting to server %s:%d ...\n", SERVER_IP, SERVER_PORT);
    if (serverClient.connect(SERVER_IP, SERVER_PORT)) {
        Serial.println("[TCP] >>> Connected to Server successfully!");
        return true;
    }
    Serial.println("[TCP] >>> Connection Failed!");
    return false;
}