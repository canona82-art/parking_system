#ifndef PARKING_H
#define PARKING_H

#include <Arduino.h>
#include <WiFi.h>


// Wi-Fi와 메인 서버 설정
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* SERVER_IP;
extern const uint16_t SERVER_PORT;

// 초음파 센서 핀과 감지 거리
extern const uint8_t SENSOR1_TRIG_PIN;
extern const uint8_t SENSOR1_ECHO_PIN;
extern const uint8_t SENSOR2_TRIG_PIN;
extern const uint8_t SENSOR2_ECHO_PIN;
extern const uint8_t SENSOR3_TRIG_PIN;
extern const uint8_t SENSOR3_ECHO_PIN;
extern const uint8_t SENSOR4_TRIG_PIN;
extern const uint8_t SENSOR4_ECHO_PIN;
extern const float SENSOR_DETECT_DISTANCE_CM;

// 메인 서버와 연결하는 TCP 클라이언트
extern WiFiClient serverClient;

// 공통 바이너리 프로토콜
extern const uint8_t VERSION;
extern const uint8_t REQUEST;
extern const uint8_t RESPONSE;
extern const uint8_t EVENT;

extern const uint16_t SENSOR1_ENTRY;
extern const uint16_t SENSOR2_ENTRY_CONFIRMED;
extern const uint16_t SENSOR3_EXIT;
extern const uint16_t SENSOR4_EXIT_CONFIRMED;
extern const uint16_t ENTRY_RECOGNIZED;
extern const uint16_t EXIT_FARE_INFO;
extern const uint16_t AI_SLOT_FLAGS;

extern const uint8_t STATUS_OK;
extern const uint8_t STATUS_ERROR;
extern const uint8_t VEHICLE_NUMBER_SIZE;
extern const uint8_t TIMESTAMP_SIZE;
extern const uint8_t HEADER_SIZE;
extern const uint8_t AI_SLOT_COUNT;

// 통신과 센서 처리에서 공유하는 상태
extern uint32_t requestId;
extern uint32_t entryEventId;
extern uint32_t exitEventId;
extern char recognizedVehicle[];
extern char recognizedTime[];
extern char exitVehicle[];
extern uint32_t exitFee;
extern bool previousSensor1Detected;
extern bool previousSensor2Detected;
extern bool previousSensor3Detected;
extern bool previousSensor4Detected;
extern bool entryConfirmationSent;
extern bool exitConfirmationSent;
extern bool aiSlotFlags[];

// Big Endian 변환
void writeUint16(uint8_t* buffer, uint16_t value);
void writeUint32(uint8_t* buffer, uint32_t value);
uint32_t readUint32(const uint8_t* buffer);

// 초음파 센서
float readDistanceCm(uint8_t trigPin, uint8_t echoPin);
bool isUltrasonicTriggered(
    uint8_t trigPin,
    uint8_t echoPin,
    bool& previousDetected
);

// 네트워크 패킷
bool sendPacket(
    uint8_t messageType,
    uint16_t command,
    uint32_t requestIdValue,
    const uint8_t* payload,
    uint32_t payloadSize
);
bool receiveExact(
    uint8_t* buffer,
    size_t size,
    uint32_t timeoutMs = 5000
);
void copyFixedText(
    char* destination,
    const uint8_t* source,
    size_t size
);
bool receivePacket();

// 센서 이벤트
void sendSensor1Entry();
void sendSensor2EntryConfirmed();
void sendSensor3Exit();
void sendSensor4ExitConfirmed();

// 연결
void connectWiFi();
bool connectServer();

#endif
