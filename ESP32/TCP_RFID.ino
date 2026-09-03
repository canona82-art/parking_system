#include <WiFi.h>
#include <FastLED.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <MFRC522.h>
#include "parking.h"

// ==========================================
// 하드웨어 핀 및 상수 설정 (핀 스왑 완료 기준)
// ==========================================
#define I2C_SDA_PIN      21
#define I2C_SCL_PIN      22
#define SERVO1_PIN       25 // 입차 차단기
#define SERVO2_PIN       33 // 출차 차단기

#define LED1_PIN         2  // 1번 배열 LED
#define LED2_PIN         4  // 2번 배열 LED (GPIO 4)
#define LDR_PIN          32
#define SW_PIN           16
#define BUZZER_PIN       15 // 부저 핀 (GPIO 15)

#define RFID_SS_PIN      5
#define RFID_RST_PIN     17

#define NUM_LEDS1        16
#define NUM_LEDS2        10

// [수정됨] 잔액을 저장할 RFID 블록 번호 정의 (MIFARE 카드의 4번 블록 사용)
#define TAG_BALANCE_BLOCK 4 

// ==========================================
// 전역 객체 및 상태 변수
// ==========================================
CRGB leds1[NUM_LEDS1];
CRGB leds2[NUM_LEDS2];
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1;
Servo servo2;
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);

// [수정됨] RFID 인증을 위한 기본 키 선언
MFRC522::MIFARE_Key defaultKey; 

// 시나리오 상태 정의
enum SystemState { IDLE, RECOGNIZING, ACTION_WAIT, PASSING };
SystemState entryState = IDLE;
SystemState exitState = IDLE;

// 논블로킹 및 디바운스 타이머
unsigned long buzzerStartTime = 0;
bool isBuzzerOn = false;

// 센서 2, 4 통과 감지용 타이머
unsigned long sensor2LastSeen = 0;
unsigned long sensor4LastSeen = 0;

// AI 인식 응답 대기 시간초과 복구용 타이머
unsigned long entryRecognizingStart = 0;
unsigned long exitRecognizingStart = 0;
const unsigned long recognizingTimeout = 7000; // 7초

bool manualMode = false;
int lastSwState = HIGH;
int debouncedSwState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

int lastAvailableSpaces = -1;

void printUid()
{
    Serial.print("UID: ");

    for (byte index = 0; index < rfid.uid.size; index++)
    {
        if (rfid.uid.uidByte[index] < 0x10)
        {
            Serial.print("0");
        }

        Serial.print(rfid.uid.uidByte[index], HEX);

        if (index + 1 < rfid.uid.size)
        {
            Serial.print(":");
        }
    }

    Serial.println();
}

uint32_t readUint32BigEndian(const byte* data)
{
    return ((uint32_t)data[0] << 24)
        | ((uint32_t)data[1] << 16)
        | ((uint32_t)data[2] << 8)
        | data[3];
}

void writeUint32BigEndian(byte* data, uint32_t value)
{
    data[0] = (value >> 24) & 0xFF;
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >> 8) & 0xFF;
    data[3] = value & 0xFF;
}

bool authenticateBalanceBlock()
{
    MFRC522::StatusCode status = rfid.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A,
        TAG_BALANCE_BLOCK,
        &defaultKey,
        &(rfid.uid)
    );

    if (status != MFRC522::STATUS_OK)
    {
        Serial.print("RFID authentication failed: ");
        Serial.println(rfid.GetStatusCodeName(status));
        return false;
    }

    return true;
}

bool readTagBalance(uint32_t& balance)
{
    if (!authenticateBalanceBlock())
    {
        return false;
    }

    byte buffer[18] = {0};
    byte bufferSize = sizeof(buffer);

    MFRC522::StatusCode status = rfid.MIFARE_Read(
        TAG_BALANCE_BLOCK,
        buffer,
        &bufferSize
    );

    if (status != MFRC522::STATUS_OK)
    {
        Serial.print("RFID read failed: ");
        Serial.println(rfid.GetStatusCodeName(status));
        return false;
    }

    balance = readUint32BigEndian(buffer);
    return true;
}

bool writeTagBalance(uint32_t balance)
{
    byte data[16] = {0};
    writeUint32BigEndian(data, balance);

    if (!authenticateBalanceBlock())
    {
        return false;
    }

    MFRC522::StatusCode status = rfid.MIFARE_Write(
        TAG_BALANCE_BLOCK,
        data,
        sizeof(data)
    );

    if (status != MFRC522::STATUS_OK)
    {
        Serial.print("RFID write failed: ");
        Serial.println(rfid.GetStatusCodeName(status));
        return false;
    }

    return true;
}

// 카드 잔액을 읽고 출차 요금을 차감한다.
bool processExitFee()
{
    uint32_t balance = 0;

    if (!readTagBalance(balance))
    {
        return false;
    }

    Serial.print("Card balance: ");
    Serial.println(balance);

    // 요금과 잔액이 같은 경우도 결제 가능하도록 처리한다.
    if (balance < exitFee)
    {
        Serial.println("Insufficient balance");
        return false;
    }

    uint32_t remainingBalance = balance - exitFee;

    if (!writeTagBalance(remainingBalance))
    {
        return false;
    }

    Serial.print("Payment accepted. Remaining balance: ");
    Serial.println(remainingBalance);
    return true;
}

// ==========================================
// 헬퍼 함수
// ==========================================
// 한글(UTF-8) 제거 및 ASCII 추출
String filterKorean(const char* input) {
    String result = "";
    int len = strlen(input);

    for (int i = 0; i < len; i++) {
        if ((input[i] & 0x80) == 0) {
            result += input[i];
        } else {
            result += " ";
            while (i + 1 < len && (input[i + 1] & 0xC0) == 0x80) {
                i++;
            }
        }
    }

    result.replace("  ", " ");
    result.trim();
    return result;
}

// 서보 모터 천천히 닫기 (하강 속도 조절)
void closeServoSlowly(Servo& targetServo) {
    for (int angle = 90; angle >= 0; angle--) {
        targetServo.write(angle);

        if (serverClient.available()) {
            receivePacket(); // 통신 누락 방지
        }

        delay(15); // 하강 속도 딜레이
    }
}

// ==========================================
// 디스플레이 및 액추에이터 업데이트 함수
// ==========================================
void updateDefaultLCD() {
    int parkedCount = 0;

    for (int i = 0; i < AI_SLOT_COUNT; i++) {
        if (aiSlotFlags[i]) parkedCount++;
    }

    int availableSpaces = AI_SLOT_COUNT - parkedCount;

    if (availableSpaces != lastAvailableSpaces &&
        entryState == IDLE &&
        exitState == IDLE) {

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Parking Space");
        lcd.setCursor(0, 1);
        lcd.printf("%d / %d", availableSpaces, AI_SLOT_COUNT);

        lastAvailableSpaces = availableSpaces;
    }
}

void updateLEDs() {
    fill_solid(leds1, NUM_LEDS1, CRGB::Black);
    fill_solid(leds2, NUM_LEDS2, CRGB::Black);

    // AI 주차면 매핑
    leds1[12] = aiSlotFlags[0] ? CRGB::Red : CRGB::Blue;
    leds1[6]  = aiSlotFlags[1] ? CRGB::Red : CRGB::Blue;
    leds1[0]  = aiSlotFlags[2] ? CRGB::Red : CRGB::Blue;

    leds2[8]  = aiSlotFlags[3] ? CRGB::Red : CRGB::Blue;
    leds2[1]  = aiSlotFlags[4] ? CRGB::Red : CRGB::Blue;
    leds2[0]  = aiSlotFlags[5] ? CRGB::Red : CRGB::Blue;

    // 조도 센서 조명 연동
    analogReadResolution(8);
    int ldrValue = analogRead(LDR_PIN);

    if (ldrValue >= 100) {
        leds1[13] = CRGB::White; 
        leds1[14] = CRGB::White; 
        leds1[15] = CRGB::White; 
    }

    FastLED.show();
}

void handleManualSwitch() {
    int reading = digitalRead(SW_PIN);

    if (reading != lastSwState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != debouncedSwState) {
            debouncedSwState = reading;

            if (debouncedSwState == LOW) {
                manualMode = !manualMode;

                if (manualMode) {
                    servo1.write(90);
                    servo2.write(90);

                    lcd.clear();
                    lcd.setCursor(0, 0);
                    lcd.print("MANUAL MODE ON");
                    lcd.setCursor(0, 1);
                    lcd.print("Gates Opened");

                    Serial.println("[비상] 수동 모드 ON - 차단기 전체 개방");
                } else {
                    servo1.write(0);
                    servo2.write(0);

                    lastAvailableSpaces = -1;
                    entryState = IDLE;
                    exitState = IDLE;

                    previousSensor1Detected = false;
                    previousSensor2Detected = false;
                    previousSensor3Detected = false;
                    previousSensor4Detected = false;

                    entryRecognizingStart = 0;
                    exitRecognizingStart = 0;

                    Serial.println("[비상] 수동 모드 OFF - 자동 모드 복귀");
                }
            }
        }
    }

    lastSwState = reading;
}

// ==========================================
// 셋업 및 메인 루프
// ==========================================
void setup() {
    Serial.begin(115200);

    pinMode(SW_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    lcd.init();
    lcd.backlight();
    lcd.print("System Starting");

    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo1.write(0);
    servo2.write(0);

    FastLED.addLeds<WS2812B, LED1_PIN, GRB>(leds1, NUM_LEDS1);
    FastLED.addLeds<WS2812B, LED2_PIN, GRB>(leds2, NUM_LEDS2);
    FastLED.setBrightness(50);

    fill_solid(leds1, NUM_LEDS1, CRGB::Blue);
    fill_solid(leds2, NUM_LEDS2, CRGB::Blue);
    FastLED.show();

    SPI.begin(18, 19, 23, 5);
    rfid.PCD_Init();

    // [수정됨] RFID 인증을 위한 defaultKey(공장 출하 상태) 초기화
    for (byte i = 0; i < 6; i++) {
        defaultKey.keyByte[i] = 0xFF;
    }

    pinMode(SENSOR1_TRIG_PIN, OUTPUT);
    pinMode(SENSOR1_ECHO_PIN, INPUT);
    pinMode(SENSOR2_TRIG_PIN, OUTPUT);
    pinMode(SENSOR2_ECHO_PIN, INPUT);
    pinMode(SENSOR3_TRIG_PIN, OUTPUT);
    pinMode(SENSOR3_ECHO_PIN, INPUT);
    pinMode(SENSOR4_TRIG_PIN, OUTPUT);
    pinMode(SENSOR4_ECHO_PIN, INPUT);

    digitalWrite(SENSOR1_TRIG_PIN, LOW);
    digitalWrite(SENSOR2_TRIG_PIN, LOW);
    digitalWrite(SENSOR3_TRIG_PIN, LOW);
    digitalWrite(SENSOR4_TRIG_PIN, LOW);

    connectWiFi();
    connectServer();

    lastAvailableSpaces = -1;
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }

    if (!serverClient.connected()) {
        connectServer();
        delay(1000);
        return;
    }

    if (serverClient.available()) {
        if (receivePacket()) delay(10);
    }

    updateLEDs();
    handleManualSwitch();

    if (manualMode) {
        delay(30);
        return;
    }

    if (isBuzzerOn && (millis() - buzzerStartTime >= 2000)) {
        digitalWrite(BUZZER_PIN, LOW);
        isBuzzerOn = false;
    }

    // ==========================================
    // [입차 시나리오]
    // ==========================================
    if (entryState == IDLE) {
        if (isUltrasonicTriggered(
                SENSOR1_TRIG_PIN,
                SENSOR1_ECHO_PIN,
                previousSensor1Detected)) {

            Serial.println("\n[SENSOR-1] >>> 입구 차량 감지됨! (SENSOR1_ENTRY 전송)");
            sendSensor1Entry();
            entryRecognizingStart = millis();
            entryState = RECOGNIZING;
        }
    }
    else if (entryState == RECOGNIZING) {
        if (entryEventId != 0 && !entryConfirmationSent) {
            String filteredPlate = filterKorean(recognizedVehicle);

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(filteredPlate);
            lcd.setCursor(0, 1);
            lcd.print(recognizedTime);

            Serial.println("[GATE-1] 번호판 인식 완료 -> 입구 차단기 개방 (90도)");
            servo1.write(90);

            entryRecognizingStart = 0;
            entryState = PASSING;
        }
        else if (millis() - entryRecognizingStart >= recognizingTimeout) {
            Serial.println("[ENTRY TIMEOUT] AI 응답 시간 초과 (7초 경과) -> 입차 감지 상태 강제 초기화");

            entryEventId = 0;
            entryConfirmationSent = false;
            previousSensor1Detected = false;
            entryRecognizingStart = 0;
            entryState = IDLE;
        }
    }
    else if (entryState == PASSING) {
        float dist = readDistanceCm(
            SENSOR2_TRIG_PIN,
            SENSOR2_ECHO_PIN
        );

        if (dist > 0.0f && dist < 10.0f) {
            if (!previousSensor2Detected) {
                Serial.printf("[SENSOR-2] 차 진입 통과 중 (측정 거리: %.1f cm)\n", dist);
            }
            previousSensor2Detected = true;
            sensor2LastSeen = millis();
        }
        else if (previousSensor2Detected &&
                 (millis() - sensor2LastSeen > 1500)) {

            Serial.println("[SENSOR-2] >>> 입차 완료 감지! (SENSOR2_ENTRY_CONFIRMED 전송)");
            sendSensor2EntryConfirmed();
            closeServoSlowly(servo1);

            lastAvailableSpaces = -1;
            entryState = IDLE;
            previousSensor2Detected = false;
            sensor2LastSeen = 0;
        }
    }

    // ==========================================
    // [출차 시나리오]
    // ==========================================
    if (exitState == IDLE) {
        if (isUltrasonicTriggered(
                SENSOR3_TRIG_PIN,
                SENSOR3_ECHO_PIN,
                previousSensor3Detected)) {

            Serial.println("\n[SENSOR-3] >>> 출구 차량 감지됨! (SENSOR3_EXIT 전송 & 부저 2초)");
            sendSensor3Exit();
            exitRecognizingStart = millis();

            digitalWrite(BUZZER_PIN, HIGH);
            buzzerStartTime = millis();
            isBuzzerOn = true;
            exitState = RECOGNIZING;
        }
    }
    else if (exitState == RECOGNIZING) {
        if (exitEventId != 0 && !exitConfirmationSent) {
            String filteredPlate = filterKorean(exitVehicle);

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(filteredPlate);
            lcd.setCursor(0, 1);
            lcd.printf("Total:%u KR", exitFee);

            Serial.printf("[EXIT-INFO] 요금 수신 완료 (차량: %s / 요금: %u원) -> RFID 결제 대기\n", exitVehicle, exitFee);

            exitRecognizingStart = 0;
            exitState = ACTION_WAIT;
        }
        else if (millis() - exitRecognizingStart >= recognizingTimeout) {
            Serial.println("[EXIT TIMEOUT] AI 응답 시간 초과 (7초 경과) -> 출차 감지 상태 강제 초기화");

            exitEventId = 0;
            exitConfirmationSent = false;
            previousSensor3Detected = false;
            exitRecognizingStart = 0;
            exitState = IDLE;
        }
    }
    else if (exitState == ACTION_WAIT) {
        if (rfid.PICC_IsNewCardPresent() &&
            rfid.PICC_ReadCardSerial()) {

            printUid();

            bool paymentAccepted = processExitFee();
            Serial.print("payment result: ");
            Serial.println(paymentAccepted ? "true" : "false");

            if(paymentAccepted){
                Serial.println("[RFID] 결제 카드 인식 완료 -> 출구 차단기 개방 (90도)");
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Payment");
                lcd.setCursor(0, 1);
                lcd.print("Completed");
                servo2.write(90);

                exitState = PASSING;
            }
            else{
                Serial.println("[RFID] 결제 실패");
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Payment");
                lcd.setCursor(0, 1);
                lcd.print("Failed"); // [수정됨] prdint -> print 로 수정
            }
        
            rfid.PICC_HaltA();
            rfid.PCD_StopCrypto1();
        }
    }
    else if (exitState == PASSING) {
        float dist = readDistanceCm(
            SENSOR4_TRIG_PIN,
            SENSOR4_ECHO_PIN
        );

        if (dist > 0.0f && dist < 10.0f) {
            if (!previousSensor4Detected) {
                Serial.printf("[SENSOR-4] 차 출차 통과 중 (측정 거리: %.1f cm)\n", dist);
            }
            previousSensor4Detected = true;
            sensor4LastSeen = millis();
        }
        else if (previousSensor4Detected &&
                 (millis() - sensor4LastSeen > 1500)) {

            Serial.println("[SENSOR-4] >>> 출차 완료 감지! (SENSOR4_EXIT_CONFIRMED 전송)");
            sendSensor4ExitConfirmed();
            closeServoSlowly(servo2);

            lastAvailableSpaces = -1;
            exitState = IDLE;
            previousSensor4Detected = false;
            sensor4LastSeen = 0;
        }
    }

    updateDefaultLCD();
    delay(30);
}