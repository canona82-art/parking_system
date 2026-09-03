# 주차 야호~
### 1. Smart Parking Management System
> 차량, 번호판 인식과 ESP32 센서 및 FRID를 활용해 입출차, 요금 정산, 주차 관리를 자동화한 스마트 주차 시스템

<img width="4000" height="2776" alt="SP" src="https://github.com/user-attachments/assets/8de31d9c-90a5-4333-aa51-a93ca61c86d0" />


### 2. 프로젝트 개요
> 초음파 센서로 차량 진입을 감지하고, 카메라 영상과 YOLO, EasyOCR을 이용해 차종 및 번호판 인식
> 인식 결과는 MySQL에 저장되며, 주차 위치 관리, 장애인 차량 할인, RFID 요금 결제, 차단기 제어까지 하나의 시스템으로 연동


### 3. 주요 기능
| 기능 | 설명 |
| ------ | ------ |
| 자동 입출차 | 초음파 센서 감지 후 카메라 이미지 전송 |
| 차량 인식 | YOLO로 차량 종류와 번호판 영역 탐지 |
| 번호판 OCR | EasyOCR과 후처리로 한국 번호판 인식 |
| 주차면 관리 | A1-A3, B1-B3 총 6개 주차면 상태 관리 |
| 자동 요금 계산 | 주차 시간과 차량 속성에 따라 요금 계산 |
| RFID 결제 | 카드 잔액 확인 및 출차 요금 차감 |
| 차단기 제어 | ESP32 서보모터로 입·출차 차단기 제어 |
| 관리자 GUI | 차량, 회원, 주차 상태 및 카메라 관리 |
| 장애인 구역 관리 | 미등록 차량 주차 시 경고 표시 |

### 4. 시스템 구성도
#### 4-1. 하드웨어 아키텍쳐
<img width="831" height="660" alt="Screenshot from 2026-09-03 17-20-12" src="https://github.com/user-attachments/assets/1eb9a07e-40e7-4f5f-a4b6-d11c10c50759" />

#### 4-2. 소프트웨어 아키텍쳐
<img width="612" height="474" alt="Screenshot from 2026-09-03 17-19-44" src="https://github.com/user-attachments/assets/102aa3d5-bda5-43a9-9657-15b8ee33ad94" />

#### 4-3. 포트 구성
| 포트 | 방식 | 용도 |
| --- | --- | --- |
| 5000 | TCP | ESP32 <-> 메인 서버 |
| 5001 | UDP | 입차 카메라 영상 |
| 5002 | UDP | 출차 카메라 영상 |
| 6000 | TCP | AI 서버 <-> 메인 서버 |


### 5. 동작 시나리오
#### 5-1. 입차 시나리오
<img width="792" height="672" alt="entry_SD" src="https://github.com/user-attachments/assets/c8ec158f-c273-4e22-b936-05a929918ecf" />

#### 5-2. 출차 시나리오
<img width="792" height="582" alt="exit_SD" src="https://github.com/user-attachments/assets/4ff12002-41a6-43e0-8c57-a750e52fc832" />


### 6. 기술 스택
| 구분 | 기술 |
| --- | --- |
| Main server | Python, PyQt, MySQL |
| AI | YOLO, EasyOCR, PyTorch, OpenCV |
| Embedded | ESP32, Servo Motor, RFID M522, Ultrasonic Sensor, LCD Display |
| Network | TCP, UDP, Python, Binary Protocol, Multi Thread |


### 7. 하드웨어 구성
<img width="741" height="415" alt="Screenshot from 2026-09-03 17-33-57" src="https://github.com/user-attachments/assets/f20bc24e-6803-482e-8875-7df2db5de659" />


<img width="755" height="1081" alt="Screenshot from 2026-09-03 17-33-36" src="https://github.com/user-attachments/assets/3f1cbdbf-5fee-4fe2-a7c9-a159f8f065b1" />


### 8. 디렉토리 구조
  ```text
  iot-repo-1/
  ├── AI/
  │   ├── ai_server.py       # AI 통신 클라이언트
  │   ├── ocr_adapter.py     # YOLO + EasyOCR 파이프라인
  │   ├── network_basic.py   # 공통 바이너리 프로토콜
  │   └── best.pt            # 학습된 YOLO 가중치
  ├── ESP32/
  │   ├── TCP_RFID.ino       # ESP32 메인 펌웨어
  │   ├── smart_parking.cpp  # 센서 및 통신 구현
  │   └── parking.h          # 상수와 함수 선언
  ├── camera_sender.py       # 입·출차 카메라 UDP 송신기
  ├── mainServer.py          # 통합 서버 및 관리자 GUI
  └── README.md
  ```


### 9. 팀원 및 역할
| 이름 | 담당 |
| --- | --- |
| 김대현 | 메인서버, DB, GUI |
| 이준성 | 메인서버, DB, GUI |
| 김선우 | AI 모델, OCR |
| 송주한 | ESP32, 하드웨어 |
| 박태준 | 카메라 및 네트워크 통신 |


### 10. 시연 모습
#### 10-1. Over View
https://github.com/user-attachments/assets/d7a5715a-bdbe-41e3-a171-3eb0294c5739

https://github.com/user-attachments/assets/0887aa9b-bbc4-4761-9e36-92321984fec3

https://github.com/user-attachments/assets/dab733ae-5c8d-49fd-afe3-b7fb6f84a5d7


#### 10-2. Front View
https://github.com/user-attachments/assets/0d103d21-2599-4b68-a5f8-4182db57f63c

https://github.com/user-attachments/assets/9268ef86-878c-4fed-84c8-4576744ee5de







