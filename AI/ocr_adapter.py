from __future__ import annotations

import os
import re
import threading
import cv2
import easyocr
import numpy as np
from ultralytics import YOLO

# --------------------------------------------------
# 설정 및 가중치 경로
# --------------------------------------------------
MODEL_PATH = os.getenv(
    "YOLO_MODEL_PATH",
    "/home/jhs/Project/1st_project/runs/detect/car_plate_model/weights/best.pt"
)

VALID_VEHICLES = ['911', 'ionic5', 'jeep', 'morning']
PLATE_PATTERN = re.compile(r'^\d{2,3}[가-힣]\d{4}$')  

# --------------------------------------------------
# AI 모델 싱글톤 (YOLO & EasyOCR)
# --------------------------------------------------
_model = None
_reader = None
_clahe = None
_pipeline_lock = threading.Lock()

def get_ai_resources():
    global _model, _reader, _clahe
    if _model is None or _reader is None:
        with _pipeline_lock:
            if _model is None:
                print(f"[OCR Adapter] Loading YOLO Model from {MODEL_PATH}...")
                _model = YOLO(MODEL_PATH)
            if _reader is None:
                print("[OCR Adapter] Loading EasyOCR GPU Reader...")
                _reader = easyocr.Reader(['ko'], gpu=True)
            if _clahe is None:
                _clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return _model, _reader, _clahe


# --------------------------------------------------
# 번호판 오타 자동 교정 로직 
# --------------------------------------------------
def correct_plate_string(text):
    cleaned = re.sub(r'[^0-9가-힣]', '', text)
    if PLATE_PATTERN.match(cleaned):
        return cleaned
        
    correction_map = {'7': '가', '0': '머', '1': '너', '2': '고', '4': '더', '5': '오'}
    
    if len(cleaned) == 7 and cleaned[:2].isdigit() and cleaned[3:].isdigit():
        mid_char = cleaned[2]
        if mid_char in correction_map:
            return cleaned[:2] + correction_map[mid_char] + cleaned[3:]
            
    if len(cleaned) == 8 and cleaned[:3].isdigit() and cleaned[4:].isdigit():
        mid_char = cleaned[3]
        if mid_char in correction_map:
            return cleaned[:3] + correction_map[mid_char] + cleaned[4:]
            
    return cleaned


def decode_image(image_data) -> np.ndarray:
    if isinstance(image_data, np.ndarray):
        if image_data.size == 0:
            raise ValueError("empty image")
        return image_data.copy()
    if isinstance(image_data, (bytes, bytearray, memoryview)):
        image_array = np.frombuffer(image_data, dtype=np.uint8)
        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("failed to decode image_data")
        return frame
    raise TypeError("image_data must be np.ndarray, bytes, bytearray or memoryview")


# --------------------------------------------------
# 핵심 추론 파이프라인
# --------------------------------------------------
def run_ai_pipeline(image_data):
    frame = decode_image(image_data)
    model, reader, clahe = get_ai_resources()
    h_img, w_img, _ = frame.shape

    results = model(frame, conf=0.25, device='0', verbose=False)

    detected_vehicle_type = "UNKNOWN"
    detected_plate_number = ""

    print("  [DEBUG] === AI 단일 프레임 분석 시작 ===")

    for result in results:
        boxes = result.boxes
        
        # 1단계: 차종 탐색
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            label = model.names[cls_id] 
            
            print(f"  [DEBUG] YOLO 발견 객체: {label} (신뢰도: {conf:.2f})")
            if label in VALID_VEHICLES:
                detected_vehicle_type = label

        # 2단계: 직접 학습된 YOLO 바운딩 박스 기반 OCR 수행
        for box in boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            
            if label == 'number_paddle' and not detected_plate_number:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                pad_w = int((x2 - x1) * 0.05)
                pad_h = int((y2 - y1) * 0.05)
                px1 = max(0, x1 - pad_w)
                py1 = max(0, y1 - pad_h)
                px2 = min(w_img, x2 + pad_w)
                py2 = min(h_img, y2 + pad_h)

                plate_crop = frame[py1:py2, px1:px2]
                if plate_crop.size == 0:
                    continue

                gray_crop = cv2.cvtColor(plate_crop, cv2.COLOR_BGR2GRAY)
                h, w = gray_crop.shape
                
                resized = cv2.resize(gray_crop, (w * 3, h * 3), interpolation=cv2.INTER_CUBIC)
                enhanced = clahe.apply(resized)
                processed = cv2.GaussianBlur(enhanced, (3, 3), 0)

                ocr_res = reader.readtext(
                    processed, 
                    text_threshold=0.3, 
                    low_text=0.3
                )

                # ★ 핵심 추가: 찾은 텍스트 덩어리들을 좌측(X좌표)에서 우측 순서로 강제 정렬
                # ocr_res 구조: [([x, y], ...), text, score]
                ocr_res.sort(key=lambda item: item[0][0][0])

                extracted_text = ""
                for _, text, score in ocr_res:
                    if score >= 0.2:
                        extracted_text += text

                print(f"  [DEBUG] EasyOCR 정렬 추출 텍스트: '{extracted_text}'")

                corrected_text = correct_plate_string(extracted_text)
                print(f"  [DEBUG] 교정 후 텍스트: '{corrected_text}'")

                if PLATE_PATTERN.match(corrected_text):
                    print("  [DEBUG] 🟢 정규식 일치! 번호판 확정.")
                    detected_plate_number = corrected_text
                    break
                else:
                    print("  [DEBUG] ❌ 정규식 불일치. 폐기 처리됨.")

    return detected_vehicle_type, detected_plate_number


# --------------------------------------------------
# 진입점 핸들러
# --------------------------------------------------
def ocr_handler(direction, image_data):
    try:
        frame = decode_image(image_data)
        if direction == "entry":
            cv2.imwrite("debug_entry.jpg", frame)
        elif direction == "exit":
            cv2.imwrite("debug_exit.jpg", frame)
    except Exception as e:
        print(f"[OCR Adapter] 이미지 디코드 에러: {e}")

    print(f"\n[OCR Adapter] YOLOv8 (Custom) + EasyOCR 파이프라인 가동 중...")
    
    vehicle_type, vehicle_number = run_ai_pipeline(image_data)

    if vehicle_type == "UNKNOWN":
        print(f"[OCR Adapter] ❌ 차종 인식 실패 (UNKNOWN). 출입 불가 처리합니다.")
        vehicle_type = ""
        vehicle_number = ""

    if vehicle_number:
        print(f"[OCR Adapter] 최종 결과 확정 -> {vehicle_number} (차종: {vehicle_type})")
    else:
        print(f"[OCR Adapter] 번호판 텍스트를 찾지 못했거나 차종 미확인으로 실패 처리되었습니다.")

    return {
        "vehicle_type": vehicle_type,
        "vehicle_number": vehicle_number
    }