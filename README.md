# help

import cv2
import torch
import time
import serial

# YOLOv5 모델 로드
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # YOLOv5 small 모델
model.classes = [55]  # COCO 데이터셋에서 딸기 클래스 ID

# 카메라 설정
cap = cv2.VideoCapture(0)

# 로봇 제어 설정 (COM3 포트와 9600 baud rate는 필요에 따라 변경)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

def control_robot_arm(x, y, z):
    command = f"{x},{y},{z}\n"
    ser.write(command.encode())

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # YOLO 모델로 이미지 예측
    results = model(frame)

    # 탐지된 객체 중 딸기만 처리
    for detection in results.xyxy[0]:
        x1, y1, x2, y2, confidence, cls = detection
        if int(cls) == 55:  # 딸기 클래스
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            # 딸기 위치에 사각형 그리기
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, 'Strawberry', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # 로봇 팔 제어 (딸기의 중심 좌표를 사용)
            control_robot_arm(center_x, center_y, 10)  # z 값은 예제로 설정

    cv2.imshow("Image", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC 키
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
