import cv2
import mediapipe as mp
import serial
import time
import math

# Initialize serial
try:
    ser = serial.Serial('COM10', 9600, timeout=1)
    time.sleep(2)
except:
    print("Serial port error.")
    exit()

# MediaPipe setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Webcam setup
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Landmark indices
finger_joints = [
    (2, 3, 4),   # Thumb
    (5, 6, 8),   # Index
    (9, 10, 12), # Middle
    (13, 14, 16),# Ring
    (17, 18, 20) # Pinky
]

# Smoothing
prev_angles = [90] * 5
smoothing_factor = 0.2

# Calculate angle between three points
def get_angle(a, b, c):
    ba = [a.x - b.x, a.y - b.y]
    bc = [c.x - b.x, c.y - b.y]
    dot = ba[0]*bc[0] + ba[1]*bc[1]
    mag_ba = math.hypot(ba[0], ba[1])
    mag_bc = math.hypot(bc[0], bc[1])
    if mag_ba * mag_bc == 0:
        return 90
    cos_angle = dot / (mag_ba * mag_bc)
    angle = math.acos(min(1, max(-1, cos_angle)))
    return int(math.degrees(angle))  # Return angle in degrees

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)
    angles = [90] * 5

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            for i, (a, b, c) in enumerate(finger_joints):
                angle = get_angle(hand_landmarks.landmark[a], hand_landmarks.landmark[b], hand_landmarks.landmark[c])
                angle = max(0, min(180, angle))  # Clamp
                # Invert if needed (0° = fully bent, 180° = fully open)
                angle = 180 - angle
                angles[i] = int((1 - smoothing_factor) * prev_angles[i] + smoothing_factor * angle)
                prev_angles[i] = angles[i]

        serial_data = f"${','.join(map(str, angles))}\n"
        ser.write(serial_data.encode())

    cv2.imshow("Robotic Hand Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    time.sleep(0.02)

cap.release()
cv2.destroyAllWindows()
hands.close()
ser.close()
