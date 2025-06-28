import cv2
import mediapipe as mp
import serial
import time

# Initialize serial communication
ser = serial.Serial('COM10', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Initialize webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Width
cap.set(4, 480)  # Height

# Finger landmark indices (tip and pip joints)
finger_tips = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky tips
finger_pips = [3, 6, 10, 14, 18]  # PIP joints for comparison

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.flip(frame, 1)  # Mirror frame
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process hand landmarks
    results = hands.process(rgb_frame)
    angles = [90, 90, 90, 90, 90]  # Default servo angles (neutral)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            landmarks = hand_landmarks.landmark

            for i, tip in enumerate(finger_tips):
                tip_y = landmarks[tip].y
                pip_y = landmarks[finger_pips[i]].y
                # If tip is below pip, finger is closed (0°); else open (180°)
                # Note: Arduino reverses all except middle, so we send raw 0/180
                angles[i] = 0 if tip_y > pip_y else 180

        # Send angles to Arduino (format: $angle1,angle2,angle3,angle4,angle5)
        serial_data = f"${angles[0]},{angles[1]},{angles[2]},{angles[3]},{angles[4]}\n"
        ser.write(serial_data.encode())

    # Display frame
    cv2.imshow('Hand Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release() 
cv2.destroyAllWindows()
hands.close()
ser.close()