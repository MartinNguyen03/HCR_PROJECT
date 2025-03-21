import rospy
import cv2
cap = cv2.VideoCapture(0)
out = cv2.VideoWriter('/dev/video1', cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    out.write(frame)
