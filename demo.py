import cv2
import sys

stopCascPath = sys.argv[1]
stopCascade = cv2.CascadeClassifier(stopCascPath)

limitCascPath = sys.argv[2]
limitCascade = cv2.CascadeClassifier(limitCascPath)

video_capture = cv2.VideoCapture(0)

while True:
    # Capture frame-by-frame
    ret, frame = video_capture.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    stops = stopCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
    )

    limits = limitCascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30),
    )

    for (x, y, w, h) in stops:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

    for (x, y, w, h) in limits:
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    cv2.imshow('Video', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
