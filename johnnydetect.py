import cv2
import numpy as np
import os
from datetime import datetime
from pymongo import MongoClient
import gridfs

# MongoDB connection setup
client = MongoClient("mongodb://localhost:27017/")
db = client['face_detection_db']
collection = db['detections']
fs = gridfs.GridFS(db)  # Use GridFS to store images

cam = cv2.VideoCapture(0)
cam.set(3, 660)
cam.set(4, 500)
cascade_path = 'haarcascade_frontalface_default.xml'
faceDetector = cv2.CascadeClassifier(cascade_path)

if faceDetector.empty():
    print("Error: Could not load face cascade classifier.")
    exit()

while True:
    retv, frame = cam.read()
    if not retv:
        print("Failed to grab frame")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = faceDetector.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
    print(f"Faces found: {len(faces)}")  # Debug statement

    detection_data = []  # List to store detection data

    # Draw rectangle around the faces and prepare the face data for MongoDB
    for (x, y, w, h) in faces:
        frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 63), 2)
        print(f"Face found at X:{x}, Y:{y}, Width:{w}, Height:{h}")  # Debug statement

        # Extract the face region
        face_image = frame[y:y+h, x:x+w]
        # Convert to RGB (OpenCV uses BGR by default)
        face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
        # Convert to binary format
        _, buffer = cv2.imencode('.jpg', face_image)
        face_binary = buffer.tobytes()

        # Prepare detection data
        detection = {
            "timestamp": datetime.now(),
            "coordinates": {
                "x": int(x),
                "y": int(y),
                "width": int(w),
                "height": int(h)
            },
            "face_image": fs.put(face_binary, filename=f"face_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")
        }
        detection_data.append(detection)

    # Insert detection data into MongoDB
    if detection_data:
        collection.insert_many(detection_data)
        print(f"Inserted {len(detection_data)} detections into MongoDB")

    # Display the output
    cv2.imshow('Webcam', frame)

    close = cv2.waitKey(1) & 0xFF
    if close == 27 or close == ord('n'):
        break

cam.release()
cv2.destroyAllWindows()