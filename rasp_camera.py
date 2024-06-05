import cv2

# Set video resolution
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720

# Set video duration (in seconds)
VIDEO_DURATION = 10

# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output_video.avi', fourcc, 20.0, (VIDEO_WIDTH, VIDEO_HEIGHT))

# Record video
start_time = cv2.getTickCount()
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret:
        out.write(frame)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # Check for video duration
        if (cv2.getTickCount() - start_time) / cv2.getTickFrequency() > VIDEO_DURATION:
            break
    else:
        break

# Release resources
cap.release()
out.release()
cv2.destroyAllWindows()