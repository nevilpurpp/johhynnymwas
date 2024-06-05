import picamera
import time

# Set video resolution and framerate
VIDEO_RESOLUTION = (1280, 720)
VIDEO_FRAMERATE = 30

# Set video duration (in seconds)
VIDEO_DURATION = 10

# Function to record video
def record_video(output_file, duration):
    with picamera.PiCamera() as camera:
        camera.resolution = VIDEO_RESOLUTION
        camera.framerate = VIDEO_FRAMERATE
        camera.start_recording(output_file)
        camera.wait_recording(duration)
        camera.stop_recording()

# Main function
def main():
    output_file = "output_video.h264"  # Set the output file name
    print(f"Recording video to {output_file} for {VIDEO_DURATION} seconds...")
    record_video(output_file, VIDEO_DURATION)
    print("Video recording complete.")

if __name__ == "__main__":
    main()