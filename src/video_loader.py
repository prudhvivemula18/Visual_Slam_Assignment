import cv2
import os

def load_video(video_path=r"P:\OpenCv Proj\Lastmile assignment\video_input\IMG_2790.MOV"):
    if not os.path.exists(video_path):
        print(f"[ERROR] Video file does not exist: {video_path}")
        return []

    cap = cv2.VideoCapture(video_path)
    frames = []
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frames.append(frame_gray)

    cap.release()
    print(f"[INFO] Total frames loaded: {len(frames)}")
    return frames


if __name__ == "__main__":
    video_path = r"P:\OpenCv Proj\Lastmile assignment\video_input\IMG_2790.MOV"
    frames = load_video(video_path)

    if len(frames) == 0:
        print("[ERROR] No frames were loaded from the video.")
    else:
        cv2.imshow("Sample Frame", frames[0])
        cv2.waitKey(0)
        cv2.destroyAllWindows()

