import cv2
import numpy as np
import os
from video_loader import load_video

def run_feature_tracking(frames):
    lk_params = dict(winSize=(21, 21), maxLevel=3,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

    p0 = cv2.goodFeaturesToTrack(frames[0], maxCorners=500, qualityLevel=0.3, minDistance=7)
    trajectory = [np.array([0.0, 0.0])]  # Start at origin

    for i in range(1, len(frames)):
        p1, st, err = cv2.calcOpticalFlowPyrLK(frames[i - 1], frames[i], p0, None, **lk_params)
        if p1 is None: break

        motion_vector = np.mean(p1 - p0, axis=0)
        new_pos = trajectory[-1] + motion_vector.flatten()[:2]
        trajectory.append(new_pos)

        p0 = p1

    return np.array(trajectory)

if __name__ == "__main__":
    video_path = r"P:\OpenCv Proj\Lastmile assignment\video_input\IMG_2790.MOV"
    frames = load_video(video_path)
    if len(frames) == 0:
        print("[ERROR] No frames were loaded from the video.")
        exit()

    traj = run_feature_tracking(frames)

    # Ensure the output directory exists
    output_dir = "output"
    os.makedirs(output_dir, exist_ok=True)

    # Save the trajectory
    output_file = os.path.join(output_dir, "trajectory.txt")
    np.savetxt(output_file, traj)
    print(f"[INFO] Trajectory saved to {output_file}.")
