import cv2
import numpy as np
import open3d as o3d
from video_loader import load_video

K = np.array([[718.856, 0, 607.1928],
              [0, 718.856, 185.2157],
              [0, 0, 1]])

def build_point_cloud(frames, K):
    orb = cv2.ORB_create(2000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    pcd = o3d.geometry.PointCloud()
    prev_gray = frames[0]
    prev_kp, prev_des = orb.detectAndCompute(prev_gray, None)

    for i in range(1, len(frames)):
        curr_gray = frames[i]
        curr_kp, curr_des = orb.detectAndCompute(curr_gray, None)
        if curr_des is None or prev_des is None:
            continue

        matches = bf.match(prev_des, curr_des)
        matches = sorted(matches, key=lambda x: x.distance)[:300]

        pts1 = np.float32([prev_kp[m.queryIdx].pt for m in matches])
        pts2 = np.float32([curr_kp[m.trainIdx].pt for m in matches])

        E, _ = cv2.findEssentialMat(pts2, pts1, K, method=cv2.RANSAC, threshold=1.0)
        if E is None:
            continue

        _, R, t, _ = cv2.recoverPose(E, pts2, pts1, K)

        P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = K @ np.hstack((R, t))

        pts4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
        pts4d /= pts4d[3]

        points_3d = pts4d[:3].T
        pcd.points.extend(o3d.utility.Vector3dVector(points_3d))

        prev_gray = curr_gray
        prev_kp = curr_kp
        prev_des = curr_des

    return pcd

if __name__ == "__main__":
    video_path = r"P:\OpenCv Proj\Lastmile assignment\video_input\IMG_2790.MOV"  # use relative path
    frames = load_video(video_path)
    if len(frames) == 0:
        print("[ERROR] No frames loaded.")
        exit()

    print("[INFO] Building 3D point cloud from video...")
    pcd = build_point_cloud(frames, K)

    # Try to load trajectory and add it
    try:
        traj = np.loadtxt("output/trajectory.txt")
        traj_z = np.random.normal(0, 0.2, (traj.shape[0], 1))  # Simulated depth
        traj_points = np.hstack((traj, traj_z))

        traj_geom = o3d.geometry.PointCloud()
        traj_geom.points = o3d.utility.Vector3dVector(traj_points)
        traj_geom.paint_uniform_color([1, 0, 0])  # Red

        # Add robot final pose marker
        final_pose = traj_points[-1]
        robot_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
        robot_marker.translate(final_pose)
        robot_marker.paint_uniform_color([1, 0, 0])

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="3D Map with Robot Pose", width=1024, height=768, visible=True)
        vis.add_geometry(pcd)
        vis.add_geometry(traj_geom)
        vis.add_geometry(robot_marker)
        vis.poll_events()
        vis.update_renderer()
        vis.run()
        vis.destroy_window()

    except Exception as e:
        print("[WARNING] Could not overlay trajectory:", e)
        # Fall back to showing only the point cloud
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="3D Point Cloud", width=1024, height=768)
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()

    input("Press Enter to exit...")
