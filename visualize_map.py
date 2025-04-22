# This script visualizes the 3D trajectory of a robot in a simulated environment.
import numpy as np
import matplotlib.pyplot as plt

def visualize_trajectory():
    traj = np.loadtxt("output/trajectory.txt")
    z = np.random.normal(0, 0.2, (traj.shape[0], 1))
    points_3d = np.hstack((traj, z))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory
    ax.plot(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], label='Trajectory', marker='o')

    # Mark robot pose
    final_pose = points_3d[-1]
    ax.scatter(final_pose[0], final_pose[1], final_pose[2], c='red', s=100, label='Robot Pose')

    ax.set_title("Estimated 3D Trajectory with Robot Pose")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z (simulated)")
    ax.grid(True)
    ax.legend()

    ax.set_xlim([min(points_3d[:, 0])-10, max(points_3d[:, 0])+10])
    ax.set_ylim([min(points_3d[:, 1])-10, max(points_3d[:, 1])+10])
    ax.set_zlim([min(points_3d[:, 2])-1, max(points_3d[:, 2])+1])

    # Save to file
    plt.savefig("output/trajectory_plot_with_pose.png")
    plt.show()

if __name__ == "__main__":
    visualize_trajectory()


