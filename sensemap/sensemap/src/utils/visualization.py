import matplotlib.pyplot as plt
import numpy as np

def plot_2d_map(positions, objects):
    """
    Plots a 2D map with robot positions and detected objects.

    Parameters:
    - positions: List of tuples containing (x, y) positions of the robot.
    - objects: List of tuples containing (x, y, label) for detected objects.
    """
    plt.figure(figsize=(10, 10))
    plt.title("2D Map Visualization")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")

    # Plot robot positions
    if positions:
        x_positions, y_positions = zip(*positions)
        plt.plot(x_positions, y_positions, marker='o', color='blue', label='Robot Path')

    # Plot detected objects
    for obj in objects:
        x, y, label = obj
        plt.scatter(x, y, marker='x', color='red', label=label)

    plt.legend()
    plt.grid()
    plt.axis('equal')
    plt.show()

def plot_point_cloud(points):
    """
    Visualizes a point cloud in 3D.

    Parameters:
    - points: Nx3 array-like structure containing (x, y, z) coordinates.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o')
    ax.set_title("3D Point Cloud Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()