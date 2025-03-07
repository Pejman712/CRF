import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import pandas as pd

# Load the quaternion data from CSV
quaternion_data = pd.read_csv('quaternion_log.csv')

# Create a persistent figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Define the original coordinate axes
origin = np.array([0, 0, 0])
x_axis = np.array([1, 0, 0])
y_axis = np.array([0, 1, 0])
z_axis = np.array([0, 0, 1])

for index, row in quaternion_data.iterrows():
    quaternion = [row['w'], row['x'], row['y'], row['z']]  # w, x, y, z format

    # Create rotation object using the quaternion
    rotation = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])  # x, y, z, w format

    # Rotate the original axes using the quaternion
    x_rotated = rotation.apply(x_axis)
    y_rotated = rotation.apply(y_axis)
    z_rotated = rotation.apply(z_axis)

    # Clear the previous plot
    ax.cla()

    # Plot the original and rotated axes
    ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color='r', label='Original X', arrow_length_ratio=0.1)
    ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color='g', label='Original Y', arrow_length_ratio=0.1)
    ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color='b', label='Original Z', arrow_length_ratio=0.1)

    ax.quiver(origin[0], origin[1], origin[2], x_rotated[0], x_rotated[1], x_rotated[2], color='orange', label='Rotated X', arrow_length_ratio=0.1)
    ax.quiver(origin[0], origin[1], origin[2], y_rotated[0], y_rotated[1], y_rotated[2], color='purple', label='Rotated Y', arrow_length_ratio=0.1)
    ax.quiver(origin[0], origin[1], origin[2], z_rotated[0], z_rotated[1], z_rotated[2], color='cyan', label='Rotated Z', arrow_length_ratio=0.1)

    # Setting the plot limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # Labels and legend
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title(f'Quaternion Rotation Visualization {index + 1}/{len(quaternion_data)}')
    ax.legend()
    plt.grid(True)

    # Update the figure without closing it
    plt.pause(0.1)

# Keep the window open at the end
plt.show()
