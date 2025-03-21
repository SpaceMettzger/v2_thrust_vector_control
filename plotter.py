import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

def plot(rocket, time_values, show_orientation: bool = False):
    x_positions = [pos[0] for pos in rocket.records["lateral_positions"]]
    y_positions = [pos[1] for pos in rocket.records["lateral_positions"]]
    z_positions = [pos[2] for pos in rocket.records["lateral_positions"]]

    fig = plt.figure(figsize=(10, 8))

    if show_orientation:
        ax_3d = fig.add_subplot(211, projection='3d')
    else:
        ax_3d = fig.add_subplot(111, projection='3d')

    trajectory, = ax_3d.plot([], [], [], label="Rocket Trajectory", color="b")

    ax_3d.set_xlabel("X (Horizontal Distance, m)")
    ax_3d.set_ylabel("Y (Lateral Distance, m)")
    ax_3d.set_zlabel("Z (Altitude, m)")

    ax_3d.set_title("3D Rocket Trajectory (Aerospace NED)")
    ax_3d.legend()

    y_min, y_max = min(y_positions), max(y_positions)
    if y_min == y_max:
        y_min -= 1
        y_max += 1

    x_min, x_max = min(x_positions), max(x_positions)
    if x_min == x_max:
        x_min -= 1
        x_max += 1

    ax_3d.set_xlim(x_min, x_max)
    ax_3d.set_ylim(y_min, y_max)
    ax_3d.set_zlim(min(z_positions), max(z_positions))

    if show_orientation:
        ax_attitude = fig.add_subplot(212)
        ax_attitude.set_title("Rocket Attitude Over Time")
        ax_attitude.set_xlabel("Time (s)")
        ax_attitude.set_ylabel("Angle (degrees)")
        ax_attitude.plot(time_values, rocket.records["pitch_angles"], label="Pitch")
        ax_attitude.plot(time_values, rocket.records["yaw_angles"], label="Yaw")
        ax_attitude.plot(time_values, rocket.records["roll_angles"], label="Roll")
        ax_attitude.legend()

    ax_slider = plt.axes((0.2, 0.02, 0.65, 0.03))
    time_slider = Slider(ax_slider, "Time", 0, len(time_values) - 1, valinit=0, valstep=1)

    def update(val):
        t = int(time_slider.val)
        trajectory.set_data(x_positions[:t], y_positions[:t])  # Y replaces Z (lateral movement)
        trajectory.set_3d_properties(z_positions[:t])  # Z now represents altitude
        fig.canvas.draw_idle()

    time_slider.on_changed(update)

    plt.show()