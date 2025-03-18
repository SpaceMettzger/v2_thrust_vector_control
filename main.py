import rocket_physics as rp
from rocket import Rocket
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import numpy as np


def run_simulation(thrust_vector_waypoints, vector_control: bool = True, pid_thruster_control: bool = True):

    rocket = Rocket()

    t = 0
    time_step = 1  # s
    total_time = 10000  # s

    time_values = []
    x_positions = []
    y_positions = []
    z_positions = []

    target_pitch, target_yaw = 0, 0

    while rocket.z_position >= 0 and t <= total_time:
        rp.update_flight_time(rocket, time_step)

        if t in thrust_vector_waypoints:
            target_yaw = thrust_vector_waypoints[t]["yaw"]
            target_pitch = rp.convert_target_pitch_yup_to_ned(thrust_vector_waypoints[t]["pitch"])

        if vector_control:
            rocket.update_thruster_deflection(target_pitch, target_yaw, pid_thruster_control)
            # rocket.update_thruster_deflection_simple(target_pitch, target_yaw)

            pitch_torque, yaw_torque, roll_torque= rp.get_torque(rocket)

            rp.calculate_angular_acceleration(rocket,
                pitch_torque, yaw_torque, roll_torque)

            rp.update_angular_motion(rocket, time_step)
        else:
            rocket.calculate_fuel_consumption()  # Needed to determine when thrust stops
            rocket.pitch_angle = target_pitch
            rocket.yaw_angle = target_yaw

        thrust_x, thrust_y, thrust_z = rocket.calculate_thruster_deflection_transformation()

        acceleration_x, acceleration_y, acceleration_z = rp.calculate_linear_acceleration(
            thrust_x, thrust_y, thrust_z, rocket)
        rp.update_velocity(rocket, acceleration_x, acceleration_y, acceleration_z, time_step)

        rp.update_position(rocket, time_step)

        time_values.append(t)
        x_positions.append(rocket.x_position)
        y_positions.append(rocket.y_position)
        z_positions.append(rocket.z_position)

        rocket.record_rocket_params()

        t += time_step

    print(max(rocket.records['lateral_velocities']))

    initial_position = np.array([x_positions[0], y_positions[0]])
    final_position = np.array([x_positions[-1], y_positions[-1]])

    # Compute range as horizontal displacement
    range_distance = np.linalg.norm(final_position - initial_position)

    print("Rocket altitude:", max(z_positions))
    print("Rocket range:", range_distance)

    return rocket, x_positions, y_positions, z_positions, time_values


def plot(rocket, x_positions, y_positions, z_positions, time_values, show_orientation: bool = False):

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

if __name__ == "__main__":
    thrust_vector_waypoints_yaw = {0: {"yaw": 0, "pitch": 0},
                                   10: {"yaw": 45, "pitch": 20},
                                   20: {"yaw": 45, "pitch": 20},
                                   30: {"yaw": 45, "pitch": 20},
                                   40: {"yaw": 45, "pitch": 20}}

    thrust_vector_waypoints_pitch = {0: {"yaw": 0, "pitch": 0},
                                     10: {"yaw": 15, "pitch": -45},
                                     20: {"yaw": 15, "pitch": -45},
                                     30: {"yaw": 90, "pitch": -45},
                                     40: {"yaw": 90, "pitch": -45}}

    rocket_pitch, x_positions_pitch, y_positions_pitch, z_positions_pitch, time_values_pitch = run_simulation(
        thrust_vector_waypoints_yaw, vector_control=True)
    rocket_yaw, x_positions_yaw, y_positions_yaw, z_positions_yaw, time_values_yaw = run_simulation(
        thrust_vector_waypoints_pitch, vector_control=True)
    plot(rocket_pitch, x_positions_pitch, y_positions_pitch, z_positions_pitch, time_values_pitch, show_orientation=True)
    plot(rocket_yaw, x_positions_yaw, y_positions_yaw, z_positions_yaw, time_values_yaw, show_orientation=True)

    # Todo - Check damping_factor in update_angular_motion