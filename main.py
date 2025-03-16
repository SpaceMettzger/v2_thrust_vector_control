import rocket_physics as rp
from rocket import Rocket
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider


def run_simulation(thrust_vector_waypoints):
    # Initialize Rocket
    rocket = Rocket()

    # Simulation Parameters
    t = 0
    time_step = 1  # s
    total_time = 500  # s


    # Store trajectory and attitude data
    time_values = []
    x_positions = []
    y_positions = []
    z_positions = []
    pitch_torques = []
    yaw_torques = []
    thrust_vectors = []

    target_pitch, target_yaw = 0, 0

    # Run Simulation
    while rocket.z_position >= 0 and t <= total_time:
        rocket.update_flight_time(time_step)

        if t in thrust_vector_waypoints:
            target_yaw = thrust_vector_waypoints[t]["yaw"]
            target_pitch = rp.convert_target_pitch_yup_to_ned(thrust_vector_waypoints[t]["pitch"])

        rocket.update_thruster_deflection(target_pitch, target_yaw)

        pitch_torque, yaw_torque, roll_torque= rocket.get_torque()
        pitch_torques.append(pitch_torque)
        yaw_torques.append(yaw_torque)

        rp.calculate_angular_acceleration(rocket,
            pitch_torque, yaw_torque, roll_torque)

        rocket.update_angular_motion(time_step)

        thrust_x, thrust_y, thrust_z = rocket.calculate_thruster_deflection_transformation()
        thrust_vectors.append([float(thrust_x), float(thrust_y), float(thrust_z)])

        acceleration_x, acceleration_y, acceleration_z = rp.calculate_linear_acceleration(
            thrust_x, thrust_y, thrust_z, rocket)
        rocket.update_velocity(acceleration_x, acceleration_y, acceleration_z, time_step)

        rocket.update_position(time_step)
        """
        print("Thrust: ", thrust_x, thrust_y, thrust_z)
        print("Acceleration: ", acceleration_x, acceleration_y, acceleration_z)
        print("Global_positions: ", rocket.x_position, rocket.y_position, rocket.z_position)
        """

        time_values.append(t)
        x_positions.append(rocket.x_position)
        y_positions.append(rocket.y_position)
        z_positions.append(rocket.z_position)

        rocket.record_rocket_params()

        t += time_step

    # print("pitch_angles: ", rocket.records["pitch_angles"])
    # print("yaw_angles: ", rocket.records["yaw_angles"])
    print("lateral_accelerations: ", rocket.records["lateral_accelerations"])


    return rocket, x_positions, y_positions, z_positions, time_values


def plot(rocket, x_positions, y_positions, z_positions, time_values):

    # Create Visualization
    fig = plt.figure(figsize=(10, 8))

    # 3D Trajectory Plot (Adjusted for NED)
    ax_3d = fig.add_subplot(211, projection='3d')
    trajectory, = ax_3d.plot([], [], [], label="Rocket Trajectory", color="b")

    # Correcting axis labels to match NED
    ax_3d.set_xlabel("X (Horizontal Distance, m)")  # X remains the same
    ax_3d.set_ylabel("Y (Lateral Distance, m)")  # Y is lateral movement (sideways)
    ax_3d.set_zlabel("Z (Altitude, m)")  # Z is altitude (downward in NED)

    ax_3d.set_title("3D Rocket Trajectory (Aerospace NED)")
    ax_3d.legend()

    # Fix axis limits (Z should show altitude)
    y_min, y_max = min(y_positions), max(y_positions)
    if y_min == y_max:
        y_min -= 1  # Small buffer
        y_max += 1

    x_min, x_max = min(x_positions), max(x_positions)
    if x_min == x_max:
        x_min -= 1  # Small buffer
        x_max += 1

    ax_3d.set_xlim(x_min, x_max)
    ax_3d.set_ylim(y_min, y_max)  # Lateral movement (Y)
    ax_3d.set_zlim(min(z_positions), max(z_positions))  # Altitude (flip because Z is down in NED)

    # Plot Pitch, Yaw, Roll vs Time
    ax_attitude = fig.add_subplot(212)
    ax_attitude.set_title("Rocket Attitude Over Time")
    ax_attitude.set_xlabel("Time (s)")
    ax_attitude.set_ylabel("Angle (degrees)")
    ax_attitude.plot(time_values, rocket.records["pitch_angles"], label="Pitch")
    ax_attitude.plot(time_values, rocket.records["yaw_angles"], label="Yaw")
    ax_attitude.plot(time_values, rocket.records["roll_angles"], label="Roll")
    ax_attitude.legend()

    # Add Time Slider
    ax_slider = plt.axes((0.2, 0.02, 0.65, 0.03))
    time_slider = Slider(ax_slider, "Time", 0, len(time_values) - 1, valinit=0, valstep=1)

    # Update function for the slider
    def update(val):
        t = int(time_slider.val)
        trajectory.set_data(x_positions[:t], y_positions[:t])  # Y replaces Z (lateral movement)
        trajectory.set_3d_properties(z_positions[:t])  # Z now represents altitude
        fig.canvas.draw_idle()

    time_slider.on_changed(update)

    plt.show()

if __name__ == "__main__":
    thrust_vector_waypoints_yaw = {0: {"yaw": 0, "pitch": 0},
                                   10: {"yaw": 90, "pitch": 0},
                                   20: {"yaw": 90, "pitch": 0},
                                   30: {"yaw": 90, "pitch": 0},
                                   40: {"yaw": 90, "pitch": 0}}

    thrust_vector_waypoints_pitch = {0: {"yaw": 0, "pitch": 0},
                                     10: {"yaw": 0, "pitch": 90},
                                     20: {"yaw": 0, "pitch": 90},
                                     30: {"yaw": 0, "pitch": 90},
                                     40: {"yaw": 0, "pitch": 90}}

    rocket, x_positions, y_positions, z_positions, time_values = run_simulation(thrust_vector_waypoints_pitch)
    # rocket, x_positions, y_positions, z_positions, time_values = run_simulation(thrust_vector_waypoints_yaw)
    plot(rocket, x_positions, y_positions, z_positions, time_values)