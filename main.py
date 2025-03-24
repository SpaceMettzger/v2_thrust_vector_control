import rocket_physics as rp
from rocket import Rocket
import numpy as np
from plotter import plot


def run_simulation(thrust_vector_waypoints, vector_control: bool = True, pid_thruster_control: bool = True):

    rocket = Rocket()

    t = 0
    time_step = 1  # s
    total_time = 10000  # s

    time_values = []

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

        rocket.record_rocket_params()

        t += time_step

    print(max(rocket.records['lateral_velocities']))

    print("Max Thrust: ", max(rocket.records['thrust_values']))

    return rocket, time_values




if __name__ == "__main__":
    thrust_vector_waypoints_yaw = {0: {"yaw": 0, "pitch": 0},
                                   10: {"yaw": 0, "pitch": 0},
                                   20: {"yaw": 45, "pitch": 65},
                                   30: {"yaw": 45, "pitch": 65},
                                   40: {"yaw": 45, "pitch": 65},
                                   50: {"yaw": 45, "pitch": 65}}

    thrust_vector_waypoints_pitch = {0: {"yaw": 0, "pitch": 0},
                                     10: {"yaw": 15, "pitch": -45},
                                     20: {"yaw": 15, "pitch": -45},
                                     30: {"yaw": 90, "pitch": -90},
                                     40: {"yaw": 120, "pitch": -90}}


    rocket_pitch, time_values_pitch = run_simulation(thrust_vector_waypoints_yaw, vector_control=True)
    plot(rocket_pitch, time_values_pitch, show_orientation=True)

    rocket_pitch_2, time_values_pitch_2 = run_simulation(thrust_vector_waypoints_yaw, vector_control=True,
                                                         pid_thruster_control=False)
    plot(rocket_pitch_2, time_values_pitch_2, show_orientation=True)

    rocket_yaw, time_values_yaw = run_simulation(thrust_vector_waypoints_pitch, vector_control=True,
                                                 pid_thruster_control=True)
    plot(rocket_yaw, time_values_yaw, show_orientation=True)

    rocket_yaw_2, time_values_yaw_2 = run_simulation(thrust_vector_waypoints_pitch, vector_control=True,
                                                     pid_thruster_control=False)
    plot(rocket_yaw_2, time_values_yaw_2, show_orientation=True)
