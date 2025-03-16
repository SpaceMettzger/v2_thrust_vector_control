import numpy as np
import thrust
import math

class Rocket:
    def __init__(self):
        self.mass_gross = 12900 # kg
        self.mass_fuel = 8720 # kg
        self.dry_mass = self.mass_gross - self.mass_fuel
        self.current_mass = self.mass_gross
        self.current_fuel = self.mass_fuel
        self.length = 14.03 # m, including warhead
        self.length_warhead = 2.30 # m
        self.warhead_weight = 1000 # kg
        self.diameter = 1.651 # m
        self.specific_impulse_sea_level = 203 # s
        self.mass_flow_rate = thrust.calculate_mass_flow_rate()
        self.flight_time = 0

        self.x_position = 0
        self.y_position = 0
        self.z_position = 0
        self.x_velocity = 0
        self.y_velocity = 0
        self.z_velocity = 0

        self.pitch_angle = -90
        self.yaw_angle = 0
        self.roll_angle = 0
        self.pitch_acceleration = 0
        self.yaw_acceleration = 0
        self.roll_acceleration = 0
        self.pitch_velocity = 0
        self.yaw_velocity = 0
        self.roll_velocity = 0
        self.thrust_pitch_local = 0
        self.thrust_yaw_local = 0
        self.thrust_roll_local = 0

        self.records = {
            "thrust_values": [],
            "pitch_angles": [],
            "yaw_angles": [],
            "roll_angles": [],
            "pitch_accelerations": [],
            "yaw_accelerations": [],
            "roll_accelerations": [],
            "pitch_velocities": [],
            "yaw_velocities": [],
            "roll_velocities": [],
            "thrust_pitch_angles": [],
            "thrust_yaw_angles": [],
            "local_thrust_vectors": [],
            "lateral_accelerations": [],
            "lateral_velocities": [],
            "lateral_positions": []
        }

    def calculate_center_of_mass(self):
        """
        Computes the center of mass of the rocket dynamically as fuel is consumed.

        Returns:
            float: Center of mass (m) from the tail.
        """
        consumed_fuel = min(self.mass_fuel, self.mass_flow_rate * self.flight_time)
        self.current_fuel = max(0, self.mass_fuel - consumed_fuel)
        self.current_mass = self.dry_mass + self.current_fuel

        center_of_mass_warhead = self.warhead_weight * (self.length_warhead / 2)
        center_of_mass_body = self.current_mass * ((self.length - self.length_warhead) / 2)

        return (center_of_mass_warhead + center_of_mass_body) / self.current_mass

    def update_flight_time(self, dt):
        """
        Advances the rocket's flight time.

        Parameters:
            dt (float): Time step (seconds).
        """
        self.flight_time += dt

    def update_angular_motion(self, time_step=1, damping_factor=0.2):
        """
        Updates the rocket's angular velocity and angles (pitch, yaw, roll) over time,
        preserving inertia while introducing damping to prevent oscillations.

        Parameters:
            time_step (float): Simulation time step (s).
            damping_factor (float): Reduces angular velocity over time.
        """

        self.pitch_velocity = (self.pitch_velocity + self.pitch_acceleration * time_step) * (1 - damping_factor)
        self.yaw_velocity = (self.yaw_velocity + self.yaw_acceleration * time_step) * (1 - damping_factor)
        self.roll_velocity = (self.roll_velocity + self.roll_acceleration * time_step) * (1 - damping_factor)

        self.pitch_angle += self.pitch_velocity * time_step
        self.yaw_angle += self.yaw_velocity * time_step
        self.roll_angle += self.roll_velocity * time_step


    def update_velocity(self, acceleration_x, acceleration_y, acceleration_z, time_step=1):
        """
        Updates velocity components in the Aerospace (NED) coordinate system.

        Parameters:
            acceleration_x (float): Linear acceleration in m/s² along x-axis.
            acceleration_y (float): Linear acceleration in m/s² along y-axis.
            acceleration_z (float): Linear acceleration in m/s² along z-axis.
            time_step (float): Simulation time step in seconds.
        """
        self.x_velocity += acceleration_x * time_step
        self.y_velocity += acceleration_y * time_step
        self.z_velocity += acceleration_z * time_step

    def update_position(self, time_step=1):
        """
        Updates position using Euler integration in the Aerospace (NED) coordinate system.

        Parameters:
            time_step (float): Simulation time step in seconds.
        """
        self.x_position += self.x_velocity * time_step
        self.y_position += self.y_velocity * time_step
        self.z_position += self.z_velocity * time_step

    def get_thrust(self):
        thrust_val = thrust.calculate_thrust(self.mass_flow_rate, self.specific_impulse_sea_level) if self.current_fuel > 0 else 0
        return thrust_val

    def get_torque(self):
        """Computes torque based on the difference between thrust vector and body orientation."""
        thrust_force = self.get_thrust()
        lever_arm = self.length - self.calculate_center_of_mass()


        # Compute torque in the Aerospace (NED) frame using LOCAL deflection
        pitch_torque = lever_arm * thrust_force * math.sin(math.radians(self.thrust_pitch_local))  # Now around Y-axis
        yaw_torque = lever_arm * thrust_force * math.sin(math.radians(self.thrust_yaw_local))  # Now around Z-axis
        roll_torque = lever_arm * thrust_force * math.sin(math.radians(self.thrust_roll_local))  # Remains around X-axis

        return pitch_torque, yaw_torque, roll_torque

    @staticmethod
    def control_system_correction(velocity, target_pitch, angle):
        correction_angle = 0
        if velocity < 0 and (target_pitch - angle) > 1e-3:
            correction_angle = (velocity ** 2) / (2 * abs(target_pitch - angle))
        if velocity > 0 and (target_pitch - angle) < 1e-3:
            correction_angle = -(velocity ** 2) / (2 * abs(target_pitch - angle))
        if correction_angle != 0:
            pass
        return correction_angle

    def update_thruster_deflection(self, target_pitch, target_yaw, max_thrust_change=20, time_step=1):
        """
        Adjusts thruster deflection based on the difference between the rocket's orientation and the target thrust direction.

        Parameters:
            target_pitch (float): Desired pitch in NED frame (degrees).
            target_yaw (float): Desired yaw in NED frame (degrees).
            max_thrust_change (float): Maximum allowed deflection change per second (deg/s).
            time_step (float): Simulation time step (s).
        """

        max_deflection_change = max_thrust_change * time_step
        max_thruster_deflection = 30

        pitch_error = target_pitch - self.pitch_angle
        yaw_error = target_yaw - self.yaw_angle

        pitch_correction = max(-max_deflection_change, min(max_deflection_change, pitch_error))
        yaw_correction = max(-max_deflection_change, min(max_deflection_change, yaw_error))

        damping_factor = 0.3
        pitch_correction -= damping_factor * self.pitch_velocity
        yaw_correction -= damping_factor * self.yaw_velocity

        if abs(pitch_error) < 0.2 and abs(self.pitch_velocity) > abs(pitch_error) / time_step:
            pitch_correction = 0

        if abs(yaw_error) < 1.0 and abs(self.yaw_velocity) > abs(yaw_error) / time_step:
            yaw_correction = 0

        self.thrust_pitch_local += pitch_correction
        self.thrust_yaw_local += yaw_correction

        self.thrust_pitch_local = max(-max_thruster_deflection, min(max_thruster_deflection, self.thrust_pitch_local))
        self.thrust_yaw_local = max(-max_thruster_deflection, min(max_thruster_deflection, self.thrust_yaw_local))


    def calculate_thruster_deflection_transformation(self):
        def define_rotation_matrices(axis: str, angle_rad):
            if axis == "x":
                return np.array([
                    [1, 0, 0],
                    [0, math.cos(angle_rad), -math.sin(angle_rad)],
                    [0, math.sin(angle_rad), math.cos(angle_rad)]
                ])
            if axis == "y":
                return np.array([
                    [math.cos(angle_rad), 0, math.sin(angle_rad)],
                    [0, 1, 0],
                    [-math.sin(angle_rad), 0, math.cos(angle_rad)]
                ])
            if axis == "z":
                return np.array([
                    [math.cos(angle_rad), -math.sin(angle_rad), 0],
                    [math.sin(angle_rad), math.cos(angle_rad), 0],
                    [0, 0, 1]
                ])

        thrust_pitch_rad = math.radians(self.thrust_pitch_local) if math.radians(self.thrust_pitch_local) > 0.001 else 0
        thrust_yaw_rad = math.radians(self.thrust_yaw_local) if math.radians(self.thrust_yaw_local) > 0.001 else 0
        rocket_pitch_rad = math.radians(self.pitch_angle + 90) if math.radians(self.pitch_angle + 90) > 0.001 else 0
        rocket_yaw_rad = math.radians(self.yaw_angle) if math.radians(self.yaw_angle) > 0.001 else 0
        rocket_roll_rad = math.radians(self.roll_angle) if math.radians(self.roll_angle) > 0.001 else 0

        thrust_vector = (self.get_thrust()) * np.array([
            math.sin(thrust_pitch_rad) * math.cos(thrust_yaw_rad),
            math.sin(thrust_yaw_rad),
            math.cos(thrust_pitch_rad) * math.cos(thrust_yaw_rad)
        ])

        self.records["local_thrust_vectors"].append([float(thrust_vector[0]), float(thrust_vector[1]), float(thrust_vector[2])])

        r_yaw = define_rotation_matrices("x", rocket_yaw_rad)

        r_pitch = define_rotation_matrices("y", -rocket_pitch_rad)

        r_roll = define_rotation_matrices("z", rocket_roll_rad)

        global_thrust_vector = r_roll @ r_pitch @ r_yaw @ thrust_vector

        return global_thrust_vector[0], global_thrust_vector[1], global_thrust_vector[2]


    def record_rocket_params(self):
        self.records["thrust_values"].append(self.get_thrust())
        self.records["pitch_angles"].append(self.pitch_angle + 90)
        self.records["yaw_angles"].append(self.yaw_angle)
        self.records["roll_angles"].append(self.roll_angle)
        self.records["pitch_accelerations"].append(self.pitch_acceleration)
        self.records["yaw_accelerations"].append(self.yaw_acceleration)
        self.records["roll_accelerations"].append(self.roll_acceleration)
        self.records["pitch_velocities"].append(self.pitch_velocity)
        self.records["yaw_velocities"].append(self.yaw_velocity)
        self.records["roll_velocities"].append(self.roll_velocity)
        self.records["thrust_pitch_angles"].append(self.thrust_pitch_local)
        self.records["thrust_yaw_angles"].append(self.thrust_yaw_local)
