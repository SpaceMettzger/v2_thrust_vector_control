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

        self.x_position = 0  # Horizontal position (m)
        self.y_position = 0  # Altitude (m)
        self.z_position = 0
        self.x_velocity = 0  # Horizontal velocity (m/s)
        self.y_velocity = 0  # Vertical velocity (m/s)
        self.z_velocity = 0

        self.pitch_angle = 0
        self.yaw_angle = 0
        self.roll_angle = 0
        self.prev_pitch_angle = 0
        self.prev_yaw_angle = 0
        self.prev_roll_angle = 0
        self.pitch_acceleration = 0
        self.yaw_acceleration = 0
        self.roll_acceleration = 0
        self.pitch_velocity = 0
        self.yaw_velocity = 0
        self.roll_velocity = 0
        self.thrust_pitch = 0
        self.thrust_yaw = 0

        self.thrust_values = []
        self.pitch_angles = []
        self.yaw_angles = []
        self.roll_angles = []
        self.pitch_accelerations = []
        self.yaw_accelerations = []
        self.roll_accelerations = []
        self.pitch_velocities = []
        self.yaw_velocities = []
        self.roll_velocities = []
        self.thrust_pitch_angles = []
        self.thrust_yaw_angles = []

        self.pitch_integral = 0  # Integral term storage
        self.yaw_integral = 0
        self.roll_integral = 0
        self.Kp = 1.0  # Proportional gain (Adjust as needed)
        self.Ki = 0.0  # Integral gain (Adjust as needed)
        self.Kd = 0.9  # Derivative gain (Adjust as needed)
        self.last_pitch_error = 0
        self.last_yaw_error = 0
        self.last_roll_error = 0

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


    def update_angular_motion(self, pitch_accel, yaw_accel, roll_accel, time_step=1, damping_factor=0.2):
        """
        Updates the rocket's angular velocity and angles (pitch, yaw, roll) over time,
        preserving inertia while introducing damping to prevent oscillations.

        Parameters:
            pitch_accel (float): Angular acceleration in pitch (rad/s²).
            yaw_accel (float): Angular acceleration in yaw (rad/s²).
            roll_accel (float): Angular acceleration in roll (rad/s²).
            time_step (float): Simulation time step (s).
            damping_factor (float): Reduces angular velocity over time.
         """
        # Update angular velocity with damping
        self.prev_pitch_angle = self.pitch_angle
        self.prev_yaw_angle = self.yaw_angle

        self.pitch_velocity = (self.pitch_velocity + pitch_accel * time_step)  * (1 - damping_factor)
        self.yaw_velocity = (self.yaw_velocity + yaw_accel * time_step)  * (1 - damping_factor)
        self.roll_velocity = (self.roll_velocity + roll_accel * time_step)  * (1 - damping_factor)

        # Update orientation (Euler integration)
        self.pitch_angle += self.pitch_velocity * time_step
        self.yaw_angle += self.yaw_velocity * time_step
        self.roll_angle += self.roll_velocity * time_step

    """
    def update_angular_motion(self, pitch_target, yaw_target, roll_target, time_step=1):
        # Calculate error
        pitch_error = pitch_target - self.pitch_angle
        yaw_error = yaw_target - self.yaw_angle
        roll_error = roll_target - self.roll_angle

        # Integral term (accumulated error over time)
        self.pitch_integral += pitch_error * time_step
        self.yaw_integral += yaw_error * time_step
        self.roll_integral += roll_error * time_step

        # Derivative term (change in error over time)
        pitch_derivative = (pitch_error - self.last_pitch_error) / time_step
        yaw_derivative = (yaw_error - self.last_yaw_error) / time_step
        roll_derivative = (roll_error - self.last_roll_error) / time_step

        # Update last error
        self.last_pitch_error = pitch_error
        self.last_yaw_error = yaw_error
        self.last_roll_error = roll_error

        # Compute PID control
        pitch_control = (self.Kp * pitch_error) + (self.Ki * self.pitch_integral) + (self.Kd * pitch_derivative)
        yaw_control = (self.Kp * yaw_error) + (self.Ki * self.yaw_integral) + (self.Kd * yaw_derivative)
        roll_control = (self.Kp * roll_error) + (self.Ki * self.roll_integral) + (self.Kd * roll_derivative)

        # Apply control to pitch and yaw
        self.pitch_angle += pitch_control * time_step
        self.yaw_angle += yaw_control * time_step
        self.roll_angle += roll_control * time_step
    """

    def update_velocity(self, acceleration_x, acceleration_y, acceleration_z, time_step = 1):
        self.x_velocity += acceleration_x * time_step
        self.y_velocity += acceleration_y * time_step
        self.z_velocity += acceleration_z * time_step


    def update_position(self, time_step = 1):
        """Updates position using Euler integration."""
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

        # Compute torque only for the difference between current rocket angle and thrust vector
        pitch_torque = lever_arm * thrust_force * math.sin(math.radians(self.thrust_pitch))
        yaw_torque = lever_arm * thrust_force * math.sin(math.radians(self.thrust_yaw))

        return pitch_torque, yaw_torque

    def should_start_counteracting(self, current_pitch, target_pitch, pitch_velocity, max_deceleration):
        """
        Determines if the rocket needs to start counteracting its rotation
        to avoid overshooting the target angle.

        Parameters:
            current_pitch (float): The current pitch angle (degrees).
            target_pitch (float): The desired pitch angle (degrees).
            pitch_velocity (float): The current angular velocity (degrees/s).
            max_deceleration (float): The maximum possible angular deceleration (degrees/s²).

        Returns:
            bool: True if counteracting should start, False otherwise.
        """

        # Compute remaining angle to target
        delta_theta = target_pitch - current_pitch  # Remaining rotation needed

        # If we're already at the target, no correction needed
        if abs(delta_theta) < 1e-3:
            return False

        # Compute the required deceleration using kinematic equation:
        # v^2 = v_0^2 + 2 * a * delta_theta  (solving for a)
        if abs(delta_theta) > 1e-3 and pitch_velocity != 0:
            required_deceleration = (pitch_velocity ** 2) / (2 * abs(delta_theta))
        else:
            required_deceleration = 0

        # If required deceleration is more than what we can apply, we need to start counteracting
        return required_deceleration > abs(max_deceleration)

    def update_thrust_vector(self, target_pitch, target_yaw, max_thrust_change=20, time_step=1):
        """
        Updates the thrust vector towards the target while ensuring smooth control,
        preventing overshoot, and respecting thruster limitations.

        Parameters:
            target_pitch (float): Desired pitch angle (degrees).
            target_yaw (float): Desired yaw angle (degrees).
            max_thrust_change (float): Maximum allowed change in thrust vector per second (deg/s).
            time_step (float): Simulation time step (s).
            damping_factor (float): Reduces velocity to smooth out oscillations.
        """

        max_change = max_thrust_change * time_step
        max_thruster_deflection = -45  # Thruster can only tilt between -45° and +45°

        error_pitch = target_pitch - self.pitch_angle
        self.thrust_pitch += max(-max_change, min(max_change, error_pitch))

        self.thrust_yaw += self.thrust_yaw - abs(self.yaw_angle - self.prev_yaw_angle)

        damping = 0.2
        if self.pitch_angle - target_pitch > target_pitch / 10 and self.pitch_velocity !=0:
            correction_pitch = -self.pitch_velocity * damping if abs(self.pitch_velocity) > 0.001 else 0
            print(f"break_condition, correction pitch: {correction_pitch}")
        else:
            correction_pitch = max(-max_change, min(max_change, target_pitch))
        print(self.pitch_angle - target_pitch, correction_pitch)

        should_brake_yaw = self.should_start_counteracting(self.yaw_angle, target_yaw, self.yaw_velocity,
                                                      self.yaw_acceleration)

        if should_brake_yaw:
            correction_yaw = - (self.yaw_velocity ** 2) / (2 * self.yaw_acceleration) if self.yaw_acceleration != 0 else 0
        else:
            correction_yaw = max(-max_change, min(max_change, target_yaw))

        self.thrust_pitch = max(-max_thruster_deflection - self.pitch_angle,
                                min(max_thruster_deflection -  self.pitch_angle, self.pitch_angle + correction_pitch))
        self.thrust_yaw = max(-max_thruster_deflection - self.yaw_angle, min(max_thruster_deflection + self.yaw_angle, self.thrust_yaw + correction_yaw))

    def record_rocket_params(self):
        self.thrust_values.append(self.get_thrust())
        self.pitch_angles.append(self.pitch_angle)
        self.yaw_angles.append(self.yaw_angle)
        self.roll_angles.append(self.roll_angle)
        self.pitch_accelerations.append(self.pitch_acceleration)
        self.yaw_accelerations.append(self.yaw_acceleration)
        self.roll_accelerations.append(self.roll_acceleration)
        self.pitch_velocities.append(self.pitch_velocity)
        self.yaw_velocities.append(self.yaw_velocity)
        self.roll_velocities.append(self.roll_velocity)
        self.thrust_pitch_angles.append(self.thrust_pitch)
        self.thrust_yaw_angles.append(self.thrust_yaw)
