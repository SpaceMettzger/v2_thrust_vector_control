import math


def calculate_center_of_mass(rocket):
    """
    Computes the center of mass of the rocket dynamically as fuel is consumed.

    Returns:
        float: Center of mass (m) from the tail.
    """
    rocket.calculate_fuel_consumption()

    rocket.current_mass = rocket.dry_mass + rocket.current_fuel

    center_of_mass_warhead = rocket.warhead_weight * (rocket.length_warhead / 2)
    center_of_mass_body = rocket.current_mass * ((rocket.length - rocket.length_warhead) / 2)

    return (center_of_mass_warhead + center_of_mass_body) / rocket.current_mass


def update_flight_time(rocket, dt):
    """
    Advances the rocket's flight time.

    Parameters:
        dt (float): Time step (seconds).
    """
    rocket.flight_time += dt


def update_angular_motion(rocket, time_step=1, damping_factor=0.2):
    """
    Updates the rocket's angular velocity and angles (pitch, yaw, roll) over time,
    preserving inertia while introducing damping to prevent oscillations.

    Parameters:
        time_step (float): Simulation time step (s).
        damping_factor (float): Reduces angular velocity over time.
    """

    rocket.pitch_velocity = (rocket.pitch_velocity + rocket.pitch_acceleration * time_step) * (1 - damping_factor)
    rocket.yaw_velocity = (rocket.yaw_velocity + rocket.yaw_acceleration * time_step) * (1 - damping_factor)
    rocket.roll_velocity = (rocket.roll_velocity + rocket.roll_acceleration * time_step) * (1 - damping_factor)

    rocket.pitch_angle += rocket.pitch_velocity * time_step
    rocket.yaw_angle += rocket.yaw_velocity * time_step
    rocket.roll_angle += rocket.roll_velocity * time_step


def update_velocity(rocket, acceleration_x, acceleration_y, acceleration_z, time_step=1):
    """
    Updates velocity components in the Aerospace (NED) coordinate system.

    Parameters:
        acceleration_x (float): Linear acceleration in m/s² along x-axis.
        acceleration_y (float): Linear acceleration in m/s² along y-axis.
        acceleration_z (float): Linear acceleration in m/s² along z-axis.
        time_step (float): Simulation time step in seconds.
    """
    rocket.x_velocity += acceleration_x * time_step
    rocket.y_velocity += acceleration_y * time_step
    rocket.z_velocity += acceleration_z * time_step
    print(rocket.x_velocity, rocket.y_velocity, rocket.z_velocity)


def update_position(rocket, time_step=1):
    """
    Updates position using Euler integration in the Aerospace (NED) coordinate system.

    Parameters:
        time_step (float): Simulation time step in seconds.
    """
    rocket.x_position += rocket.x_velocity * time_step
    rocket.y_position += rocket.y_velocity * time_step
    rocket.z_position += rocket.z_velocity * time_step


def get_torque(rocket):
    """Computes torque based on the difference between thrust vector and body orientation."""
    thrust_force = rocket.get_thrust()
    lever_arm = rocket.length - calculate_center_of_mass(rocket)

    # Compute torque in the Aerospace (NED) frame using LOCAL deflection
    pitch_torque = lever_arm * thrust_force * math.sin(math.radians(rocket.thrust_pitch_local))  # Now around Y-axis
    yaw_torque = lever_arm * thrust_force * math.sin(math.radians(rocket.thrust_yaw_local))  # Now around Z-axis
    roll_torque = lever_arm * thrust_force * math.sin(math.radians(rocket.thrust_roll_local))  # Remains around X-axis

    return pitch_torque, yaw_torque, roll_torque

def calculate_inertia(mass, radius, length):
    pitch_and_yaw_inertia = (mass * radius**2) / 4 + (mass * length**2) / 12
    roll_inertia = (mass * radius**2) / 2
    return pitch_and_yaw_inertia, roll_inertia

def calculate_angular_acceleration(rocket, pitch_torque, yaw_torque, roll_torque):
    """
    Computes angular acceleration based on applied torques and rocket inertia.

    Parameters:
        rocket: The rocket object.
        pitch_torque (float): Torque applied around the Y-axis (NED).
        yaw_torque (float): Torque applied around the Z-axis (NED).
        roll_torque (float): Torque applied around the X-axis (NED).
    """
    pitch_and_yaw_inertia, roll_inertia = calculate_inertia(rocket.current_mass, rocket.diameter / 2, rocket.length)

    # Apply torques to the correct axes in the NED coordinate system
    rocket.pitch_acceleration = pitch_torque / pitch_and_yaw_inertia  # Now around Y-axis
    rocket.yaw_acceleration = yaw_torque / pitch_and_yaw_inertia  # Now around Z-axis
    rocket.roll_acceleration = roll_torque / roll_inertia  # Remains around X-axis


def calculate_linear_acceleration(thrust_x, thrust_y, thrust_z, rocket):
    """
    Computes the linear acceleration including thrust, drag, and gravity in the Aerospace (NED) coordinate system.

    Parameters:
        thrust_x (float): Thrust force components (N) in NED.
        thrust_y (float): Thrust force components (N) in NED.
        thrust_z (float): Thrust force components (N) in NED.
        rocket (Rocket): The rocket object.

    Returns:
        tuple: (acceleration_x, acceleration_y, acceleration_z) in m/s².
    """

    # Get drag forces
    drag_x, drag_y, drag_z = calculate_drag(rocket)

    # Compute acceleration (F = ma)
    acceleration_x = (thrust_x + drag_x / 2) / rocket.current_mass
    acceleration_y = (thrust_y + drag_y / 2) / rocket.current_mass
    acceleration_z = (thrust_z + drag_z / 2) / rocket.current_mass

    acceleration_z -= 9.81  # Gravity in the downward (NED) direction
    rocket.records["lateral_accelerations"].append([float(acceleration_x), float(acceleration_y), float(acceleration_z)])

    return acceleration_x, acceleration_y, acceleration_z


def calculate_drag(rocket):
    """
    Computes the aerodynamic drag force on the rocket.

    Parameters:
        rocket (Rocket): The rocket object containing velocity and altitude.

    Returns:
        drag_x, drag_y, drag_z: Drag forces in each direction (N).
    """
    # Constants
    cd = 0.55  # Drag coefficient (approx. for V2)
    a = math.pi * (rocket.diameter / 2) ** 2  # Cross-sectional area (m²)

    # Get velocity components
    v_x, v_y, v_z = rocket.x_velocity, rocket.y_velocity, rocket.z_velocity
    v_total = math.sqrt(v_x**2 + v_y**2 + v_z**2)  # Magnitude of velocity

    # Get atmospheric density based on altitude
    rho = get_air_density(rocket.z_position)

    # Compute drag force
    drag_force = 0.5 * cd * rho * v_total**2 * a

    # Compute drag components (opposite to velocity direction)
    if v_total > 0:
        drag_x = -drag_force * (v_x / v_total)
        drag_y = -drag_force * (v_y / v_total)
        drag_z = -drag_force * (v_z / v_total)
    else:
        drag_x, drag_y, drag_z = 0, 0, 0  # No drag if no movement
    return drag_x, drag_y, drag_z

def get_air_density(altitude):
    """
    Computes atmospheric density based on altitude.

    Parameters:
        altitude (float): Rocket's altitude (m).

    Returns:
        rho (float): Air density (kg/m³).
    """
    if altitude < 11000:
        # Troposphere (up to ~11km)
        rho0 = 1.225  # kg/m³ at sea level
        h = 8500  # Scale height (m)
        return rho0 * math.exp(-altitude / h)
    elif altitude < 25000:
        # Stratosphere (11-25 km, roughly constant density)
        return 0.3  # kg/m³ (approx)
    else:
        # Upper atmosphere (very low density)
        return 0.02  # kg/m³ (approx)

def convert_target_pitch_yup_to_ned(target_pitch):
    """
    Converts the target pitch from a Y-up system to an Aerospace (NED) system.

    Parameters:
        target_pitch (float): The pitch angle in degrees (Y-up).

    Returns:
        float: The converted pitch angle in degrees (Aerospace NED).
    """
    return target_pitch - 90  # Adjust for coordinate system change





