import math

def calculate_inertia(mass, radius, length):
    pitch_and_yaw_inertia = (mass * radius**2) / 4 + (mass * length**2) / 12
    roll_inertia = (mass * radius**2) / 2
    return pitch_and_yaw_inertia, roll_inertia

def calculate_angular_acceleration(rocket, pitch_torque, yaw_torque, roll_torque):
    pitch_and_yaw_inertia, roll_inertia = calculate_inertia(rocket.current_mass, rocket.diameter / 2, rocket.length)
    rocket.pitch_acceleration = pitch_torque / pitch_and_yaw_inertia
    rocket.yaw_acceleration = yaw_torque / pitch_and_yaw_inertia
    rocket.roll_acceleration = roll_torque / roll_inertia

def calculate_linear_acceleration(thrust_x, thrust_y, thrust_z, rocket):
    """
    Computes the linear acceleration including thrust, drag, and gravity.

    Parameters:
        thrust_x, thrust_y, thrust_z (float): Thrust force components (N).
        rocket (Rocket): The rocket object.

    Returns:
        tuple: (acceleration_x, acceleration_y, acceleration_z) in m/s².
    """
    if rocket.current_mass <= 0:
        return 0, -9.81, 0  # Free fall if out of mass

    # Get drag forces
    drag_x, drag_y, drag_z = calculate_drag(rocket)

    # Compute acceleration (F = ma)
    acceleration_x = (thrust_x + drag_x) / rocket.current_mass
    acceleration_y = (thrust_y + drag_y) / rocket.current_mass - 9.81  # Gravity acts downward
    acceleration_z = (thrust_z + drag_z) / rocket.current_mass

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
    Cd = 0.55  # Drag coefficient (approx. for V2)
    A = math.pi * (rocket.diameter / 2) ** 2  # Cross-sectional area (m²)

    # Get velocity components
    v_x, v_y, v_z = rocket.x_velocity, rocket.y_velocity, rocket.z_velocity
    v_total = math.sqrt(v_x**2 + v_y**2 + v_z**2)  # Magnitude of velocity

    # Get atmospheric density based on altitude
    rho = get_air_density(rocket.y_position)

    # Compute drag force
    drag_force = 0.5 * Cd * rho * v_total**2 * A

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
        H = 8500  # Scale height (m)
        return rho0 * math.exp(-altitude / H)
    elif altitude < 25000:
        # Stratosphere (11-25 km, roughly constant density)
        return 0.3  # kg/m³ (approx)
    else:
        # Upper atmosphere (very low density)
        return 0.02  # kg/m³ (approx)




