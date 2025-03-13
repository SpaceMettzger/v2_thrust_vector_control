from rocketcea.cea_obj import CEA_Obj, add_new_fuel
import math
import numpy as np

def calculate_mass_flow_rate(nasa_cea: bool = False, choked_flow: bool = False) -> float:

    def calculate_mass_flow_rate_cea() -> float:
        ethanol_water_blend = """ 
        fuel C2H5OH(L)   C 2 H 6 O 1   wt%=75.00 
        h,cal=-66370.0   t(k)=298.15 
        fuel H2O   H 2 O 1   wt%=25.00 
        h,cal=-68308.0   t(k)=298.15 
        """

        custom_fuel_name = "Ethanol75Water25"
        add_new_fuel(custom_fuel_name, ethanol_water_blend)

        cea = CEA_Obj(oxName="LOX", fuelName=custom_fuel_name)

        pc = 15  # Chamber pressure in bar
        expansion_ratio = 1.9  # Estimated nozzle expansion ratio
        mr = 1.27  # Mixture Ratio (Oxidizer/Fuel)

        isp_vac = cea.get_Isp(Pc=pc, MR=mr, eps=expansion_ratio)
        # ve = isp_vac * 9.81  # Convert Isp to exhaust velocity in m/s
        # molecular_weight, gamma_throat = cea.get_Throat_MolWt_gamma(Pc=pc, MR=mr)

        g0 = 9.81  # Gravity acceleration in m/s²
        a_e = 0.21  # Effective nozzle exit area in m²

        mass_flow_rate = (pc * 1e5 * a_e) / (g0 * isp_vac)

        return mass_flow_rate

    def calculate_gamma_mixture(gas_mixture=None):
        """
        Computes the specific heat ratio (gamma) for a given gas mixture.

        Parameters:
            gas_mixture (dict): Optional dictionary of gas components with values as (mass fraction, Cp, Cv).

        Returns:
            gamma_mixture (float): The specific heat ratio for the gas mixture.
        """
        if gas_mixture is None:
            gas_mixture = {
                "water_vapor": (0.20, 1950, 1500),
                "carbon_dioxide": (0.25, 846, 655),
                "oxygen": (0.10, 918, 659),
                "nitrogen": (0.10, 1040, 743),
                "carbon_monoxide": (0.15, 1040, 743),
                "hydrogen": (0.20, 14300, 10100),
            }

        cp_mixture = sum(X * Cp for X, Cp, _ in gas_mixture.values())
        cv_mixture = sum(X * Cv for X, _, Cv in gas_mixture.values())

        return cp_mixture / cv_mixture

    def choked_mass_flow(a, pt, tt, g, r: float = 314, m=1) -> float:
        """
        Computes the choked mass flow rate given:

        Parameters:
        a  = Throat area (m²)
        pt = Total (stagnation) pressure (Pa)
        tt = Total (stagnation) temperature (K)
        g = Specific heat ratio gamma (Cp/Cv)
        r  = Specific gas constant (J/kg·K) (NASA seems to assume a value of ~314 in their widget)
        m  = Mach number at the throat (default is 1 for choked flow)

        Returns:
        mdot = Mass flow rate (kg/s)
        """

        term1 = (a * pt / math.sqrt(tt))
        term2 = math.sqrt(g / r) * m
        term3 = (1 + 0.5 * (g - 1) * m ** 2) ** (-((g + 1) / (2 * (g - 1))))

        mdot = term1 * term2 * term3
        return mdot

    def isentropic_density_scaling_method(a, pt, tt, g, r: float = 314):
        """
        Computes the choked mass flow rate given:

        Parameters:
        a  = Throat area (m²)
        pt = Total chamber pressure (Pa)
        tt = Total (stagnation) temperature (K)
        g = Specific heat ratio gamma (Cp/Cv)
        r  = Specific gas constant (J/kg·K) (NASA seems to assume a value of ~314 in their widget)

        Returns:
        mdot = Mass flow rate (kg/s)
        """
        throat_velocity = math.sqrt(1.1 * r * tt)  # m/s

        rho = calculate_chamber_density(
            r_mixture=r,
            p_chamber=pt,  # Chamber pressure in Pa
            t_chamber=tt  # Chamber temperature in K
        )

        p_throat = rho * (0.528 ** (1 / g))  # Proper isentropic density scaling
        m_dot = p_throat * throat_velocity * a

        return m_dot

    def calculate_specific_gas_constant_of_mixture(**kwargs):
        """
        Calculates the specific gas constant for a given fuel mixture.

        Parameters:
        **kwargs: Dictionary where keys are substance names and values are tuples (mass_kg, molecular_weight_kg_per_kmol).
        """

        r_universal = 8314  # J/kmol·K
        total_mass = sum(mass for mass, _ in kwargs.values())

        r_mixture_inv = sum((mass / total_mass) / (r_universal / mw) for mass, mw in kwargs.values())
        r_mixture = 1 / r_mixture_inv

        return r_mixture

    def calculate_chamber_density(r_mixture, p_chamber=1_500_000, t_chamber=2500):
        """
        Calculates the combustion chamber density for a given mixture of substances.

        Parameters:
            r_mixture (float): Specific gas constant of fuel mixture
            p_chamber (float): Chamber pressure in Pascals.
            t_chamber (float): Chamber temperature in Kelvin.

        Returns:
            rho_chamber (float): Density of the combustion chamber (kg/m³).
        """

        rho_chamber = p_chamber / (r_mixture * t_chamber)
        return rho_chamber

    if nasa_cea:
        return calculate_mass_flow_rate_cea()

    fuel_mixture = {"ethanol": {"mass": 3810 * 0.75,
                                "mol_weight": 46.07},
                    "water": {"mass": 3810 * 0.25,
                              "mol_weight": 18.02},
                    "oxygen": {"mass": 4910,
                               "mol_weight": 32.00}}

    r_specific = calculate_specific_gas_constant_of_mixture(
        **{k: (v["mass"], v["mol_weight"]) for k, v in fuel_mixture.items()}
    )
    diameter_throat = 0.40  # m
    a_throat = (diameter_throat / 2) ** 2 * math.pi

    chamber_temperature = 2670 + 273.15
    chamber_pressure = 1.5 * 1e6
    # gamma = calculate_gamma_mixture()
    gamma = 1.11  # Specific heat ratio

    if choked_flow:
        return choked_mass_flow(a_throat, chamber_pressure, chamber_temperature, gamma, r_specific, m=1)

    else:
        return isentropic_density_scaling_method(a_throat, chamber_pressure, chamber_temperature, gamma, r_specific)

def calculate_thrust(mass_flow_rate = 133.155, isp = 203, g0=9.81):
    """
    Computes rocket thrust.

    Parameters:
        mass_flow_rate (float): Mass of fuel burned per second (kg/s).
        isp (float): Specific impulse (seconds).
        g0 (float): Standard gravity (m/s²), default is 9.81.

    Returns:
        thrust (float): Thrust force in Newtons.
    """
    thrust = mass_flow_rate * isp * g0
    return thrust

def update_thrust_vector(current_theta, current_phi, target_theta, target_phi, max_rate, dt):
    """
    Updates the thrust vector angles based on a max gimbal rate.

    Parameters:
        current_theta (float): Current pitch deflection angle (degrees).
        current_phi (float): Current yaw deflection angle (degrees).
        target_theta (float): Desired pitch deflection (degrees).
        target_phi (float): Desired yaw deflection (degrees).
        max_rate (float): Maximum gimbal movement per second (deg/s).
        dt (float): Time step (seconds).

    Returns:
        new_theta, new_phi: Updated thrust vector angles.
    """
    def clamp_angle_change(current, target, max_change):
        delta = target - current
        if abs(delta) > max_change:
            return current + math.copysign(max_change, delta)
        return target

    max_change = max_rate * dt
    new_theta = clamp_angle_change(current_theta, target_theta, max_change)
    new_phi = clamp_angle_change(current_phi, target_phi, max_change)

    return new_theta, new_phi

def calculate_thrust_torque(thrust, theta_t, phi_t, offset):
    """
    Computes the torque induced by thrust vectoring.

    Parameters:
        thrust (float): Thrust force in Newtons.
        theta_t (float): Pitch deflection angle (radians).
        phi_t (float): Yaw deflection angle (radians).
        offset (float): Distance from center of mass to thrust vector (meters).

    Returns:
        pitch_torque, yaw_torque: Torque values in Nm.
    """
    pitch_torque = offset * thrust * math.sin(math.radians(theta_t))
    yaw_torque = offset * thrust * math.sin(math.radians(phi_t))

    return pitch_torque, yaw_torque

def calculate_thrust_vector_components_old(thrust, pitch_angle, yaw_angle):
    """
    Computes the thrust force components in X, Y, and Z directions.

    Parameters:
        thrust (float): Total thrust force (N).
        pitch_angle (float): Pitch deflection angle (degrees).
        yaw_angle (float): Yaw deflection angle (degrees).

    Returns:
        tuple: (thrust_x, thrust_y, thrust_z) in Newtons (N).
    """
    thrust_x = thrust * np.sin(np.radians(pitch_angle)) * np.cos(np.radians(yaw_angle))
    thrust_y = thrust * np.cos(np.radians(pitch_angle))  # Major force along Y (upward)
    thrust_z = thrust * np.sin(np.radians(pitch_angle)) * np.sin(np.radians(yaw_angle))

    return thrust_x, thrust_y, thrust_z

import numpy as np

def calculate_thrust_vector_components(rocket):
    """
    Computes the thrust force components in global X, Y, and Z directions,
    considering the thruster's deflection and the rocket's global orientation.

    Parameters:
        rocket (Rocket): rocket object.

    Returns:
        tuple: (thrust_x, thrust_y, thrust_z) in Newtons (N).
    """

    thrust = rocket.get_thrust()

    # Convert angles to radians
    rp = np.radians(rocket.pitch_angle)
    ry = np.radians(rocket.yaw_angle)

    epsilon = 1e-10  # Tolerance level for floating-point errors
    thrust_local_x = thrust * np.sin(np.radians(rocket.thrust_pitch))
    thrust_local_y = thrust * np.cos(np.radians(rocket.thrust_pitch))
    thrust_local_z = thrust * np.sin(np.radians(rocket.thrust_yaw))

    thrust_local_x = 0 if abs(thrust_local_x) < epsilon else thrust_local_x
    thrust_local_y = 0 if abs(thrust_local_y) < epsilon else thrust_local_y
    thrust_local_z = 0 if abs(thrust_local_z) < epsilon else thrust_local_z

    thrust_intermediate_x = thrust_local_x * np.cos(ry) - thrust_local_z * np.sin(ry)
    thrust_intermediate_z = thrust_local_x * np.sin(ry) + thrust_local_z * np.cos(ry)

    # Rotation about PITCH (X-axis)
    thrust_global_x = thrust_intermediate_x
    thrust_global_y = thrust_local_y * np.cos(rp) - thrust_intermediate_z * np.sin(rp)
    thrust_global_z = thrust_local_y * np.sin(rp) + thrust_intermediate_z * np.cos(rp)

    # return thrust_global_x, thrust_global_y, thrust_global_z
    return thrust_local_x, thrust_local_y, thrust_local_z

if __name__ == "__main__":

    print(calculate_mass_flow_rate(nasa_cea=True))
    print(calculate_mass_flow_rate(choked_flow=True))
    print(calculate_mass_flow_rate())
