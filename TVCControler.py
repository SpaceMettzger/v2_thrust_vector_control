import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt


class TVCController:
    def __init__(self, Kp, Ki, Kd, Kv):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kv = Kv  # New: velocity damping factor
        self.prev_error = 0
        self.integral = 0
        self.prev_deflection = 0

    def compute_control(self, target_angle, current_angle, angular_velocity, max_angle, max_rate, timestep):
        error = target_angle - current_angle
        self.integral += error * timestep
        derivative = (error - self.prev_error) / timestep
        self.prev_error = error

        raw_deflection = (self.Kp * error +
                          self.Ki * self.integral +
                          self.Kd * derivative -
                          self.Kv * angular_velocity)  # Velocity damping

        raw_deflection = np.clip(raw_deflection, -max_angle, max_angle)

        max_step = max_rate * timestep
        deflection = np.clip(raw_deflection, self.prev_deflection - max_step, self.prev_deflection + max_step)

        self.prev_deflection = deflection
        return deflection


class TVC_MPC:
    def __init__(self, dt=0.1, horizon=10):
        self.dt = dt
        self.horizon = horizon  # Prediction steps

    def compute_control(self, target_pitch, current_pitch, pitch_velocity,
                               target_yaw, current_yaw, yaw_velocity, max_angle, max_rate):
        """Solves MPC for both pitch and yaw simultaneously."""
        N = self.horizon  # Prediction horizon

        # Decision variables: thrust deflection angles over the horizon
        u_pitch = cp.Variable(N)
        u_yaw = cp.Variable(N)

        # State variables (pitch/yaw angles and velocities)
        theta_pitch = cp.Variable(N+1)  # Predicted pitch angles
        omega_pitch = cp.Variable(N+1)  # Predicted pitch velocities
        theta_yaw = cp.Variable(N+1)    # Predicted yaw angles
        omega_yaw = cp.Variable(N+1)    # Predicted yaw velocities

        # Initial conditions
        theta_pitch_0 = current_pitch
        omega_pitch_0 = pitch_velocity
        theta_yaw_0 = current_yaw
        omega_yaw_0 = yaw_velocity

        # Target angles over the horizon
        theta_pitch_target = np.full(N+1, target_pitch)
        theta_yaw_target = np.full(N+1, target_yaw)

        # Cost function: Minimize deviation + smooth control
        cost = (
            cp.sum_squares(theta_pitch - theta_pitch_target) +
            cp.sum_squares(theta_yaw - theta_yaw_target) +
            0.1 * cp.sum_squares(u_pitch) +
            0.1 * cp.sum_squares(u_yaw) +
            0.5 * cp.sum_squares(omega_pitch) +
            0.5 * cp.sum_squares(omega_yaw)
        )

        # Constraints
        constraints = [
            theta_pitch[0] == theta_pitch_0,  # Initial pitch
            omega_pitch[0] == omega_pitch_0,  # Initial pitch velocity
            theta_yaw[0] == theta_yaw_0,      # Initial yaw
            omega_yaw[0] == omega_yaw_0       # Initial yaw velocity
        ]

        for k in range(N):
            # Update pitch dynamics
            constraints.append(theta_pitch[k+1] == theta_pitch[k] + omega_pitch[k] * self.dt)
            constraints.append(omega_pitch[k+1] == omega_pitch[k] + u_pitch[k] * self.dt)

            # Update yaw dynamics
            constraints.append(theta_yaw[k+1] == theta_yaw[k] + omega_yaw[k] * self.dt)
            constraints.append(omega_yaw[k+1] == omega_yaw[k] + u_yaw[k] * self.dt)

            # Control constraints
            constraints.append(u_pitch[k] >= -max_angle)
            constraints.append(u_pitch[k] <= max_angle)
            constraints.append(u_yaw[k] >= -max_angle)
            constraints.append(u_yaw[k] <= max_angle)

            # Rate limit constraints (∆u ≤ max_rate * dt)
            if k > 0:
                constraints.append(cp.abs(u_pitch[k] - u_pitch[k-1]) <= max_rate * self.dt)
                constraints.append(cp.abs(u_yaw[k] - u_yaw[k-1]) <= max_rate * self.dt)

        # Solve optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        if problem.status not in ["optimal", "optimal_inaccurate"]:
            print("MPC did not find an optimal solution.")
            return 0, 0  # Fail-safe return

        return u_pitch.value[0], u_yaw.value[0]  # Apply first control actions

# === TESTING ===
if __name__ == "__main__":
    dt = 0.1  # Time step (100ms)
    steps = 100  # Simulation duration
    target_pitch = 10  # Target pitch angle
    target_yaw = 5  # Target yaw angle

    mpc = TVC_MPC(dt=dt, horizon=10)

    # Simulated rocket state
    pitch_angles = []
    pitch_velocities = []
    yaw_angles = []
    yaw_velocities = []
    pitch_controls = []
    yaw_controls = []

    current_pitch = 0
    current_pitch_velocity = 0
    current_yaw = 0
    current_yaw_velocity = 0

    for t in range(steps):
        control_pitch, control_yaw = mpc.compute_control(
            target_pitch, current_pitch, current_pitch_velocity,
            target_yaw, current_yaw, current_yaw_velocity
        )

        # Apply controls (simulate motion)
        current_pitch_velocity += control_pitch * dt
        current_pitch += current_pitch_velocity * dt
        current_yaw_velocity += control_yaw * dt
        current_yaw += current_yaw_velocity * dt

        pitch_angles.append(current_pitch)
        pitch_velocities.append(current_pitch_velocity)
        yaw_angles.append(current_yaw)
        yaw_velocities.append(current_yaw_velocity)
        pitch_controls.append(control_pitch)
        yaw_controls.append(control_yaw)

    # Plot results
    plt.figure(figsize=(10, 6))

    plt.subplot(3, 1, 1)
    plt.plot(pitch_angles, label="Rocket Pitch")
    plt.axhline(target_pitch, color='r', linestyle="--", label="Target Pitch")
    plt.plot(yaw_angles, label="Rocket Yaw")
    plt.axhline(target_yaw, color='g', linestyle="--", label="Target Yaw")
    plt.ylabel("Angles (°)")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(pitch_velocities, label="Pitch Velocity")
    plt.plot(yaw_velocities, label="Yaw Velocity")
    plt.ylabel("Velocity (°/s)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(pitch_controls, label="Pitch Thruster Deflection")
    plt.plot(yaw_controls, label="Yaw Thruster Deflection")
    plt.ylabel("Deflection (°)")
    plt.xlabel("Time Steps")
    plt.legend()

    plt.show()
