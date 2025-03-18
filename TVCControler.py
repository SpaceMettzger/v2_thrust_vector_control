import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt


# noinspection PyPep8Naming
class TVC_PID:
    def __init__(self, Kp, Ki, Kd, Kv, dt=1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Kv = Kv
        self.dt = dt
        self.prev_error_pitch = 0
        self.prev_error_yaw = 0
        self.integral_pitch = 0
        self.integral_yaw = 0
        self.prev_deflection_pitch = 0
        self.prev_deflection_yaw = 0

    def compute_control(self, target_pitch, current_pitch, pitch_velocity,
                        target_yaw, current_yaw, yaw_velocity, max_rate, max_angle):
        """Computes thruster deflection for pitch and yaw using PID"""

        error_pitch = target_pitch - current_pitch
        self.integral_pitch += error_pitch * self.dt
        derivative_pitch = (error_pitch - self.prev_error_pitch) / self.dt
        self.prev_error_pitch = error_pitch

        raw_deflection_pitch = (self.Kp * error_pitch +
                                self.Ki * self.integral_pitch +
                                self.Kd * derivative_pitch -
                                self.Kv * pitch_velocity)

        error_yaw = target_yaw - current_yaw
        self.integral_yaw += error_yaw * self.dt
        derivative_yaw = (error_yaw - self.prev_error_yaw) / self.dt
        self.prev_error_yaw = error_yaw

        raw_deflection_yaw = (self.Kp * error_yaw +
                              self.Ki * self.integral_yaw +
                              self.Kd * derivative_yaw -
                              self.Kv * yaw_velocity)

        raw_deflection_pitch = np.clip(raw_deflection_pitch, -max_angle, max_angle)
        raw_deflection_yaw = np.clip(raw_deflection_yaw, -max_angle, max_angle)

        max_step = max_rate * self.dt
        deflection_pitch = np.clip(raw_deflection_pitch,
                                   self.prev_deflection_pitch - max_step,
                                   self.prev_deflection_pitch + max_step)
        deflection_yaw = np.clip(raw_deflection_yaw,
                                 self.prev_deflection_yaw - max_step,
                                 self.prev_deflection_yaw + max_step)

        self.prev_deflection_pitch = deflection_pitch
        self.prev_deflection_yaw = deflection_yaw

        return deflection_pitch, deflection_yaw


class TVC_MPC:
    def __init__(self, dt=0.1, horizon=10):
        self.dt = dt
        self.horizon = horizon

    def compute_control(self, target_pitch, current_pitch, pitch_velocity,
                        target_yaw, current_yaw, yaw_velocity, max_angle, max_rate):
        """Solves MPC for pitch and yaw."""
        N = self.horizon

        u_pitch = cp.Variable(N)
        u_yaw = cp.Variable(N)

        theta_pitch = cp.Variable(N+1)
        omega_pitch = cp.Variable(N+1)
        theta_yaw = cp.Variable(N+1)
        omega_yaw = cp.Variable(N+1)

        theta_pitch_0 = current_pitch
        omega_pitch_0 = pitch_velocity
        theta_yaw_0 = current_yaw
        omega_yaw_0 = yaw_velocity

        theta_pitch_target = np.full(N+1, target_pitch)
        theta_yaw_target = np.full(N+1, target_yaw)

        cost = (
            cp.sum_squares(theta_pitch - theta_pitch_target) +
            cp.sum_squares(theta_yaw - theta_yaw_target) +
            0.1 * cp.sum_squares(u_pitch) +
            0.1 * cp.sum_squares(u_yaw) +
            0.5 * cp.sum_squares(omega_pitch) +
            0.5 * cp.sum_squares(omega_yaw)
        )

        constraints = [
            theta_pitch[0] == theta_pitch_0,
            omega_pitch[0] == omega_pitch_0,
            theta_yaw[0] == theta_yaw_0,
            omega_yaw[0] == omega_yaw_0
        ]

        for k in range(N):
            constraints.append(theta_pitch[k+1] == theta_pitch[k] + omega_pitch[k] * self.dt)
            constraints.append(omega_pitch[k+1] == omega_pitch[k] + u_pitch[k] * self.dt)

            constraints.append(theta_yaw[k+1] == theta_yaw[k] + omega_yaw[k] * self.dt)
            constraints.append(omega_yaw[k+1] == omega_yaw[k] + u_yaw[k] * self.dt)

            constraints.append(u_pitch[k] >= -max_angle)
            constraints.append(u_pitch[k] <= max_angle)
            constraints.append(u_yaw[k] >= -max_angle)
            constraints.append(u_yaw[k] <= max_angle)

            if k > 0:
                constraints.append(cp.abs(u_pitch[k] - u_pitch[k-1]) <= max_rate * self.dt)
                constraints.append(cp.abs(u_yaw[k] - u_yaw[k-1]) <= max_rate * self.dt)

        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        if problem.status not in ["optimal", "optimal_inaccurate"]:
            print("MPC did not find an optimal solution.")
            return 0, 0  # Fail-safe return

        return u_pitch.value[0], u_yaw.value[0]

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
    max_deflection_angle = 30
    max_deflection_rate = 20

    for t in range(steps):
        control_pitch, control_yaw = mpc.compute_control(
            target_pitch, current_pitch, current_pitch_velocity,
            target_yaw, current_yaw, current_yaw_velocity, max_deflection_angle, max_deflection_rate
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
