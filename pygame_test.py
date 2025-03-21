import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math
import rocket_physics as rp
from rocket import Rocket
from plotter import plot


def draw_reference_rings(center, radius=15, segments=100):
    """ Draws 3 axis-aligned rings (X, Y, Z) centered at the given position. """
    x, y, z = center
    glLineWidth(1)

    # Circle in XY plane (Z-axis normal)
    glColor3f(1, 0, 0)  # Red
    glBegin(GL_LINE_LOOP)
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        glVertex3f(x + radius * math.cos(theta), y + radius * math.sin(theta), z)
    glEnd()

    # Circle in XZ plane (Y-axis normal)
    glColor3f(0, 1, 0)  # Green
    glBegin(GL_LINE_LOOP)
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        glVertex3f(x + radius * math.cos(theta), y, z + radius * math.sin(theta))
    glEnd()

    # Circle in YZ plane (X-axis normal)
    glColor3f(0, 0.5, 1)  # Blue
    glBegin(GL_LINE_LOOP)
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        glVertex3f(x, y + radius * math.cos(theta), z + radius * math.sin(theta))
    glEnd()


def draw_rocket():
    """ Draws the rocket as a cylinder (body) + cone (nose) with correct orientation. """
    glPushMatrix()
    draw_reference_rings((rocket.x_position, rocket.y_position, rocket.z_position))

    scale = 3
    glColor3f(0, 1, 0)
    glBegin(GL_LINES)
    glVertex3f(rocket.x_position, rocket.y_position, rocket.z_position)
    glVertex3f(rocket.x_position + rocket.global_thrust_vector[0] * scale,
               rocket.y_position + rocket.global_thrust_vector[1] * scale,
               rocket.z_position + rocket.global_thrust_vector[2] * scale)
    glEnd()
    glColor3f(1, 0, 0)
    glBegin(GL_LINES)
    glVertex3f(rocket.x_position, rocket.y_position, rocket.z_position)
    glVertex3f(rocket.x_position + rocket.x_velocity * scale,
               rocket.y_position + rocket.y_velocity * scale,
               rocket.z_position + rocket.z_velocity * scale)
    glEnd()

    glTranslatef(rocket.x_position, rocket.y_position, rocket.z_position)
    glRotatef(rocket.yaw_angle, 1, 0, 0)  # Rotate around world Y-axis (Yaw)
    glRotatef(-rocket.pitch_angle - 90, 0, 1, 0)  # Rotate around local X-axis (Pitch)

    quadric = gluNewQuadric()
    glColor3f(0, 0, 0)  # Black body

    # **1️⃣ Draw Rocket Body (Cylinder)**
    glPushMatrix()
    gluCylinder(quadric, 2, 2, 10, 20, 20)  # (quadric, baseRadius, topRadius, height, slices, stacks)
    glPopMatrix()

    glBegin(GL_LINES)
    # **Red Pitch Axis Line (Forward direction)**
    glColor3f(1, 0, 0)  # Red
    glVertex3f(0, -5, 0)  # Bottom of rocket
    glVertex3f(0, 5, 0)  # Extends downward

    # **Green Thrust Axis Line (Thrust direction)**
    glColor3f(0, 1, 0)  # Green
    glVertex3f(-5, 0, 0)  # Bottom of rocket
    glVertex3f(5, 0, 0)  # Extends downward
    glEnd()

    # Rocket Nose
    glPushMatrix()
    glTranslatef(0, 0, 10.0)  # Move cone to top of the cylinder
    glColor3f(1, 0, 0)  # Red nose
    gluCylinder(quadric, 2, 0, 5, 20, 20)  # Cone shape
    glPopMatrix()

    # Rocket_thruster
    glPushMatrix()
    glRotatef(- rocket.thrust_yaw_local, 1, 0, 0)  # Rotate around world Y-axis (Yaw)
    glRotatef(rocket.thrust_pitch_local, 0, 1, 0)  # Rotate around local X-axis (Pitch)
    glPushMatrix()
    glTranslatef(0, 0, -5)  # Move cone to bottom of the cylinder
    glColor3f(0.5, 0.7, 1)  # Blue cone
    gluCylinder(quadric, 2, 1, 5, 10, 10)  # Cone shape
    glPopMatrix()
    glPopMatrix()

    glPopMatrix()

    trajectory = rocket.records["lateral_positions"]
    # Limit the number of points in the trajectory for performance reasons
    if len(trajectory) > 10000:
        trajectory.pop(0)

    # Draw trajectory as fading points
    glPointSize(4)
    glBegin(GL_POINTS)
    for i, (x, y, z) in enumerate(trajectory):
        glColor4f(1, 1, 1, i / len(trajectory))  # Fading effect
        glVertex3f(x, y, z)
    glEnd()


def draw_grid():
    """ Draws a large ground grid for reference. """
    glPushMatrix()
    glRotatef(90, 1, 0, 0)  # Rotate around local X-axis (Pitch)

    glColor3f(0.3, 0.3, 0.3)  # Light grey
    glBegin(GL_LINES)

    # Grid size and spacing
    grid_size = 10000  # Increase to make it visible
    spacing = 5  # Distance between grid lines

    # Draw vertical and horizontal lines
    for i in range(-grid_size, grid_size + 1, spacing):
        glVertex3f(i, -1, -grid_size)  # Move slightly up (-1 instead of -5)
        glVertex3f(i, -1, grid_size)

        glVertex3f(-grid_size, -1, i)
        glVertex3f(grid_size, -1, i)

    glEnd()
    glPopMatrix()


def draw_stars():
    """ Draws stars in the background for reference. """
    glPointSize(2)
    glBegin(GL_POINTS)
    glColor3f(1, 1, 1)
    for x, y, z in stars:
        glVertex3f(x, y, z)
    glEnd()


def draw_background():
    """ Draws a full-screen gradient background. """
    glMatrixMode(GL_PROJECTION)  # Switch to projection mode
    glPushMatrix()
    glLoadIdentity()
    glOrtho(-1, 1, -1, 1, -1, 1)  # Set orthographic projection

    glMatrixMode(GL_MODELVIEW)  # Switch back to model view
    glPushMatrix()
    glLoadIdentity()

    glDisable(GL_DEPTH_TEST)  # Ensure it's always behind everything
    glBegin(GL_QUADS)

    # Top color (Bright Blue Sky)
    glColor3f(0.1, 0.3, 0.8)
    glVertex2f(-1, 1)
    glVertex2f(1, 1)

    # Bottom color (Dark Space)
    glColor3f(0, 0, 0.2)
    glVertex2f(1, -1)
    glVertex2f(-1, -1)

    glEnd()
    glEnable(GL_DEPTH_TEST)  # Restore depth testing

    # Restore previous projection
    glPopMatrix()
    glMatrixMode(GL_PROJECTION)
    glPopMatrix()
    glMatrixMode(GL_MODELVIEW)  # Back to model view mode


def draw_text(font, text, x, y, align="left", color=(255, 255, 255)):
    """Draw text using Pygame on top of OpenGL with alignment options."""
    text_surface = font.render(text, True, color)
    text_data = pygame.image.tostring(text_surface, "RGBA", True)

    text_width = text_surface.get_width()

    if align == "right":
        x -= text_width
    elif align == "center":
        x -= text_width // 2

    glWindowPos2d(x, y)
    glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)


def draw_controls(font):
    draw_text(font, "Mouse: Move camera", 10, screen_height - 20)
    draw_text(font, "W/S/A/D: Thrust vectoring", 10, screen_height - 40)
    draw_text(font, "Space: Launch", 10, screen_height - 60)


def draw_stats(font, rocket):
    draw_text(font, f"Speed: {int(rocket.total_lateral_velocity)} m/s", screen_width - 10, screen_height - 20, align="right")
    draw_text(font, f"Altitude {int(rocket.z_position)} m", screen_width - 10, screen_height - 40, align="right")
    draw_text(font, f"Remaining Fuel {int(rocket.current_fuel)} kg", screen_width - 10, screen_height - 60, align="right")
    draw_text(font, f"Thrust {int(np.linalg.norm(rocket.global_thrust_vector) )} N", screen_width - 10, screen_height - 80, align="right")


def display(font, rocket):
    """ Updates the camera to follow the rocket. """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    draw_background()

    glPushMatrix()

    center_of_mass = rocket.length - rp.calculate_center_of_mass(rocket)

    height_factor = min(250, abs(rocket.z_position) / 5)  # Increase distance at high altitude

    camera_distance = 5 + height_factor
    camera_height = 15 + height_factor

    yaw_camera_angle_rad = np.deg2rad(camera_angle_yaw)
    pitch_camera_angle_rad = np.deg2rad(camera_angle_pitch)

    eye_x = rocket.x_position + camera_distance * math.cos(yaw_camera_angle_rad)
    eye_y = rocket.y_position + camera_distance * math.sin(yaw_camera_angle_rad)

    eye_z = rocket.z_position + camera_height * math.sin(pitch_camera_angle_rad)

    gluLookAt(eye_x, eye_y, eye_z,
              rocket.x_position, rocket.y_position, rocket.z_position + center_of_mass,
              0, 0, 1)

    draw_stars()
    draw_grid()
    draw_rocket()
    draw_controls(font)
    draw_stats(font, rocket)

    glPopMatrix()
    pygame.display.flip()


if __name__ == "__main__":
    pygame.init()
    screen_width, screen_height = 800, 600
    screen = pygame.display.set_mode((screen_width, screen_height), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Real-Time 3D Rocket Simulation")

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60, screen_width / screen_height, 0.1, 1000.0)  # Increase FOV & max distance
    glTranslatef(0, -5, -100)

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    pygame.font.init()
    font = pygame.font.SysFont("Arial", 18)

    # Initialize Rocket
    rocket = Rocket()
    max_gimbal_rate = 20  # Degrees per second (limits pitch/yaw changes)

    camera_angle_pitch = 0
    camera_angle_yaw = 0
    mouse_x, mouse_y = 0, 0
    launched = False
    running = True
    clock = pygame.time.Clock()
    time_values = []
    t = 0

    stars = [(np.random.uniform(-500, 500), np.random.uniform(100, 500), np.random.uniform(-500, 500)) for _ in
             range(200)]

    while running:
        pygame.event.set_grab(True)
        pygame.mouse.set_visible(False)  # Hide the cursor

        if rocket.z_position < 0:
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)
            running = False
            continue

        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        keys = pygame.key.get_pressed()
        if keys[K_SPACE]:
            launched = True

        mouse_x, mouse_y = pygame.mouse.get_rel()
        camera_angle_pitch += mouse_y / 10
        camera_angle_yaw -= mouse_x / 10

        return_rate = max_gimbal_rate * dt * 0.5
        if keys[K_a]:
            rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local - (max_gimbal_rate * dt),
                                                     rocket.thrust_yaw_local, dt)
        if keys[K_d]:
            rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local + (max_gimbal_rate * dt),
                                                     rocket.thrust_yaw_local, dt)
        if keys[K_s]:
            rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local,
                                                     rocket.thrust_yaw_local - (max_gimbal_rate * dt), dt)
        if keys[K_w]:
            rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local,
                                                     rocket.thrust_yaw_local + (max_gimbal_rate * dt), dt)

        if not (keys[K_a] or keys[K_d]) and rocket.thrust_pitch_local > 0:
            rocket.thrust_pitch_local = max(0, rocket.thrust_pitch_local - return_rate)
        elif not (keys[K_a] or keys[K_d]) and rocket.thrust_pitch_local < 0:
            rocket.thrust_pitch_local = min(0, rocket.thrust_pitch_local + return_rate)

        if not (keys[K_w] or keys[K_s]) and rocket.thrust_yaw_local > 0:
            rocket.thrust_yaw_local = max(0, rocket.thrust_yaw_local - return_rate)
        elif not (keys[K_w] or keys[K_s]) and rocket.thrust_yaw_local < 0:
            rocket.thrust_yaw_local = min(0, rocket.thrust_yaw_local + return_rate)

        if launched:
            rp.update_flight_time(rocket, dt)

            thrust_force = rocket.get_thrust()

            pitch_torque, yaw_torque, _ = rp.get_torque(rocket)
            roll_torque = 0

            rp.calculate_angular_acceleration(rocket, pitch_torque, yaw_torque, roll_torque)

            rp.update_angular_motion(rocket, dt)

            thrust_x, thrust_y, thrust_z = rocket.calculate_thruster_deflection_transformation()

            acceleration_x, acceleration_y, acceleration_z = rp.calculate_linear_acceleration(
                thrust_x, thrust_y, thrust_z, rocket)

            rp.update_velocity(rocket, acceleration_x, acceleration_y, acceleration_z, dt)
            rp.update_position(rocket, dt)
            rocket.record_rocket_params()
            t += dt
            time_values.append(t)

        display(font, rocket)

    plot(rocket, time_values, show_orientation = True)
    pygame.quit()
