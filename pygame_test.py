import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math
import rocket_physics as rp
import thrust
from rocket import Rocket



# Pygame Initialization
pygame.init()
width, height = 800, 600
screen = pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
pygame.display.set_caption("Real-Time 3D Rocket Simulation")

# OpenGL Setup
glEnable(GL_DEPTH_TEST)
gluPerspective(60, width / height, 0.1, 1000.0)  # Increase FOV & max distance
glTranslatef(0, -5, -100)  # Move the camera further back


# Initialize Rocket
rocket = Rocket()

# Control Parameters
max_gimbal_rate = 20  # Degrees per second (limits pitch/yaw changes)
running = True
clock = pygame.time.Clock()

# Function to Render 3D Rocket Model
# Store past positions for trajectory visualization
trajectory = []

stars = [(np.random.uniform(-500, 500), np.random.uniform(100, 500), np.random.uniform(-500, 500)) for _ in range(200)]


from OpenGL.GLU import *

from OpenGL.GLU import *

def draw_rocket():
    """ Draws the rocket as a cylinder (body) + cone (nose) with correct orientation. """
    glPushMatrix()

    # Move rocket to its current position
    glTranslatef(rocket.x_position, rocket.y_position, rocket.z_position)
    # Apply rotations in correct order: Yaw (Y), Pitch (X), Roll (Z)
    glRotatef(rocket.thrust_yaw, 0, 0, 1)  # Rotate around world Y-axis (Yaw)
    glRotatef(rocket.thrust_pitch, 0, 1, 0)  # Rotate around local X-axis (Pitch)

    # Create a quadric object for smooth rendering
    quadric = gluNewQuadric()
    glColor3f(0, 0, 0)  # Black body

    # **1️⃣ Draw Rocket Body (Cylinder)**
    glPushMatrix()
    gluCylinder(quadric, 2, 2, 10, 20, 20) # (quadric, baseRadius, topRadius, height, slices, stacks)
    glPopMatrix()

    # **2️⃣ Draw Rocket Nose (Cone)**
    glPushMatrix()
    glTranslatef(0, 0, 10.0)  # Move cone to top of the cylinder
    glColor3f(1, 0, 0)  # White nose
    gluCylinder(quadric, 2, 0, 5, 20, 20)  # Cone shape
    glPopMatrix()

    glPopMatrix()  # Restore matrix





    # Store trajectory points
    trajectory.append((rocket.x_position, rocket.y_position, rocket.z_position))

    # Limit the number of points in the trajectory for performance reasons
    if len(trajectory) > 1000:
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




def display():
    """ Updates the camera to follow the rocket. """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    draw_background()

    glPushMatrix()

    speed_factor = max(100, abs(rocket.y_velocity) / 5)  # Increase distance at high speeds
    camera_distance = 5#  + speed_factor
    camera_height = 15

    angle_rad = math.radians(rocket.yaw_angle)
    eye_x = rocket.x_position  - camera_distance * math.sin(angle_rad)
    eye_y = rocket.y_position  + camera_height  # Keep camera above the rocket
    eye_z = rocket.z_position  - camera_distance * math.cos(angle_rad)

    gluLookAt(eye_x, eye_y, eye_z,
              rocket.x_position, rocket.y_position, rocket.z_position,
              0, 1, 0)

    # 🛠 Draw Grid First

    draw_stars()
    draw_grid()  # Restore normal rendering
    draw_rocket()
    glPopMatrix()
    pygame.display.flip()



# **Real-Time Simulation Loop**
while running:
    if rocket.y_position < 0:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        pygame.time.wait(100)
        continue

    dt = clock.tick(60) / 1000.0  # Delta time for real-time step

    # Handle Events (Quit & Keyboard Input)
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    # **Read Key Inputs for Thrust Vectoring**
    keys = pygame.key.get_pressed()
    if keys[K_UP]:  # Increase pitch (tilt down)
        rocket.update_thrust_vector(rocket.thrust_pitch - max_gimbal_rate, rocket.thrust_yaw, max_gimbal_rate, dt)
    if keys[K_DOWN]:  # Decrease pitch (tilt up)
        rocket.update_thrust_vector(rocket.thrust_pitch + max_gimbal_rate, rocket.thrust_yaw, max_gimbal_rate, dt)
    if keys[K_LEFT]:  # Yaw left
        rocket.update_thrust_vector(rocket.thrust_pitch, rocket.thrust_yaw - max_gimbal_rate, max_gimbal_rate, dt)
    if keys[K_RIGHT]:  # Yaw right
        rocket.update_thrust_vector(rocket.thrust_pitch, rocket.thrust_yaw + max_gimbal_rate, max_gimbal_rate, dt)

    # **Calculate Rocket Physics**
    thrust_force = rocket.get_thrust()

    pitch_torque, yaw_torque = rocket.get_torque()
    roll_torque = 0  # No roll control for now

    # Compute angular acceleration (rotation effect)
    pitch_accel, yaw_accel, roll_accel = rp.calculate_angular_acceleration(
        rocket.current_mass, rocket.diameter / 2, rocket.length,
        pitch_torque, yaw_torque, roll_torque)

    rocket.update_angular_motion(pitch_accel, yaw_accel, roll_accel, dt)


    # Compute thrust components in x, y, z
    thrust_x, thrust_y, thrust_z = thrust.calculate_thrust_vector_components(
        thrust_force, rocket.pitch_angle, rocket.yaw_angle
    )

    # Compute acceleration (F = ma)
    acceleration_x, acceleration_y, acceleration_z = rp.calculate_linear_acceleration(
        thrust_x, thrust_y, thrust_z, rocket)

    rocket.update_velocity(acceleration_x, acceleration_y, acceleration_z, dt)
    rocket.update_position(dt)

    # **Render Scene with Dynamic Camera**
    display()

pygame.quit()
