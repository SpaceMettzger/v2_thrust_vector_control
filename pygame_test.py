import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math
import rocket_physics as rp
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

def draw_rocket():
    """ Draws the rocket as a cylinder (body) + cone (nose) with correct orientation. """
    glPushMatrix()

    glTranslatef(rocket.x_position, rocket.y_position, rocket.z_position)
    glRotatef(rocket.yaw_angle, 1, 0, 0)  # Rotate around world Y-axis (Yaw)
    glRotatef(-rocket.pitch_angle - 90, 0, 1, 0)  # Rotate around local X-axis (Pitch)

    quadric = gluNewQuadric()
    glColor3f(0, 0, 0)  # Black body

    # **1️⃣ Draw Rocket Body (Cylinder)**
    glPushMatrix()
    gluCylinder(quadric, 2, 2, 10, 20, 20) # (quadric, baseRadius, topRadius, height, slices, stacks)
    glPopMatrix()

    glBegin(GL_LINES)
    # **Red Pitch Axis Line (Forward direction)**
    glColor3f(1, 0, 0)  # Red
    glVertex3f(0, 0, 0)  # Bottom of rocket
    glVertex3f(0, 5, 0)  # Extends downward

    # **Green Thrust Axis Line (Thrust direction)**
    glColor3f(0, 1, 0)  # Green
    glVertex3f(0, 0, 0)  # Bottom of rocket
    glVertex3f(5, 0, 0)  # Extends downward
    glEnd()

    # **2️⃣ Draw Rocket Nose (Cone)**
    glPushMatrix()
    glTranslatef(0, 0, 10.0)  # Move cone to top of the cylinder
    glColor3f(1, 0, 0)  # White nose
    gluCylinder(quadric, 2, 0, 5, 20, 20)  # Cone shape
    glPopMatrix()

    # Rocket_thruster
    glPushMatrix()
    glRotatef(rocket.thrust_yaw_local, 0, 0, 1)  # Rotate around world Y-axis (Yaw)
    glRotatef(rocket.thrust_pitch_local, 0, 1, 0)  # Rotate around local X-axis (Pitch)
    glPushMatrix()
    glTranslatef(0, 0, -5)  # Move cone to top of the cylinder
    glColor3f(0.5, 0.7, 1)  # White nose
    gluCylinder(quadric, 2, 1, 5, 10, 10)  # Cone shape
    glPopMatrix()
    glPopMatrix()

    glPopMatrix()

      # Restore matrix

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




def display():
    """ Updates the camera to follow the rocket. """
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    draw_background()

    glPushMatrix()

    speed_factor = max(100, abs(rocket.y_velocity) / 5)  # Increase distance at high speeds
    camera_distance = 5#  + speed_factor
    camera_height = 15

    angle_rad = math.radians(rocket.yaw_angle)
    eye_x = rocket.x_position  - camera_distance # * math.sin(angle_rad)
    eye_y = rocket.y_position  - camera_height  # Keep camera above the rocket
    eye_z = rocket.z_position  - camera_distance # * math.cos(angle_rad)

    gluLookAt(eye_x, eye_y, eye_z,
              rocket.x_position, rocket.y_position, rocket.z_position,
              0, 0, 1)

    draw_stars()
    draw_grid()
    draw_rocket()
    glPopMatrix()
    pygame.display.flip()

while running:
    if rocket.z_position < 0:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        pygame.time.wait(100)
        continue

    dt = clock.tick(60) / 1000.0

    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    keys = pygame.key.get_pressed()
    return_rate = max_gimbal_rate * dt * 0.5
    if keys[K_LEFT]:
        rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local - (max_gimbal_rate * dt),
                                                 rocket.thrust_yaw_local, dt)
    elif keys[K_RIGHT]:
        rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local + (max_gimbal_rate * dt),
                                                 rocket.thrust_yaw_local, dt)
    elif keys[K_DOWN]:
        rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local,
                                                 rocket.thrust_yaw_local - (max_gimbal_rate * dt), dt)
    elif keys[K_UP]:
        rocket.update_thruster_deflection_simple(rocket.thrust_pitch_local,
                                                 rocket.thrust_yaw_local + (max_gimbal_rate * dt), dt)
    else:
        if rocket.thrust_pitch_local > 0:
            rocket.thrust_pitch_local = max(0, rocket.thrust_pitch_local - return_rate)
        elif rocket.thrust_pitch_local < 0:
            rocket.thrust_pitch_local = min(0, rocket.thrust_pitch_local + return_rate)

        if rocket.thrust_yaw_local > 0:
            rocket.thrust_yaw_local = max(0, rocket.thrust_yaw_local - return_rate)
        elif rocket.thrust_yaw_local < 0:
            rocket.thrust_yaw_local = min(0, rocket.thrust_yaw_local + return_rate)

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

    display()

pygame.quit()
