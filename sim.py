import numpy as np
import matplotlib.pyplot as plt

# Set the size of the environment
environment_size = (50, 50)

# Initialize the occupancy grid to -1, meaning unknown
occupancy_grid = -1 * np.ones(environment_size)

# Fake obstacles in the environment (we'll use a simple static layout)
obstacles = np.zeros(environment_size)
obstacles[20:30, 20:30] = 1  # A square obstacle
obstacles[40:43, 40:43] = 1  # Additional small obstacle in the upper right corner

# Simulate the LiDAR 'detection' of obstacles
def simulate_lidar_detection(pose, environment, range_of_view=1):
    x, y, theta = pose
    readings = []

    # Check the surrounding cells within the range_of_view
    for dx in range(-range_of_view, range_of_view + 1):
        for dy in range(-range_of_view, range_of_view + 1):
            new_x, new_y = int(x + dx), int(y + dy)
            if (0 <= new_x < environment.shape[0]) and (0 <= new_y < environment.shape[1]):
                if environment[new_x, new_y] == 1:
                    readings.append((new_x, new_y))
    return readings

# Update the occupancy grid based on the sensor readings
def update_occupancy_grid(grid, pose, readings, range_of_view=1):
    x, y, theta = pose

    # First, update the cells within the robot's range of view to free (0)
    for dx in range(-range_of_view, range_of_view + 1):
        for dy in range(-range_of_view, range_of_view + 1):
            new_x, new_y = int(x + dx), int(y + dy)
            if (0 <= new_x < grid.shape[0]) and (0 <= new_y < grid.shape[1]):
                grid[new_x, new_y] = 0

    # Then, update the cells where obstacles are detected to occupied (1)
    for (ox, oy) in readings:
        grid[ox, oy] = 1

# Run the simulation
def run_simulation(steps=35):
    # Initialize the robot's starting position
    robot_pose = (25, 25, 0)  # Start in the middle of the grid, facing up
    viewRange = 2
    
    
    for _ in range(steps):  # Move for a specified number of steps
        # Get the LiDAR readings
        readings = simulate_lidar_detection(robot_pose, obstacles, viewRange)
        
        # Update the occupancy grid
        update_occupancy_grid(occupancy_grid, robot_pose, readings, viewRange)
        
        # Plot the current state of the occupancy grid
        plt.imshow(occupancy_grid, cmap='gray', interpolation='nearest', origin='lower')
        plt.draw()
        plt.pause(0.1)
        
        # For simplicity, let's move the robot in a simple square pattern
        x, y, theta = robot_pose
        if _ % steps < steps // 4:
            robot_pose = (x, y + 1, theta)  # Move up
        elif _ % steps < steps // 2:
            robot_pose = (x + 1, y, theta)  # Move right
        elif _ % steps < 3 * (steps // 4):
            robot_pose = (x, y - 1, theta)  # Move down
        else:
            robot_pose = (x - 1, y, theta)  # Move left

    plt.show()

# Run the simulation for 65 steps
run_simulation(65)
