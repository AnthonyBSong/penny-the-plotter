"""
Draw path simulation with turtle graphics while controlling the real robot.
This script sends commands to the robot and simultaneously draws the path on screen.
"""

import turtle
import time
from datasender import datasender

# Robot configuration
ROBOT_IP = "10.59.35.74"
ROBOT_PORT = 5555
DEFAULT_SPEED = 300

# Timing configuration (adjust based on your robot's actual movement)
FORWARD_TIME = 1.3     # seconds to move forward
ROTATE_TIME = 0.7       # seconds to rotate 90 degrees
COMMAND_INTERVAL = 0.05  # send commands every 100ms

# Turtle simulation scale (pixels per "unit" of movement)
TURTLE_FORWARD_DISTANCE = 100  # pixels for each forward segment

# Swerve drive parameters (for curved rotation simulation)
# Approximate turning radius in pixels (adjust based on your robot's track width)
TURNING_RADIUS = 50  # pixels - radius of the arc when rotating


def setup_turtle():
    """Initialize turtle graphics."""
    screen = turtle.Screen()
    screen.title("Penny the Plotter - Path Simulation")
    screen.bgcolor("#1a1a2e")
    
    t = turtle.Turtle()
    t.shape("turtle")
    t.color("#00d4ff")
    t.pencolor("#ff6b6b")
    t.pensize(3)
    t.speed(1)  # Slow animation to match robot movement
    
    return screen, t 


def send_command_for_duration(robot, command_func, duration, t=None, turtle_action=None):
    """
    Send a command repeatedly for a specified duration.
    Optionally perform turtle animation simultaneously.
    
    Args:
        robot: robot_instr instance
        command_func: function to call (e.g., robot.move_forward)
        duration: how long to send commands (seconds)
        t: turtle instance (optional)
        turtle_action: tuple of (action, value) for turtle (optional)
            - ("forward", distance): move forward
            - ("arc_left", degrees): rotate left in an arc (swerve drive)
            - ("arc_right", degrees): rotate right in an arc (swerve drive)
    """
    start_time = time.time()
    
    # If turtle animation is provided, do both animation and command sending together
    if t and turtle_action:
        action, value = turtle_action
        if action == "forward":
            # Animate forward movement smoothly while continuously sending commands
            steps = int(duration / COMMAND_INTERVAL)
            step_distance = value / steps if steps > 0 else value
            for _ in range(steps):
                command_func()  # Send command continuously
                t.forward(step_distance)
                time.sleep(COMMAND_INTERVAL)
        elif action == "arc_left":
            # Draw a curved arc for left rotation (swerve drive) while sending commands
            arc_length = (value * 3.14159 * TURNING_RADIUS) / 180.0
            steps = int(duration / COMMAND_INTERVAL)
            step_angle = value / steps if steps > 0 else value
            step_distance = arc_length / steps if steps > 0 else arc_length
            
            # Draw arc by moving forward while turning, sending commands continuously
            for _ in range(steps):
                command_func()  # Send command continuously
                t.forward(step_distance)
                t.left(step_angle)
                time.sleep(COMMAND_INTERVAL)
        elif action == "arc_right":
            # Draw a curved arc for right rotation (swerve drive) while sending commands
            arc_length = (value * 3.14159 * TURNING_RADIUS) / 180.0
            steps = int(duration / COMMAND_INTERVAL)
            step_angle = value / steps if steps > 0 else value
            step_distance = arc_length / steps if steps > 0 else arc_length
            
            # Draw arc by moving forward while turning, sending commands continuously
            for _ in range(steps):
                command_func()  # Send command continuously
                t.forward(step_distance)
                t.right(step_angle)
                time.sleep(COMMAND_INTERVAL)
    else:
        # No animation - just send commands continuously
        while time.time() - start_time < duration:
            command_func()
            time.sleep(COMMAND_INTERVAL)


def draw_l_shape():
    """
    Draw an L-shape: forward, rotate left 90°, forward.
    Both on turtle simulation and the real robot.
    """
    # Setup
    screen, t = setup_turtle()
    
    print(f"Connecting to robot at {ROBOT_IP}:{ROBOT_PORT}...")
    sender = datasender(ROBOT_IP, ROBOT_PORT)
    robot = sender.robot
    
    print("\n=== Starting L-Shape Path ===\n")
    
    try:
        # Step 1: Move forward
        print("Step 1: Moving forward...")
        send_command_for_duration(robot, robot.move_forward, FORWARD_TIME, 
                                  t, ("forward", TURTLE_FORWARD_DISTANCE))
        
        # Brief stop
        print("Stopping...")
        robot.stop()
        time.sleep(0.5)
        
        # Step 2: Rotate counter-clockwise 90 degrees (as a curved arc for swerve drive)
        print("Step 2: Rotating CCW 90° (curved arc)...")
        send_command_for_duration(robot, robot.rotate_ccw, ROTATE_TIME,
                                  t, ("arc_left", 90))
        
        # Brief stop
        print("Stopping...")
        robot.stop()
        time.sleep(0.5)
        
        # Step 3: Move forward again
        print("Step 3: Moving forward...")
        send_command_for_duration(robot, robot.move_forward, FORWARD_TIME,
                                  t, ("forward", TURTLE_FORWARD_DISTANCE))
        
        # Final stop
        print("Final stop...")
        robot.stop()
        
        print("\n=== Path Complete! ===")
        print("Close the turtle window to exit.")
        
        # Keep window open
        screen.mainloop()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        robot.stop()
    finally:
        sender.close()
        print("Connection closed.")


if __name__ == "__main__":
    draw_l_shape()

