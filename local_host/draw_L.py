import turtle
import time
from datasender import datasender

# Robot configuration
ROBOT_IP = "10.59.35.74"
ROBOT_PORT = 5555
DEFAULT_SPEED = 300

# Timing configuration 
FORWARD_TIME = 1.3     
ROTATE_TIME = 0.7   
COMMAND_INTERVAL = 0.05  

# Turtle simulation scale 
TURTLE_FORWARD_DISTANCE = 100  # pixels for each forward segment

# Swerve drive parameters 
TURNING_RADIUS = 50  


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
    t.speed(1)  
    
    return screen, t 


def send_command_for_duration(robot, command_func, duration, t=None, turtle_action=None):
    start_time = time.time()
    if t and turtle_action:
        action, value = turtle_action
        if action == "forward":
            steps = int(duration / COMMAND_INTERVAL)
            step_distance = value / steps if steps > 0 else value
            for _ in range(steps):
                command_func()  
                t.forward(step_distance)
                time.sleep(COMMAND_INTERVAL)
        elif action == "arc_left":
            arc_length = (value * 3.14159 * TURNING_RADIUS) / 180.0
            steps = int(duration / COMMAND_INTERVAL)
            step_angle = value / steps if steps > 0 else value
            step_distance = arc_length / steps if steps > 0 else arc_length
            for _ in range(steps):
                command_func() 
                t.forward(step_distance)
                t.left(step_angle)
                time.sleep(COMMAND_INTERVAL)
        elif action == "arc_right":
            arc_length = (value * 3.14159 * TURNING_RADIUS) / 180.0
            steps = int(duration / COMMAND_INTERVAL)
            step_angle = value / steps if steps > 0 else value
            step_distance = arc_length / steps if steps > 0 else arc_length
            for _ in range(steps):
                command_func()  
                t.forward(step_distance)
                t.right(step_angle)
                time.sleep(COMMAND_INTERVAL)
    else:
        while time.time() - start_time < duration:
            command_func()
            time.sleep(COMMAND_INTERVAL)


def draw_l_shape():
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
        
        # Step 2: Rotate counter-clockwise 90 degrees 
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
        screen.mainloop()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        robot.stop()
    finally:
        sender.close()
        print("Connection closed.")


if __name__ == "__main__":
    draw_l_shape()

