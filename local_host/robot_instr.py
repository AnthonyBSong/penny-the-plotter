"""
Robot instruction set for controlling the Penny the Plotter robot.

UDP Command Protocol:
  Format: "<linear_speed> <angular_speed>"
  
  linear_speed:  -500 to 500 (negative=backward, positive=forward)
  angular_speed: -500 to 500 (negative=right, positive=left)
"""

MAX_SPEED = 500


class robot_instr:
    """
    Robot instruction generator for motor control commands.
    
    This class generates command strings that can be sent via UDP
    to control the robot's differential swerve drive.
    """
    
    def __init__(self, sender):
        """
        Initialize with a datasender instance.
        
        Args:
            sender: A datasender instance with a forward() method
        """
        self._sender = sender
    
    def _clamp(self, value, min_val=-MAX_SPEED, max_val=MAX_SPEED):
        """Clamp a value to the valid speed range."""
        return max(min_val, min(max_val, value))
    
    def _send(self, linear, angular):
        """Send a motor command."""
        linear = self._clamp(linear)
        angular = self._clamp(angular)
        self._sender.forward(f"{linear} {angular}")
    
    def move_forward(self, speed):
        """
        Move the robot forward.
        
        Args:
            speed: Forward speed (0 to 500)
        """
        self._send(abs(speed), 0)
    
    def move_backward(self, speed):
        """
        Move the robot backward.
        
        Args:
            speed: Backward speed (0 to 500)
        """
        self._send(-abs(speed), 0)
    
    def rotate_left(self, speed):
        """
        Rotate the robot left (counter-clockwise).
        
        Args:
            speed: Rotation speed (0 to 500)
        """
        self._send(0, abs(speed))
    
    def rotate_right(self, speed):
        """
        Rotate the robot right (clockwise).
        
        Args:
            speed: Rotation speed (0 to 500)
        """
        self._send(0, -abs(speed))
    
    def stop(self):
        """Stop all motors."""
        self._send(0, 0)
    
    def drive(self, linear, angular):
        """
        Combined linear and angular movement.
        
        Args:
            linear: Linear speed (-500 to 500, positive=forward)
            angular: Angular speed (-500 to 500, positive=left)
        """
        self._send(linear, angular)
    
    def arc_forward_left(self, linear, angular):
        """
        Move forward while turning left.
        
        Args:
            linear: Forward speed (0 to 500)
            angular: Turn rate (0 to 500)
        """
        self._send(abs(linear), abs(angular))
    
    def arc_forward_right(self, linear, angular):
        """
        Move forward while turning right.
        
        Args:
            linear: Forward speed (0 to 500)
            angular: Turn rate (0 to 500)
        """
        self._send(abs(linear), -abs(angular))
    
    def arc_backward_left(self, linear, angular):
        """
        Move backward while turning left.
        
        Args:
            linear: Backward speed (0 to 500)
            angular: Turn rate (0 to 500)
        """
        self._send(-abs(linear), abs(angular))
    
    def arc_backward_right(self, linear, angular):
        """
        Move backward while turning right.
        
        Args:
            linear: Backward speed (0 to 500)
            angular: Turn rate (0 to 500)
        """
        self._send(-abs(linear), -abs(angular))

