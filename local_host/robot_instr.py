MAX_SPEED = 500


class robot_instr:
    
    def __init__(self, sender):
        self._sender = sender
    
    def _clamp(self, value, min_val=-MAX_SPEED, max_val=MAX_SPEED):
        """Clamp a value to the valid speed range."""
        return max(min_val, min(max_val, value))
    
    def _send(self, linear, angular):
        """Send a motor command."""
        linear = self._clamp(linear)
        angular = self._clamp(angular)
        self._sender.forward(f"{linear} {angular}")
    
    def move_forward(self, speed=400):
        self._send(abs(speed), 0)
    
    def move_backward(self, speed=400):
        self._send(-abs(speed), 0)
    
    def rotate_ccw(self, speed=400):
        self._send(0, abs(speed))
    
    def rotate_cw(self, speed=400):
        self._send(0, -abs(speed))
    
    def stop(self):
        """Stop all motors."""
        self._send(0, 0)
    
    def drive(self, linear, angular):
        """
        Combined linear and angular movement
        
        Args:
            linear: Linear speed (-500 to 500, positive=forward)
            angular: Angular speed (-500 to 500, positive=left)
        """
        self._send(linear, angular)
    
    def arc_forward_left(self, linear, angular):
        self._send(abs(linear), abs(angular))
    
    def arc_forward_right(self, linear, angular):
        self._send(abs(linear), -abs(angular))
    
    def arc_backward_left(self, linear, angular):
        self._send(-abs(linear), abs(angular))
    
    def arc_backward_right(self, linear, angular):
        self._send(-abs(linear), -abs(angular))

