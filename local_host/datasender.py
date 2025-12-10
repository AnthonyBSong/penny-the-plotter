import socket
import time
from InvalidAddressError import *
from robot_instr import robot_instr


class datasender():
    def __init__(self, ip, port):
        InvalidAddressError.validate_ip(ip)
        InvalidAddressError.validate_port(port)

        # Define the IP address and port of the Pico W
        self._pico_ip = ip
        self._pico_port = port

        # Create a UDP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Robot instruction interface
        self.robot = robot_instr(self)

    def forward(self, information):
        # Convert the data to bytes
        data_to_send = f"{information}".encode('utf-8')

        # Send the data to the Pico W
        self._sock.sendto(data_to_send, (self._pico_ip, self._pico_port))

        print(f"Sent: {data_to_send}")

    def close(self):
        self._sock.close()
    

if __name__ == "__main__":
    # Example usage with robot instructions
    sender = datasender("10.59.35.74", 5555)
    
    print("Testing robot commands...")
    
    move_time = time.time()
    curr_time = time.time()

    # Test forward
    while (curr_time - move_time < 2):
        print("Forward...")
        sender.robot.move_forward(500)
        curr_time = time.time()
        time.sleep(0.5)

    stop_time = time.time()

    # Test forward
    while (curr_time - stop_time < 2):
        print("Stop...")
        sender.robot.stop()  
        curr_time = time.time()
        time.sleep(0.5)
    
    sender.close()
    print("Done!")
