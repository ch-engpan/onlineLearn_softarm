import socket
import struct
import sys
import numpy as np
import netifaces
import argparse
def quat_to_euler_deg(qx, qy, qz, qw):
    """Convert quaternion to Euler angles in degrees (roll, pitch, yaw)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    # Convert to degrees
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
class MinimalNatNetClient:
    def __init__(self, server_ip="169.254.118.69", multicast_ip="239.255.42.99", interface=None):
        self.server_ip = server_ip
        self.command_port = 1510
        self.data_port = 1511
        self.multicast_ip = multicast_ip
        self.interface = interface
        self.data_socket = None
        self.command_socket = None
        self.rigid_bodies = {'arm_1': 5, 'arm_2': 6}
    def connect(self):
        try:
            # Handle Windows and Linux differently
            if sys.platform == 'win32':
                # On Windows, bind to all interfaces
                self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.command_socket.bind(('', 0))
                self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                print(f"Command socket created and bound to port {self.command_socket.getsockname()[1]}")
                # Create data socket
                self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.data_socket.bind(('', self.data_port))
                print(f"Data socket created and bound to port {self.data_port}")
                # Join multicast group
                mreq = struct.pack("4s4s", socket.inet_aton(self.multicast_ip), socket.inet_aton("0.0.0.0"))
                self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                print(f"Successfully joined multicast group {self.multicast_ip}")
            else:
                # On Linux, use a different approach
                if self.interface is None:
                    self.interface = 'enp0s31f6'  # Default Ethernet interface
                # Get interface IP address
                interface_ip = netifaces.ifaddresses(self.interface)[netifaces.AF_INET][0]['addr']
                print(f"Using Ethernet interface {self.interface} with IP {interface_ip}")
                # Create command socket
                self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.command_socket.bind(('', 0))
                self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                print(f"Command socket created and bound to port {self.command_socket.getsockname()[1]}")
                # Create data socket
                self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                # Bind to all interfaces
                self.data_socket.bind(('', self.data_port))
                print(f"Data socket created and bound to port {self.data_port}")
                # Try different multicast binding approaches
                try:
                    # First try: bind to interface IP
                    mreq = struct.pack("4s4s", socket.inet_aton(self.multicast_ip), socket.inet_aton(interface_ip))
                    self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                except:
                    try:
                        # Second try: bind to all interfaces
                        mreq = struct.pack("4s4s", socket.inet_aton(self.multicast_ip), socket.inet_aton("0.0.0.0"))
                        self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                    except:
                        # Third try: bind to specific interface
                        self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(interface_ip))
                        mreq = struct.pack("4s4s", socket.inet_aton(self.multicast_ip), socket.inet_aton("0.0.0.0"))
                        self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
                print(f"Successfully joined multicast group {self.multicast_ip}")
            # Print network interface information
            print("\nNetwork Interface Information:")
            for interface in netifaces.interfaces():
                addrs = netifaces.ifaddresses(interface)
                if netifaces.AF_INET in addrs:
                    print(f"\nInterface: {interface}")
                    for addr in addrs[netifaces.AF_INET]:
                        print(f"  IP: {addr['addr']}")
                        print(f"  Netmask: {addr['netmask']}")
                        print(f"  Broadcast: {addr.get('broadcast', 'N/A')}")
            return True
        except Exception as e:
            print(f"Error connecting to OptiTrack server: {e}")
            print(f"Error type: {type(e).__name__}")
            import traceback
            traceback.print_exc()
            return False
    def unpack_rigid_bodies(self, data, offset):
        rigid_body_count = struct.unpack('i', data[offset:offset+4])[0]
        offset += 4
        print(f"Total rigid bodies in frame: {rigid_body_count}")
        rigid_bodies_data = []
        for i in range(rigid_body_count):
            # Read ID, position (x,y,z), and rotation (qx,qy,qz,qw)
            values = struct.unpack('i7f', data[offset:offset+32])
            rb_id, x, y, z, qx, qy, qz, qw = values
            offset += 32
            # Skip mean error and tracking flags
            offset += 6
            # Only store data for specified rigid body IDs
            if rb_id in self.rigid_bodies.values():
                print(f"Found tracked rigid body ID: {rb_id}")
                # Convert quaternion to Euler angles
                roll, pitch, yaw = quat_to_euler_deg(qx, qy, qz, qw)
                # Store position and Euler angles
                rigid_bodies_data.append((rb_id, (x, y, z), (roll, pitch, yaw)))
            else:
                print(f"Skipping untracked rigid body ID: {rb_id}")
        return rigid_bodies_data, offset
    def receive_data(self):
        try:
            print("\nWaiting for data...")
            data, addr = self.data_socket.recvfrom(32768)
            print(f"Received {len(data)} bytes from {addr}")
            if len(data) < 4:
                print("Received data too short")
                return None
            message_id = struct.unpack('h', data[0:2])[0]
            packet_size = struct.unpack('h', data[2:4])[0]
            print(f"Message ID: {message_id}, Packet Size: {packet_size}")
            if message_id == 7:  # NAT_FRAMEOFDATA
                offset = 4
                # Skip frame number
                frame_number = struct.unpack('i', data[offset:offset+4])[0]
                offset += 4
                print(f"Frame number: {frame_number}")
                # Skip markersets
                markerset_count = struct.unpack('i', data[offset:offset+4])[0]
                offset += 4
                print(f"Markerset count: {markerset_count}")
                for _ in range(markerset_count):
                    name_len = data[offset:].find(b'\0')
                    offset += name_len + 1
                    marker_count = struct.unpack('i', data[offset:offset+4])[0]
                    offset += 4 + (marker_count * 12)
                # Skip unlabeled markers
                unlabeled_markers = struct.unpack('i', data[offset:offset+4])[0]
                offset += 4 + (unlabeled_markers * 12)
                print(f"Unlabeled markers: {unlabeled_markers}")
                # Process rigid bodies
                rigid_bodies_data, _ = self.unpack_rigid_bodies(data, offset)
                return rigid_bodies_data
            return None
        except Exception as e:
            print(f"Error receiving data: {e}")
            print(f"Error type: {type(e).__name__}")
            import traceback
            traceback.print_exc()
            return None
    def run(self):
        try:
            if self.connect():
                while True:
                    data = self.receive_data()
                    if data:
                        for rb_id, pos, euler in data:
                            print(f"RB ID: {rb_id}")
                            print(f"Position (x,y,z): {pos[0]:.12f}, {pos[1]:.12f}, {pos[2]:.12f}")
                            print(f"Rotation (roll,pitch,yaw): {euler[0]:.12f}°, {euler[1]:.12f}°, {euler[2]:.12f}°\n")
        except KeyboardInterrupt:
            if self.data_socket:
                self.data_socket.close()
            if self.command_socket:
                self.command_socket.close()
def main():
    parser = argparse.ArgumentParser(description='OptiTrack Client')
    parser.add_argument('--server-ip', type=str, default="169.254.118.69",
                      help='OptiTrack server IP address')
    parser.add_argument('--multicast-ip', type=str, default="239.255.42.99",
                      help='Multicast IP address')
    parser.add_argument('--rigid-bodies', type=str, default="3,4",
                      help='Comma-separated list of rigid body IDs to track')
    parser.add_argument('--interface', type=str, default=None,
                      help='Network interface to bind to (e.g., eth0, enp0s3)')
    args = parser.parse_args()
    client = MinimalNatNetClient(
        server_ip=args.server_ip,
        multicast_ip=args.multicast_ip,
        interface=args.interface
    )
    client.rigid_bodies = {
        'arm_1': 5,
        'arm_2': 6
    }
    client.run()

if __name__ == "__main__":
    main()

