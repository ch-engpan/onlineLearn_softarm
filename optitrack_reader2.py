import socket
import struct
import sys

class MinimalNatNetClient:
    def __init__(self, server_ip="169.254.118.69", multicast_ip="239.255.42.99"):
        self.server_ip = server_ip
        self.command_port = 1510
        self.data_port = 1511
        self.multicast_ip = multicast_ip
        self.data_socket = None
        self.command_socket = None
        self.rigid_bodies = {'arm_1': 5, 'arm_2': 6}

    def connect(self):
        # Create command socket
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind(('', 0))
        self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # Create data socket
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.data_socket.bind(('', self.data_port))

        # Join multicast group
        mreq = struct.pack("4s4s", socket.inet_aton(self.multicast_ip), socket.inet_aton("0.0.0.0"))
        self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        return True

    def unpack_rigid_bodies(self, data, offset):
        rigid_body_count = struct.unpack('i', data[offset:offset+4])[0]
        offset += 4
        
        rigid_bodies_data = []
        for _ in range(rigid_body_count):
            # Read ID, position (x,y,z), and rotation (qx,qy,qz,qw)
            values = struct.unpack('i7f', data[offset:offset+32])
            rb_id, x, y, z, qx, qy, qz, qw = values
            offset += 32

            # Skip mean error and tracking flags
            offset += 6
            
            rigid_bodies_data.append((rb_id, (x, y, z), (qx, qy, qz, qw)))

        return rigid_bodies_data, offset

    def receive_data(self):
        data, addr = self.data_socket.recvfrom(32768)
        if len(data) < 4:
            return None

        message_id = struct.unpack('h', data[0:2])[0]
        packet_size = struct.unpack('h', data[2:4])[0]

        if message_id == 7:  # NAT_FRAMEOFDATA
            offset = 4
            # Skip frame number
            offset += 4
            # Skip markersets
            markerset_count = struct.unpack('i', data[offset:offset+4])[0]
            offset += 4
            for _ in range(markerset_count):
                name_len = data[offset:].find(b'\0')
                offset += name_len + 1
                marker_count = struct.unpack('i', data[offset:offset+4])[0]
                offset += 4 + (marker_count * 12)

            # Skip unlabeled markers
            unlabeled_markers = struct.unpack('i', data[offset:offset+4])[0]
            offset += 4 + (unlabeled_markers * 12)

            # Process rigid bodies
            rigid_bodies_data, _ = self.unpack_rigid_bodies(data, offset)
            return rigid_bodies_data

        return None

    def run(self):
        try:
            if self.connect():
                while True:
                    data = self.receive_data()
                    if data:
                        for rb_id, pos, rot in data:
                            print(f"RB ID: {rb_id}, Pos: {pos}, Rot: {rot}")
        except KeyboardInterrupt:
            if self.data_socket:
                self.data_socket.close()
            if self.command_socket:
                self.command_socket.close()

def main():
    client = MinimalNatNetClient()
    client.rigid_bodies = {
       'arm_1': 5,
        'arm_2': 6
    }
    client.run()

if __name__ == "__main__":
    main() 