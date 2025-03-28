#!/usr/bin/env python3

import os
import sys
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table address for XL330-M077
ADDR_XL330_TORQUE_ENABLE = 64
ADDR_XL330_OPERATING_MODE = 11
ADDR_XL330_GOAL_POSITION = 116
ADDR_XL330_GOAL_VELOCITY = 104
ADDR_XL330_GOAL_CURRENT = 102
ADDR_XL330_PRESENT_POSITION = 132
LEN_XL330_PRESENT_POSITION = 4
ADDR_XL330_PRESENT_VELOCITY = 128
LEN_XL330_PRESENT_VELOCITY = 4
ADDR_XL330_PRESENT_PWM = 124
LEN_XL330_PRESENT_PWM = 2
ADDR_XL330_PRESENT_CURRENT = 126
LEN_XL330_PRESENT_CURRENT = 2
ADDR_XL330_PROFILE_ACCELERATION = 108
ADDR_XL330_PROFILE_VELOCITY = 112

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
BAUDRATE = 1000000  # Updated baudrate

# Control modes
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
POSITION_CONTROL_MODE = 3
VELOCITY_CONTROL_MODE = 1
CURRENT_CONTROL_MODE = 0
EXTENDED_POSITION_CONTROL_MODE = 4
CURRENT_BASED_POSITION_CONTROL_MODE = 5

class DynamixelController:
    def __init__(self, port='COM4', baudrate=BAUDRATE):
        """Initialize Dynamixel controller.
        
        Args:
            port (str): COM port name (e.g., 'COM3', 'COM4')
            baudrate (int): Communication baudrate
        """
        print(f"Initializing Dynamixel controller on port {port} with baudrate {baudrate}")
        
        # Initialize PortHandler instance
        self.portHandler = PortHandler(port)
        
        # Initialize PacketHandler instance
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Initialize GroupBulkRead instance for all feedback
        self.group_bulk_read = GroupBulkRead(self.portHandler, self.packetHandler)
        
        # Initialize GroupBulkWrite instance
        self.group_bulk_write = GroupBulkWrite(self.portHandler, self.packetHandler)
        
        # Initialize GroupSyncWrite instances
        self.group_sync_write_current = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_CURRENT, LEN_XL330_PRESENT_CURRENT)
        self.group_sync_write_mode = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_OPERATING_MODE, 1)
        self.group_sync_write_position = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, 4)
        self.group_sync_write_velocity = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_VELOCITY, 4)
        self.group_sync_write_profile_velocity = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_PROFILE_VELOCITY, 4)
        self.group_sync_write_profile_acceleration = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_PROFILE_ACCELERATION, 4)
        
        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            return
        
        # Set port baudrate
        if not self.portHandler.setBaudRate(baudrate):
            print("Failed to change the baudrate")
            return
            
        print("Successfully opened port and set baudrate")
        
        # Search for available motors
        self.available_ids = self.search_available_ids()
        print(f"Found {len(self.available_ids)} motors: {self.available_ids}")
        
        # Add parameter for each motor to bulk read instance
        for dxl_id in self.available_ids:
            # Add parameters for reading from PWM to Position (continuous range)
            # This reads from address 124 (PWM) to 135 (end of Position)
            # Total length = (135 - 124 + 1) = 12 bytes
            # This includes:
            # - Present PWM (124-125)
            # - Present Current (126-127)
            # - Present Velocity (128-131)
            # - Present Position (132-135)
            result = self.group_bulk_read.addParam(dxl_id, ADDR_XL330_PRESENT_PWM, 12)
            if not result:
                print(f"Failed to add feedback parameters for motor {dxl_id}")
                continue

    def bulk_write_torque(self, motor_ids, enable):
        """Enable/disable torque for multiple motors using bulk write.
        
        Args:
            motor_ids: List of motor IDs
            enable: True to enable, False to disable
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not isinstance(motor_ids, list):
            motor_ids = [motor_ids]
            
        # Clear any existing parameters
        self.group_bulk_write.clearParam()
        
        # Add parameters for each motor
        for motor_id in motor_ids:
            if motor_id not in self.available_ids:
                print(f"Motor ID {motor_id} not found")
                continue
                
            # Convert to byte array
            data = [TORQUE_ENABLE if enable else TORQUE_DISABLE]
            
            if not self.group_bulk_write.addParam(motor_id, ADDR_XL330_TORQUE_ENABLE, 1, data):
                print(f"Failed to add torque parameter for motor {motor_id}")
                return False
                
        # Bulk write torque
        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to bulk write torque: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
            
        # Clear parameters after writing
        self.group_bulk_write.clearParam()
        return True

    def sync_write_operating_mode(self, motor_ids, mode):
        """Set operating mode for multiple motors using sync write.
        
        Args:
            motor_ids: List of motor IDs
            mode: Operating mode to set (single value) or list of modes for each motor
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if not isinstance(motor_ids, list):
                motor_ids = [motor_ids]
                
            # Convert mode to list if single value
            if not isinstance(mode, list):
                mode = [mode] * len(motor_ids)
                
            # First disable torque on all motors
            if not self.bulk_write_torque(motor_ids, False):
                return False
                
            # Clear any existing parameters
            self.group_sync_write_mode.clearParam()
            
            # Add parameters for each motor
            for motor_id, mode_value in zip(motor_ids, mode):
                if motor_id not in self.available_ids:
                    print(f"Motor ID {motor_id} not found")
                    continue
                    
                # Convert to byte array
                data = [mode_value]
                
                if not self.group_sync_write_mode.addParam(motor_id, data):
                    return False
                    
            # Sync write mode
            dxl_comm_result = self.group_sync_write_mode.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                return False
                
            # Clear parameters after writing
            self.group_sync_write_mode.clearParam()
            
            # Re-enable torque on all motors
            if not self.bulk_write_torque(motor_ids, True):
                return False
                
            return True
            
        except Exception as e:
            print(f"Error in sync_write_operating_mode: {str(e)}")
            return False

    def initialize_current_mode(self, motor_ids=None):
        """Initialize motors to current control mode using sync write."""
        if motor_ids is None:
            motor_ids = self.available_ids
        return self.sync_write_operating_mode(motor_ids, CURRENT_CONTROL_MODE)

    def initialize_extended_position_mode(self, motor_ids=None):
        """Initialize motors to extended position control mode using sync write."""
        if motor_ids is None:
            motor_ids = self.available_ids
        return self.sync_write_operating_mode(motor_ids, EXTENDED_POSITION_CONTROL_MODE)

    def initialize_current_based_position_mode(self, motor_ids=None):
        """Initialize motors to current-based position control mode using sync write."""
        if motor_ids is None:
            motor_ids = self.available_ids
        return self.sync_write_operating_mode(motor_ids, CURRENT_BASED_POSITION_CONTROL_MODE)

    def set_operating_mode(self, motor_id, mode):
        """Set operating mode for a single motor.
        
        Args:
            motor_id: ID of the motor
            mode: Operating mode to set
            
        Returns:
            bool: True if successful, False otherwise
        """
        return self.sync_write_operating_mode([motor_id], mode)

    def search_available_ids(self):
        """Search for all available Dynamixel motors."""
        available_ids = []
        dxl_id_dict, dxl_comm_result = self.packetHandler.broadcastPing(self.portHandler)
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to broadcast ping: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return available_ids
        
        for dxl_id in dxl_id_dict.keys():
            available_ids.append(dxl_id)
            print(f"Found Dynamixel ID: {dxl_id}")
        return available_ids

    def read_feedback(self, read_position=True, read_current=True, read_pwm=True, read_velocity=True):
        """Read specified feedback types from all motors using bulk read.
        
        The function uses a single GroupBulkRead to read all data types in one transaction:
        1. txRxPacket() - Sends one packet to read all data from all motors
        2. getData() - Extracts each motor's data from the received buffer
        
        Args:
            read_position (bool): Whether to read position feedback
            read_current (bool): Whether to read current feedback
            read_pwm (bool): Whether to read PWM feedback
            read_velocity (bool): Whether to read velocity feedback
            
        Returns:
            dict: Dictionary containing requested feedback data for each motor
        """
        # Initialize feedback structure
        feedback_data = {
            'position': {},
            'velocity': {},
            'current': {},
            'pwm': {}
        }
        
        # Perform single bulk read for all data
        if self.group_bulk_read.txRxPacket() != COMM_SUCCESS:
            return None

        # Extract data from the received buffer for each motor
        for dxl_id in self.available_ids:
            if not self.group_bulk_read.isAvailable(dxl_id, ADDR_XL330_PRESENT_PWM, 12):
                continue
            
            if read_position:
                position = self.group_bulk_read.getData(dxl_id, ADDR_XL330_PRESENT_POSITION, LEN_XL330_PRESENT_POSITION)
                if position >= 0x80000000:
                    position -= 0x100000000
                feedback_data['position'][dxl_id] = position
            
            if read_velocity:
                velocity = self.group_bulk_read.getData(dxl_id, ADDR_XL330_PRESENT_VELOCITY, LEN_XL330_PRESENT_VELOCITY)
                if velocity >= 0x80000000:
                    velocity -= 0x100000000
                feedback_data['velocity'][dxl_id] = velocity
            
            if read_current:
                current = self.group_bulk_read.getData(dxl_id, ADDR_XL330_PRESENT_CURRENT, LEN_XL330_PRESENT_CURRENT)
                if current >= 0x8000:
                    current -= 0x10000
                feedback_data['current'][dxl_id] = current
            
            if read_pwm:
                pwm = self.group_bulk_read.getData(dxl_id, ADDR_XL330_PRESENT_PWM, LEN_XL330_PRESENT_PWM)
                if pwm >= 0x8000:
                    pwm -= 0x10000
                feedback_data['pwm'][dxl_id] = pwm

        return feedback_data

    def set_current_individual(self, motor_id, current_value):
        """Set current for a single motor using individual write.
        
        Args:
            motor_id: ID of the motor
            current_value: Current value to set (-1193 to 1193 mA)
        
        Returns:
            bool: True if successful, False otherwise
        """
        if motor_id not in self.available_ids:
            print(f"Motor ID {motor_id} not found")
            return False

        # Convert current value to 16-bit signed integer
        if current_value >= 0x8000:
            current_value -= 0x10000

        # Write current value directly to the motor
        result, error = self.packetHandler.write2ByteTxRx(
            self.portHandler, motor_id, ADDR_XL330_GOAL_CURRENT, current_value)
        
        if result != COMM_SUCCESS or error != 0:
            print(f"Failed to set current on motor {motor_id}")
            return False

        return True

    def set_currents(self, motor_ids, current_values):
        """Set current values for multiple motors."""
        try:
            # Write current values
            self.group_bulk_write.clearParam()
            for motor_id, current in zip(motor_ids, current_values):
                if not self.group_bulk_write.addParam(motor_id, ADDR_XL330_GOAL_CURRENT, 2, [current & 0xFF, (current >> 8) & 0xFF]):
                    return False
            
            # Bulk write current
            dxl_comm_result = self.group_bulk_write.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                return False
            
            return True
            
        except Exception as e:
            return False

    def enable_torque(self, motor_ids=None):
        """Enable torque for specified motors using bulk write."""
        if motor_ids is None:
            motor_ids = self.available_ids
        return self.bulk_write_torque(motor_ids, True)

    def disable_torque(self, motor_ids=None):
        """Disable torque for specified motors using bulk write."""
        if motor_ids is None:
            motor_ids = self.available_ids
        return self.bulk_write_torque(motor_ids, False)

    def set_positions(self, motor_ids, position_values):
        """Set position values for multiple motors."""
        try:
            # Write position values
            self.group_bulk_write.clearParam()
            for motor_id, position in zip(motor_ids, position_values):
                if not self.group_bulk_write.addParam(motor_id, ADDR_XL330_GOAL_POSITION, 4, [position & 0xFF, (position >> 8) & 0xFF, (position >> 16) & 0xFF, (position >> 24) & 0xFF]):
                    return False
            
            # Bulk write position
            dxl_comm_result = self.group_bulk_write.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                return False
            
            return True
            
        except Exception as e:
            return False
    
    def set_acceleration(self, motor_ids, acceleration):
        """Set acceleration profile for specified motors.
        
        Args:
            motor_ids: List of motor IDs or single motor ID
            acceleration: List of acceleration values (0~32767)
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Convert to lists if single values
            if not isinstance(motor_ids, list):
                motor_ids = [motor_ids]
            if not isinstance(acceleration, list):
                acceleration = [acceleration]
                
            # Ensure arrays have same length
            if len(motor_ids) != len(acceleration):
                raise ValueError(f"Length mismatch: motor_ids ({len(motor_ids)}) != acceleration ({len(acceleration)})")
            
            # Convert all values to integers
            motor_ids = [int(id) for id in motor_ids]
            acceleration = [int(val) for val in acceleration]
            
            # Add parameters for each motor
            for motor_id, acc in zip(motor_ids, acceleration):
                # Convert acceleration to bytes
                acc_bytes = [
                    DXL_LOBYTE(DXL_LOWORD(acc)),
                    DXL_HIBYTE(DXL_LOWORD(acc)),
                    DXL_LOBYTE(DXL_HIWORD(acc)),
                    DXL_HIBYTE(DXL_HIWORD(acc))
                ]
                
                result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_XL330_PROFILE_ACCELERATION, acc)
                
                if result != COMM_SUCCESS or error != 0:
                    return False
                    
            return True
            
        except Exception as e:
            print(f"Error in set_acceleration: {str(e)}")
            return False

    def set_velocity(self, motor_ids, velocity_values):
        """Set velocity for specified motors using sync write.
        
        Args:
            motor_ids: List of motor IDs or single motor ID
            velocity_values: List of velocity values or single velocity value
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not isinstance(motor_ids, list):
            motor_ids = [motor_ids]
            velocity_values = [velocity_values]
        elif not isinstance(velocity_values, list):
            velocity_values = [velocity_values] * len(motor_ids)

        for motor_id, velocity in zip(motor_ids, velocity_values):
            if motor_id not in self.available_ids:
                continue

            result, error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_XL330_PROFILE_VELOCITY, velocity)
            if result != COMM_SUCCESS or error != 0:
                return False

        return True

    def set_velocity_profile(self, motor_ids, acceleration=None, velocity=None):
        """Set velocity profile using direct packet writing.
        
        Args:
            motor_ids: List of motor IDs or single motor ID
            acceleration: Profile acceleration (0~32767)
            velocity: Profile velocity (0~32767)
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not isinstance(motor_ids, list):
            motor_ids = [motor_ids]

        if velocity is not None:
            for motor_id in motor_ids:
                if motor_id not in self.available_ids:
                    continue
                result, error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, motor_id, ADDR_XL330_PROFILE_VELOCITY, velocity)
                if result != COMM_SUCCESS or error != 0:
                    return False

        if acceleration is not None:
            for motor_id in motor_ids:
                if motor_id not in self.available_ids:
                    continue
                result, error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, motor_id, ADDR_XL330_PROFILE_ACCELERATION, acceleration)
                if result != COMM_SUCCESS or error != 0:
                    return False

        return True

    def close(self):
        """Close the port and clean up."""
        self.group_bulk_read.clearParam()
        self.portHandler.closePort()
        print("Port closed") 



