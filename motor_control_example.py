#!/usr/bin/env python3

from dynamixel_controller import DynamixelController
import time
import numpy as np



class MotorControl:
    def __init__(self):
        # Initialize controller

#         ls /dev/ttyUSB*
#         ls /dev/ttyACM*

        self.controller = DynamixelController(port='/dev/ttyUSB1', baudrate=1000000)

        # Get list of available motor IDs
        self.motor_ids = self.controller.available_ids
        print(f"Found motors: {self.motor_ids}")
        print("Get list of available motor IDs")
        print("\n")
        input("Press Enter to continue...")
        
        # 1. Switch to Current Mode and Set Currents
        self.set_manual_mode()
        
        # 2. Read current positions as zero reference
        self.zero_positions = self.read_positions()
        print(f"\nZero positions: {self.zero_positions}")
        input("Press Enter to continue...")

        # 3. Switch to Extended Position Mode
        self.set_position_mode()

        self.mt_limits = [-4095*2, 4095*2]  # Min and Max limits for motor actions
    

    def set_manual_mode(self):
        # 1. Switch to Current Mode and Set Currents
        print("\nSwitching to Current Mode...")
        self.controller.initialize_current_mode()
        self.controller.enable_torque()
        
        # Set currents (80mA * 2 for all motors)
        current_values = [30] * len(self.motor_ids)  # 2 * 80mA for each motor
        self.controller.set_currents(self.motor_ids, current_values)
        print("Currents set")
        
        time.sleep(2)  # Wait to see the effect
        input("Press Enter to continue...")


    def set_position_mode(self):
        # 3. Switch to Extended Position Mode
        print("\nSwitching to Extended Position Mode...")
        self.controller.initialize_extended_position_mode()
        self.controller.enable_torque()
        self.controller.set_velocity_profile(self.motor_ids, acceleration=50, velocity=50)
        
        input("Press Enter to continue...")


    def read_positions(self):
        # 2. Read current positions as zero reference
        feedback = self.controller.read_feedback(read_position=True)
        zero_positions = [feedback['position'][motor_id] for motor_id in self.motor_ids]
        return np.array(zero_positions)
    

    def check_safety(self, pred_action_seq):
        # Check if predicted action sequence is within limits
        if any(pred_action_seq < self.mt_limits[0]) or any(pred_action_seq > self.mt_limits[1]):
            print("predicted action sequence is out of limits", pred_action_seq)
            # cliff
            action_safe = np.clip(pred_action_seq, self.mt_limits[0], self.mt_limits[1])
            # raise ValueError("Predicted action sequence is out of limits")
        else:
            action_safe = pred_action_seq
            print("Predicted action sequence is within limits")
        return action_safe
    
    def move_arm(self, action):

        action = action[:9]  # Only take the first 9 values
        # round to nearest integer
        action = np.round(action).astype(int)
        print(f"Action: {action}")
        print("\n")
        action = self.check_safety(action)
        input("Press Enter to continue... to move the arm")
        self.controller.set_positions(self.motor_ids, list(self.zero_positions + action))
        print("Moved to new positions...")
        input("Press Enter to continue...")



if __name__ == "__main__":



    Robo_Crtl = MotorControl()



    proprio = Robo_Crtl.read_positions() - Robo_Crtl.zero_positions  # Dummy proprio
    print("proprio  ", proprio)

    for i in range(16):
        action_seq = [0] * 9  
        action_seq[0] = 100*i
        action_seq[1] = -50*i
        action_seq[2] = -50*i  
        action_seq[3] = 100*i
        action_seq[4] = -50*i
        action_seq[5] = -50*i
        action_seq[6] = 100*i
        action_seq[7] = -50*i
        action_seq[8] = -50*i

        Robo_Crtl.move_arm(action_seq)




    # diffModel_label = "Demo_2025_3_20_softReach---reach+imgState_cam_1_cam_2+arm_9dof+a_vel+grasp_mtDemands+b64_preDone_randomCrop_imgNorm_imgAugRand3"
    # Diffu_Crtl = DiffuController(diffModel_label)


    # for step in range(100000):
    #     frame_1 = cv2.imread("cam_1.jpg")   # Read image from camera 1
    #     frame_2 = cv2.imread("cam_2.jpg")

    #     frame_dict = {"cam_1": frame_1, "cam_2": frame_2, }

    #     proprio = np.array([0] *9   )  # Dummy proprio
    #     action_seq = Diffu_Crtl.diff_control(frame_dict, proprio)