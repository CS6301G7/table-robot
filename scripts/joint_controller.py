#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tkinter as tk
from sensor_msgs.msg import JointState
from tkinter import ttk
from gripper import Gripper
import json

class JointControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Joint Control")
        self.delta = 0.1

        # Initialize ROS
        rospy.init_node("joint_control_gui")
        self.clients = {
            "arm": self.init_client("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction),
            "head":  self.init_client("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction),
            "torso": self.init_client("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        }
        self.gripper = Gripper()

        # Joint names and initial positions
        self.joint_names = {
            "arm": ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                    "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"],
            "head": ["head_pan_joint", "head_tilt_joint"],
            "torso": ["torso_lift_joint"],
        }
        
        
        
        self.joint_positions = {
            "arm": [], # [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0],
            "head": [], # [0.0, 0.7],
            "torso": [], # [0.4],
        }
        
        self.update_joint_states()

        # Create GUI elements
        self.create_controls()

    def init_client(self, name, action_type):
        client = actionlib.SimpleActionClient(name, action_type)
        rospy.loginfo(f"Waiting for {name}...")
        client.wait_for_server()
        rospy.loginfo(f"{name} connected.")
        return client

    def update_joint_states(self):
        try:
            msg = rospy.wait_for_message("/joint_states", JointState, timeout=5)
            
            joint_names = msg.name
            joint_positions = msg.position
            
            for section in self.joint_names:
                for _, joint_name in enumerate(self.joint_names[section]):
                    if joint_name in joint_names:
                        index = joint_names.index(joint_name)
                        self.joint_positions[section].append(joint_positions[index])
            
            rospy.loginfo("Updated joint states.")
        except rospy.ROSException:
            rospy.logwarn("Failed to retrieve joint states within the timeout period.")

    def create_controls(self):
        for section, joints in self.joint_names.items():
            frame = ttk.LabelFrame(self.root, text=f"{section.capitalize()} Control", padding=10)
            frame.pack(fill="x", padx=5, pady=5)

            controls = []
            for i, joint in enumerate(joints):
                controls.append(self.create_joint_controls(frame, section, joint, i))

            ttk.Button(frame, text="Update", command=lambda c=controls, s=section: self.update_all(c, s)).pack(pady=5)
            
        self.create_gripper_controls()
        self.create_automate()
        
    def create_automate(self):
        frame = ttk.LabelFrame(self.root, text="Automate", padding=10)
        frame.pack(fill="x", padx=5, pady=5)
        ttk.Button(frame, text="Execute", command=self.execute_automate).pack(side="left", padx=5)
    
    def execute_automate(self):
        try:
            with open("./poses.json") as in_f:
                pose_data = json.load(in_f)
                
            default_arm = pose_data["default_arm_pose"]
            default_delay = pose_data["default_delay"]
                
            def move_arm(pose, delay=default_delay):
                self.joint_positions["arm"] = pose
                self.send_joint_positions("arm", delay=delay)
                
            def move_poses(key):
                for model_pose_data in model_data[key]:
                    pose = model_pose_data["pose"]
                    delay = model_pose_data.get("delay", default_delay)
                    move_arm(pose, delay)
                
            # raise torso
            rospy.loginfo("raising torso")
            self.joint_positions["torso"] = [0.4]
            self.send_joint_positions("torso")
            
            rospy.loginfo("moving arm to default pose")
            move_arm(default_arm)
            
            rospy.loginfo("opening gripper")
            self.gripper.open()
            
            for model_data in pose_data["models"]:
                rospy.loginfo("moving " + model_data["model"])
                move_poses("pre_grasp")
                
                rospy.loginfo("closing gripper")
                self.gripper.close()
                
                rospy.loginfo("moving to destination")
                move_poses("post_grasp")
                
                # open gripper
                rospy.loginfo("opening gripper")
                self.gripper.open()
                
                rospy.loginfo("finishing")
                move_poses("post_place")
                    
                rospy.loginfo("moving arm to default pose")
                move_arm(default_arm)

        except (json.JSONDecodeError, ValueError) as e:
            rospy.logerr("Invalid Input", f"Error in JSON format or positions: {str(e)}")
    
    def create_gripper_controls(self):
        gripper_frame = ttk.LabelFrame(self.root, text=f"Gripper Control", padding=10)
        gripper_frame.pack(fill="x", padx=5, pady=2)
        
        gripper_label = ttk.Label(gripper_frame, text="Gripper", width=20, anchor="w")
        gripper_label.pack(side="left", padx=5)
        
        ttk.Button(gripper_frame, text="Open", command=self.gripper.open).pack(side="left", padx=2)
        ttk.Button(gripper_frame, text="Close", command=self.gripper.close).pack(side="left", padx=2)

    def create_joint_controls(self, parent, section, joint_name, index):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", padx=5, pady=2)

        label = ttk.Label(frame, text=joint_name, width=20, anchor="w")
        label.pack(side="left", padx=5)

        ttk.Button(frame, text="-", command=lambda: self.update_position(section, index, entry, -self.delta)).pack(side="left", padx=2)

        entry = ttk.Entry(frame, width=8)
        entry.pack(side="left", padx=5)
        self.refresh_entry(entry, self.joint_positions[section][index])

        entry.bind("<Return>", lambda e: self.set_position_from_entry(section, index, entry))

        ttk.Button(frame, text="+", command=lambda: self.update_position(section, index, entry, self.delta)).pack(side="left", padx=2)
        return {"entry": entry, "index": index}

    def update_position(self, section, index, entry, delta):
        self.joint_positions[section][index] += delta
        self.refresh_entry(entry, self.joint_positions[section][index])
        self.send_joint_positions(section)

    def refresh_entry(self, entry, value):
        entry.delete(0, tk.END)
        entry.insert(0, f"{value:.2f}")

    def set_position_from_entry(self, section, index, entry):
        try:
            value = float(entry.get())
            self.joint_positions[section][index] = value
            self.send_joint_positions(section)
        except ValueError:
            rospy.logwarn(f"Invalid input for {section} joint at index {index}. Please enter a number.")

    def update_all(self, controls, section):
        try:
            for control in controls:
                value = float(control["entry"].get())
                self.joint_positions[section][control["index"]] = value
            self.send_joint_positions(section)
        except ValueError:
            rospy.logwarn(f"Invalid input in {section} controls. Please ensure all values are valid numbers.")
    
    def send_joint_positions(self, section, delay=4.0):
        zeros = [0.0] * len(self.joint_positions[section])
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names[section]
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.joint_positions[section]
        trajectory.points[0].velocities = zeros
        trajectory.points[0].accelerations = zeros
        trajectory.points[0].time_from_start = rospy.Duration(delay)
        goal.trajectory = trajectory

        client = self.clients[section]
        client.send_goal(goal)
        # client.wait_for_result(rospy.Duration(delay))

        rospy.loginfo(f"Updated {section} positions: {self.joint_positions[section]}")

if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = JointControlGUI(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
