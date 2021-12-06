import pybullet as p
import os
import numpy as np


class Robot:
    def __init__(self,client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'simplerobot.urdf')
        self.robot = p.loadURDF(fileName = f_name,
                              basePosition = [0, 0, 0],
                              useFixedBase=True,
                              physicsClientId = client)
        
        self.joints = [0,1,2,3]
        self.end_link = 4
        # self.target = target


    def get_ids(self):
        return self.robot, self.client

    def apply_position(self,position):

        position += [-position[1]-position[2]]
        p.setJointMotorControlArray(self.robot, self.joints,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPositions = position,
                                    physicsClientId = self.client)
    def get_position(self):
        return p.getLinkState(self.robot,self.end_link)[0]

    def apply_action(self, action):
        # Expects action to be five dimensional
        action = list(action)

        action += [-action[2]-action[1]]

        # Set the angle
        p.setJointMotorControlArray(self.robot, self.joints,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPositions = action,
                                    forces = [50000,50000,50000,50000],
                                    physicsClientId = self.client)
        # p.setJointMotorControlArray(self.robot, self.joints,
        #                             controlMode = p.VELOCITY_CONTROL,
        #                             targetVelocities = action,
        #                             physicsClientId = self.client)

    def get_observation_robot(self):
        # Get the position and orientation of the car in the simulation
        end_position = p.getLinkState(self.robot,self.end_link)[0]
        joints_state = p.getJointStates(self.robot,self.joints)
        joints_position_state = (joints_state[0][0], joints_state[1][0],joints_state[2][0])
        # joints_speed_state = (joints_state[1][1],joints_state[2][1])
        # observation = tuple(self.target) + tuple(end_position) + joints_position_state# + joints_speed_state
        observation = tuple(end_position) + joints_position_state
        return observation
