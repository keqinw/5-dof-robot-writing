from time import sleep
import gym
# from gym import error, spaces, utils
# from gym.utils import seeding
import numpy as np
import pybullet as p
# import time
import matplotlib.pyplot as plt
from gym_ur3.resources.robot import Robot
from gym_ur3.resources.plane import Plane
from gym_ur3.resources.goal import Goal
import time

class Ur3_Env_Visual(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):

        # set the action space & observation space
        self.speed = 0.5*np.pi
        self.angle = np.pi
        
        # control joint1, joint2, joint3
        self.action_space = gym.spaces.box.Box(
            low=np.array([-self.angle,-self.angle,-self.angle], dtype=np.float32),
            high=np.array([self.angle,self.angle,self.angle], dtype=np.float32))

    
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-self.angle,-self.angle,-self.angle], dtype=np.float32),
            high=np.array([np.inf,np.inf,np.inf,np.inf,np.inf,np.inf,self.angle,self.angle,self.angle], dtype=np.float32))

        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.DIRECT)
        # self.client = p.connect(p.GUI)
        p.setTimeStep(1/5, self.client)

        self.robot = None
        self.goal = None
        self.done = False
        self.rendered_img = None
        self.previous_distance = None
        self.origin = None
        self.tol = 0.01
        # self.success = 0
        self.num = None

        self.evaluate_reset(0.01)

        self.viewMatrix = p.computeViewMatrix(cameraEyePosition=[0.2, 0, 0.5],
                                        cameraTargetPosition=[0, 0, 0.1],
                                        cameraUpVector=[0, 0, 1])
        self.projectionMatrix = p.computeProjectionMatrixFOV(fov=50,
                                                            aspect=1.0,
                                                            nearVal=0.1,
                                                            farVal=100)
    
    
    def evaluate_step(self, action):
        # self.goal = goal
        # Goal(self.client, self.goal)
        self.robot.apply_action(action)
        p.stepSimulation()
        ob = np.array(self.goal + self.robot.get_observation_robot())
        distance = np.sqrt(np.sum(ob[0:3]-ob[3:6])**2)
        
        reward = -0.1

        if distance <= self.tol:
            self.done = True
            # print(distance)
            reward = 10

        if p.getContactPoints() != ():
            self.done = True
            reward = -10

        return ob, reward, self.done, dict()

    def evaluate_reset(self,tol):
         # Set the goal to a random target
        self.goal = (0,0,0)
        # Goal(self.client, self.goal)
        self.robot = Robot(self.client)

        self.tol = tol
        self.done = False
        self.num = 0
        ob = np.array(self.goal + self.robot.get_observation_robot())
        # distance = np.sqrt(np.sum((np.array(self.goal)-np.array(ob[3:6]))**2))
        # # Get observation to return
        return ob

    def evaluate_goal(self,goal):
        self.goal = goal
        Goal(self.client, self.goal)
        ob = np.array(self.goal + self.robot.get_observation_robot())
        return ob

    def evaluate_goal_idle(self,goal):
        self.goal = goal
        # Goal(self.client, self.goal)
        ob = np.array(self.goal + self.robot.get_observation_robot())
        return ob

    def get_goal(self):
        return self.goal

    def get_tol(self):
        return self.tol

    def render(self, size, mode='human', close=False):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((size, size, 4)))

        _, _, frame, _, _ = p.getCameraImage(width=size, height=size,
                                              viewMatrix=self.viewMatrix,
                                              projectionMatrix=self.projectionMatrix)

        frame = np.reshape(frame, (size, size, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.01)
        return frame

    def close(self):
        p.disconnect(self.client)

    def print_tol(self):
        print(self.tol)