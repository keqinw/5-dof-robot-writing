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

class Ur3_Env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):

        # set the action space & observation space
        self.speed = 0.3*np.pi
        self.angle = 0.5*np.pi
        
        # control joint1, joint2, joint3
        self.action_space = gym.spaces.box.Box(
            low=np.array([-self.angle,-self.angle,0], dtype=np.float32),
            high=np.array([self.angle,0,2*self.angle], dtype=np.float32))

        # self.action_space = gym.spaces.box.Box(
        #     low=np.array([-self.speed,-self.speed], dtype=np.float32),
        #     high=np.array([self.speed,self.speed], dtype=np.float32))
        
        # self.observation_space = gym.spaces.box.Box(
        #     low=np.array([-np.inf,-np.inf,-np.inf,-np.inf,-self.angle,-self.angle,-self.speed,-self.speed], dtype=np.float32),
        #     high=np.array([np.inf,np.inf,np.inf,np.inf,self.angle,self.angle,self.speed,self.speed], dtype=np.float32))

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
        self.tol = 0.006
        # self.success = 0
        self.num = None


        self.reset()

        self.viewMatrix = p.computeViewMatrix(cameraEyePosition=[1, 0, 0.1],
                                        cameraTargetPosition=[0, 0, 0.1],
                                        cameraUpVector=[0, 0, 1])
        self.projectionMatrix = p.computeProjectionMatrixFOV(fov=50,
                                                            aspect=1.0,
                                                            nearVal=0.1,
                                                            farVal=100)
    
    def step(self, action):
        self.robot.apply_action(action)
        p.stepSimulation()
        
        ob = np.array(self.goal + self.robot.get_observation_robot())
        distance = np.sqrt(np.sum((ob[0:3]-ob[3:6])**2))
        # print(distance)

        reward = (self.previous_distance - distance) 
        self.previous_distance = distance

        self.num += 1

        if distance <= self.tol:
            self.done = True
            reward = 1


        if p.getContactPoints() != ():
            self.done = True
            reward = -1

        if self.num >= 300:
            self.done = True
            # reward = -1
        
        # print(self.tol)
        # print(self.trails)
        return ob, reward, self.done, dict()

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, 0)
        Plane(self.client)
        self.robot = Robot(self.client)

        # Set the goal to a random target
        x = self.np_random.uniform(0.05, 0.1)
        y = self.np_random.uniform(-0.03, 0.03)
        # z = self.np_random.uniform(0.08, 0.13)
        # x = 0.08
        # y = 0.05
        z = 0.1

        self.goal = (x, y, z)
        Goal(self.client, self.goal)
        

        while 1:
            angle1 = self.np_random.uniform(-3.14159/2, 3.14159/2)
            angle2 = self.np_random.uniform(-3.14159/2, 3.14159/2)
            angle3 = self.np_random.uniform(-3.14159/2, 3.14159/2)
            
            self.origin = [angle1,angle2,angle3]
            
            self.robot.apply_position(self.origin)  
            
            for _ in range(5):
                p.stepSimulation()
                # time.sleep(1/24)
            position = self.robot.get_position()
            # print(position)
            if p.getContactPoints() == () and 0.1 <= position[0] <= 0.2 and -0.1 <= position[1] <= 0.1 and 0.12 <= position[2] <= 0.17:
                break

        
        self.done = False
        self.num = 0
        ob = np.array(self.goal + self.robot.get_observation_robot())
        self.previous_distance = np.sqrt(np.sum((ob[0:3]-ob[3:6])**2))
        # print(self.previous_distance)
        # # Get observation to return
        # observation = self.robot.get_observation()
        return ob

    # def reset_test(self,tol):
    #     p.resetSimulation(self.client)
    #     p.setGravity(0, 0, 0)
    #     # Reload the plane and car
    #     Plane(self.client)

    #     # Set the goal to a random target
    #     x = self.np_random.uniform(-0.2, 0.2)
    #     y = -0.425
    #     z = self.np_random.uniform(0.45, 0.65)


    #     self.goal = (x, y, z)
    #     Goal(self.client, self.goal)
    #     self.robot = Robot(self.goal,self.client)

    #     self.tol = tol
    #     self.done = False
    #     self.num = 0
    #     ob = self.robot.get_observation()
    #     self.previous_distance = np.sqrt(np.sum((ob[2]-self.goal[0])**2+(ob[3]-self.goal[2])**2))
    #     # # Get observation to return
    #     observation = self.robot.get_observation()
    #     return observation
    
    # def evaluate_init(self,tol):
    #     self.tol = tol


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

    #     self.num += 1


    #     if self.num == 300:
    #         self.done = True
    #         # reward = -1

        # self.previous_distance = distance
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

    def get_goal(self):
        return self.goal

    def get_tol(self):
        return self.tol

    def render(self, mode='human', close=False):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((224, 224, 4)))

        _, _, frame, _, _ = p.getCameraImage(width=224, height=224,
                                              viewMatrix=self.viewMatrix,
                                              projectionMatrix=self.projectionMatrix)

        frame = np.reshape(frame, (224, 224, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.01)
        return frame

    def close(self):
        p.disconnect(self.client)

    def print_tol(self):
        print(self.tol)