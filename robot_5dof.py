import numpy as np
import gym
import gym_ur3
import time
from stable_baselines3 import PPO
import matplotlib.pyplot as plt
import cv2 as cv
import imageio
import copy 
import pybullet as p

def save2gif(frames):
    imageio.mimsave('../UJI-3D/result/ur3_5dof.gif', frames, 'GIF', duration=0.03)

# rectangular path 
xs = list(np.linspace(0.1,0.2,20)) + [0.2 for i in range(20)] + list(np.linspace(0.2,0.1,20)) + [0.1 for i in range(20)]
ys = [-0.04 for i in range(20)] + list(np.linspace(-0.04,0.04,20)) + [0.04 for i in range(20)] + list(np.linspace(0.04,-0.04,20))
zs = [0.14 for i in range(len(xs))]
# plt.plot(ys,zs)
# plt.show()

# circle path
r = 0.04
theta = np.arange(0, 2*np.pi+0.1, 0.1)
xs = 0.15 + r * np.cos(theta)
ys = 0 + r * np.sin(theta)
zs = [0.14 for i in range(len(xs))]

plt.plot(xs,ys,'-')
plt.title('origin path')
plt.axis('equal')
plt.show()



# establish the environment
env = gym.make('ur3-visual-v0')
model = PPO.load('../UJI-3D/result/ppo-ur3-3d-1202.zip')

result_x = []
result_y = []
frames = []
actions = []

for x,y,z in zip(xs,ys,zs):
    obs = env.evaluate_goal((x,y,z))
    done = False

    for i in range(100):
        if not done:
            action,_state=model.predict(obs,deterministic=True)
            obs, _, done, _ = env.evaluate_step(action)

            image = env.render(224)
            frames.append(image)
            
        if done:
            result_x.append(obs[3])
            result_y.append(obs[4])
            # let the path get more smooth
            obs, _, done, _ = env.evaluate_step(action)
            if i%5 == 0:
                actions.append([-obs[-3]]+list(obs[-2:])+[obs[-1]+obs[-2]-np.pi/2])
            break


plt.close()
plt.title('result path')
plt.plot(result_x,result_y,'-',label='result path')
plt.plot(xs,ys,'-',label='origin path')
plt.axis('equal')
plt.legend()
plt.show()

save2gif(frames)
print(actions)
np.save('../UJI-3D/result/actions.npy',actions)
