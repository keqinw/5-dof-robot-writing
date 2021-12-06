import gym
import gym_ur3
import time
from stable_baselines3 import PPO
import imageio
import pybullet as p

def test():
    env = gym.make('ur3-v0')
    # obs = env.reset()
    for _ in range(10):
        ob = env.reset()
        for _ in range(20):
            p.stepSimulation()
            time.sleep(1/24)
        # print(1)
        

def train():
    env = gym.make('ur3-v0')
    model = PPO('MlpPolicy',env,verbose=1)
    model.learn(total_timesteps=300000)
    model.save('../result/ppo.zip')

def result():
    frames = []
    env = gym.make('ur3-v0')
    model = PPO.load('../result/ppo.zip')
    
    for _ in range(10):
        obs = env.reset()
        for i in range(50):
            action,_state=model.predict(obs,deterministic=True)
            obs, _, done, _ = env.step(action)
            # print(done)
            frames.append(env.render())
            
            if done:
                # obs = env.reset()
                time.sleep(1/30)
                break
    imageio.mimsave('../UJI-3D/result/ur3.gif', frames, 'GIF', duration=0.1)
if __name__ == '__main__':
    # test()
    train()
    # result()
    
