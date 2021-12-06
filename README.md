# 5dof-robot-writing

this project aim to use PPO control a 5 dof manipulator to draw lines in 3d space.

# Introduction to the files

1. the pybullet environment is built in 'env' folder.
2. the 'PPO_trainer.py' is the trainnig program.
3. the 'robot_5dof.py' is the control program.
4. the results are put in 'result' folder.

# How to use it

1. run the 'PPO_trainner.py' to train the PPO agent. You also can test your agent by uncommenting the 'test()' funtion in main().
![alt text](https://github.com/keqinn/5dof-robot-writing/blob/main/result/ur3.gif?raw=true)
2. the trained agent will be saved in 'result' folder. 
3. run the 'robot_5dof.py' to generate the trajectory of manipulator, saved in 'action.npy'. 
![alt text](https://github.com/keqinn/5dof-robot-writing/blob/main/result/ur3_5dof.gif?raw=true)
4. connect to the manipulator, run the 'my_serial.py', enter 'python3 my_serial.py single back' in termianl to control the manipulator move to start position.
5. run 'control.py' to control the manipulator move following the trajectroy in the 'action.npy'. 
![alt text](https://github.com/keqinn/5dof-robot-writing/blob/main/result/real_world.gif?raw=true)

