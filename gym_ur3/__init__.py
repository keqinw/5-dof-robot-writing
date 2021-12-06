from gym.envs.registration import register

register(
    id='ur3-v0',
    entry_point='gym_ur3.envs:Ur3_Env',
)

register(
    id='ur3-visual-v0',
    entry_point='gym_ur3.envs:Ur3_Env_Visual',
)