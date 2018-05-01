#!/usr/bin/python3.6
from tensorforce.agents import PPOAgent
import tensorflow as tf

def string_to_state(string):
    state = string.split()
    state = list(map(float, state))

    return state

if __name__ == "__main__":
    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.2)

    agent = PPOAgent(
        states=dict(type='float', shape=(30)), #shape=[4, 9, 4, 4, 4, 4, 4, 4]),
        actions=dict(type='int', num_actions=12),
        network=[
            dict(type='dense', size=64, activation='tanh'),
            dict(type='dense', size=64, activation='tanh'),
            dict(type='dense', size=64, activation='tanh'),
            dict(type='dense', size=64, activation='tanh')
        ],
        update_mode=dict(
            batch_size=100
        ),
        execution=dict(
            type='single',
            session_config=tf.ConfigProto(gpu_options=gpu_options),
            distributed_spec=None
        ),
        step_optimizer=dict(type='adam',learning_rate=1e-4),
        baseline_mode="states",
        baseline=dict(
            type="mlp",
            sizes=[128, 128]
        ),
        baseline_optimizer=dict(
            type="multi_step",
            optimizer=dict(
                type="adam",
                learning_rate=1e-4
            ),
            num_steps=10
        )
    )

    try:
        agent.restore_model(directory="/home/figo/catkin_ws/src/ml_node/src/data", file="data")
        print("Ready; data loaded")

    except:
        print("Ready; couldn't load data (use absolute path to the data folder)")

    while True:
        s = input()
        print(agent.act(string_to_state(s), deterministic=True))
