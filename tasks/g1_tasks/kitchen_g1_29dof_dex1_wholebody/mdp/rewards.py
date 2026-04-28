# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0

import torch


def zero_reward(env):
    return torch.zeros(env.num_envs, device=env.device)


__all__ = ["zero_reward"]
