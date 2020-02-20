#!/usr/bin/env python3

from gym.envs.registration import register

register(
    id='MengeGym-v0',
    entry_point='src.envs:MengeGym'
)
