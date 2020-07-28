import numpy as np
import matplotlib.pyplot as plt

def add_guassian_noise(odometry, sigma):

    for i in range(len(odometry)):
        temp_noise = float(np.random.normal(0, sigma, 1))
        odometry[i] += temp_noise

    return odometry


odometry = [0, 0, 0, 0]
print  add_guassian_noise(odometry, 1)
