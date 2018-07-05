import numpy as np


def load_data(file_name):
    return np.loadtxt(file_name, delimiter=',', dtype='Float64', skiprows=1)