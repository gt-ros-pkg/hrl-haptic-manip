
import math, numpy as np

import roslib
roslib.load_manifest('sandbox_advait_darpa_m3')



if __name__ == '__main__':
    print 'Hello World'

    obstacle_radius = 0.01

    total_area = 0.6 * 1.2 # from generate_trials.sh

    num_obstacles = 160

    occupied_area = num_obstacles * math.pi * obstacle_radius**2

    percent_occupied = occupied_area / total_area * 100

    print 'occupied percent:', percent_occupied



