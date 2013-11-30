#!/usr/bin/python
import sys
import math
import yaml
import numpy as np

def compute_matrix(alpha, radius, spacing):
	x = [[0 for i in range(4)] for j in range(4)]
	x[0][0] = 1
	x[3][3] = 1

	angle = math.radians(alpha)

	# 3x3 Rotation 
	x[1][1] = math.cos(angle)
	x[1][2] = math.sin(angle)
	x[2][1] = -math.sin(angle)
	x[2][2] = math.cos(angle)

	# Translation vector
	x[0][3] = spacing
	x[1][3] = radius * math.sin(angle)
	x[2][3] = radius * math.cos(angle)

	return x

def get_link_to_skin_tf():
	beta = math.radians(-90)
	y = [[0 for i in range(4)] for j in range(4)]
	y[0][0] = math.cos(beta)
	y[0][2] = -math.sin(beta)
	y[1][1] = 1
	y[2][0] = math.sin(beta)
	y[2][2] = math.cos(beta)
	y[3][3] = 1

	alpha = math.radians(-180)
	x = [[0 for i in range(4)] for j in range(4)]
	x[0][0] = 1
	x[1][1] = math.cos(alpha)
	x[1][2] = math.sin(alpha)
	x[2][1] = -math.sin(alpha)
	x[2][2] = math.cos(alpha)
	x[3][3] = 1

	z = [[0 for i in range(4)] for j in range(4)]
	z[0][0] = 1
	z[1][1] = 1
	z[2][2] = 1
	z[3][3] = 1

	r = np.matrix(y) * np.matrix(z) * np.matrix(x)

	t = [[0 for i in range(4)] for j in range(4)]
	t[2][3] = float(184.32/1000)
	
	return r + t

def get_lower_bound(idx):
	return math.floor(idx/24)*24

def get_index(low, idx):
	return int(((idx - low) - 6) % 24 + low)

def main(argv):  
	if len(sys.argv) != 4 :
		print 'Wrong syntax. Should be : %s radius spacing numslices.\nAll units are in mm' % (sys.argv[0])
		sys.exit(2) 

	radius = float(sys.argv[1])/1000
	spacing = float(sys.argv[2])/1000
	slices = int(sys.argv[3])

	f = open('tf.data')
	lines = f.readlines()
	f.close()

	tfs = []
	for i in range(slices):
		for line in lines:
			mat = compute_matrix(float(line), float(radius), float(spacing)*i)
			tfs.append(mat)
	
	tfs_perms = [0 for i in range(len(tfs))]
	for i in range(len(tfs)):
	    #print 'Mapping %d to %d'%(i, get_index(get_lower_bound(i), i))
	    tfs_perms[get_index(get_lower_bound(i), i)] = tfs[i]

	print yaml.dump({'number_taxels' : len(tfs_perms), 'joint_to_skin_tf': [get_link_to_skin_tf().tolist()], 'taxels_tf' : tfs_perms})


if __name__ == "__main__":
    main(sys.argv[1:])
