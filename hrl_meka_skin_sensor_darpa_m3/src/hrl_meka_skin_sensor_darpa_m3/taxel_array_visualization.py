
import numpy as np, math
from enthought.mayavi import mlab
import matplotlib.pyplot as pp
import matplotlib.cm as cm

import scipy.ndimage as ni

import roslib; roslib.load_manifest('darpa_m3')
import rospy
import hrl_lib.mayavi2_util as mu
import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu


def plot_taxel_locations(ta):
    pts = np.matrix(np.row_stack([ta.centers_x, ta.centers_y, ta.centers_z]))
    mu.plot(pts)

def plot_taxel_array(ta):
    pts = np.matrix(np.row_stack([ta.centers_x, ta.centers_y, ta.centers_z]))
    forces = np.matrix(np.row_stack([ta.values_x, ta.values_y, ta.values_z]))
    mu.plot_quiver(pts, forces)
    mu.plot_points(pts[:,::24], (0., 0., 1.), 0.002, mode='sphere')
    
# desired_unroll_index of 0 roughly corresponds to the middle of the
# skin PCB box. (Tapo verified this and Advait is putting in the
# comment and setting the default parameter to 0)
def taxel_array_to_np_arrays(ta, desired_unroll_index=0):
    force_vectors = np.row_stack([ta.values_x, ta.values_y, ta.values_z])
    fmags = ut.norm(force_vectors)
    force_arr_raw = fmags.reshape((16,24)) #Verified by Tapo

    force_arr = np.column_stack((force_arr_raw[:,desired_unroll_index:],force_arr_raw[:,:desired_unroll_index]))

    center_arr = np.row_stack([ta.centers_x, ta.centers_y, ta.centers_z]).reshape((3,16,24))
    nrml_arr = np.row_stack([ta.normals_x, ta.normals_y, ta.normals_z]).reshape((3,16,24))
    return force_arr, center_arr, nrml_arr

def plot_taxel_array_as_image(ta):
    index = 0 #Verified by Tapo
    forces_arr = taxel_array_to_np_arrays(ta,index)[0]
    pp.imshow(forces_arr, interpolation='nearest', cmap=cm.binary,
              origin='upper', vmin=0, vmax=1)
    pp.title('Unrolled Taxel Array')
    pp.xlabel('Along the circumference')
    pp.ylabel('Along the forearm')
    pp.xlim(-0.5,23.5)
    pp.ylim(15.5, -0.5)

def compute_contact_regions(arr_2d, threshold):
    mask = arr_2d > threshold
    label_im, nb_labels = ni.label(mask)
    return label_im, nb_labels

# force_arr - 2D array of contact force magnitude. adjacency of taxels
# is the same as adjacency in the 2D array.
def compute_resultant_force_magnitudes(force_arr, label_im, nb_labels):
    total_forces = ni.sum(force_arr, label_im, range(1, nb_labels + 1))
    if total_forces.__class__ != list:
        total_forces = [total_forces]
    return total_forces

# cx_arr - 2D array of x coordinates of the centers of the taxels.
# adjacency of taxels is the same as adjacency in the 2D array.
def compute_centers_of_pressure(cx_arr, cy_arr, cz_arr, label_im,
                                nb_labels):
    # for the forearm skin patch, it might be more accurate to perform
    # the averaging in cylindrical coordinates (to ensure that the COP
    # lies on the surface of the skin), but Advait does not care
    # enough.
    cx = ni.mean(cx_arr, label_im, range(1, nb_labels + 1))
    cy = ni.mean(cy_arr, label_im, range(1, nb_labels + 1))
    cz = ni.mean(cz_arr, label_im, range(1, nb_labels + 1))
    if cx.__class__!= list:
        cx = [cx]
        cy = [cy]
        cz = [cz]
    return cx, cy, cz

def compute_resultant_force_directions(nx_arr, ny_arr, nz_arr,
                                       label_im, nb_labels):
    nx = ni.mean(nx_arr, label_im, range(1, nb_labels + 1))
    ny = ni.mean(ny_arr, label_im, range(1, nb_labels + 1))
    nz = ni.mean(nz_arr, label_im, range(1, nb_labels + 1))
    if nx.__class__!= list:
        nx = [nx]
        ny = [ny]
        nz = [nz]
    return nx, ny, nz


if __name__ == '__main__':
    ta = ut.load_pickle('taxel_array.pkl')

    if False:
        # test to ensure that unrolling taxels into an image still
        # preverves adjacency.
        ta.values_x = list(ta.values_x)
        ta.values_x[0] = 1.
        ta.values_x[1] = 1.
        ta.values_x[23] = 1.

        ta.values_y = list(ta.values_y)
        ta.values_y[0] = 1.
        ta.values_y[1] = 1.
        ta.values_y[23] = 1.

    if False:
        # sanity check that taxel centers are along a cylinder.
        plot_taxel_locations(ta)

    if True:
        plot_taxel_array(ta)
        #mu.show()

    if True:
        force_arr, center_arr, nrml_arr = taxel_array_to_np_arrays(ta)
        label_im, nb_labels = compute_contact_regions(force_arr, 0.01)

        forces = compute_resultant_force_magnitudes(force_arr,
                                                    label_im,
                                                    nb_labels)

        cx_arr = center_arr[0]
        cy_arr = center_arr[1]
        cz_arr = center_arr[2]

        cx, cy, cz = compute_centers_of_pressure(cx_arr, cy_arr,
                                                 cz_arr, label_im,
                                                 nb_labels)
        cop_mat = np.matrix(np.row_stack((cx, cy, cz)))

        nx_arr = nrml_arr[0,:,:]
        ny_arr = nrml_arr[1,:,:]
        nz_arr = nrml_arr[2,:,:]

        nx, ny, nz = compute_resultant_force_directions(nx_arr,
                                        ny_arr, nz_arr, label_im,
                                        nb_labels)

        res_force_mat = np.matrix(np.row_stack((nx, ny, nz) * np.array(forces)))

        mu.plot_quiver(cop_mat, res_force_mat, (1., 0., 0.), 0.05)
        mu.plot_points(cop_mat, (0., 1., 1.), 0.004, mode='sphere')

        print 'Number of regions:', nb_labels
        print 'resultant forces:', forces
        print 'centers_of pressure -'
        print 'cx:', cx
        print 'cx.__class__:', cx.__class__
        print 'cy:', cy
        print 'cz:', cz

        pp.imshow(label_im)


    if True:
        mpu.figure()
        plot_taxel_array_as_image(ta)
        pp.show()


