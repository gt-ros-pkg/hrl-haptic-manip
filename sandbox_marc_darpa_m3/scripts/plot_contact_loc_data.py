import cPickle as pkl
import numpy as np
import matplotlib.pyplot as pl
import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')

import hrl_lib.util as ut
import hrl_lib.matplotlib_util as mpu
from mpl_toolkits.axes_grid1 import make_axes_locatable


def plot_stuff(dyn_contact_force, dyn_contact_locs, qs_contact_force, qs_contact_locs, file_name, thresh_list):
    mpu.set_figure_size(12., 9.)
    mpu.reduce_figure_margins(left= 0.1, right = 0.75, top = 0.9, bottom=0.1)

    #for value in [6, 10, 15, 20, 25, 50]:
    #for value in [30, 35, 40, 45, 50]:
    for value in thresh_list:
        num_bins_y = 30
        num_bins_x = np.floor(0.7/1.2*num_bins_y)

        b = np.arange(num_bins_y)*(1.2/(num_bins_y-1))-0.6
        a = np.arange(num_bins_x)*(0.7/(num_bins_x-1))+0.2

        qs_ind = np.where(np.array(qs_contact_force) > value)[0]
        dyn_ind = np.where(np.array(dyn_contact_force) > value)[0]

        dyn_heatmap, dyn_xedges, dyn_yedges = np.histogram2d(np.array(dyn_contact_locs)[:,1], np.array(dyn_contact_locs)[:,0], bins = [b,a])
        dyn_heatmap2, dyn_xedges, dyn_yedges = np.histogram2d(np.array(dyn_contact_locs)[dyn_ind,1], np.array(dyn_contact_locs)[dyn_ind,0], bins = [b,a])

        qs_heatmap, qs_xedges, qs_yedges = np.histogram2d(np.array(qs_contact_locs)[:,1], np.array(qs_contact_locs)[:,0], bins = [b,a])
        qs_heatmap2, qs_xedges, qs_yedges = np.histogram2d(np.array(qs_contact_locs)[qs_ind,1], np.array(qs_contact_locs)[qs_ind,0], bins = [b,a])

        try:
            dyn_result = dyn_heatmap2/dyn_heatmap
        except ValueError:
            print "no more forces for dynamic"
            dyn_heatmap2 = np.zeros(dyn_heatmap.shape)
            dyn_result = np.zeros(dyn_heatmap.shape)
        
        dyn_extent = [dyn_xedges[0], dyn_xedges[-1], dyn_yedges[0], dyn_yedges[-1]]

        qs_result = qs_heatmap2/qs_heatmap
        qs_extent = [qs_xedges[0], qs_xedges[-1], qs_yedges[0], qs_yedges[-1]]

        dyn_ind_zero = np.where(dyn_result == np.inf)
        dyn_result[dyn_ind_zero] = 0
        dyn_ind_zero = np.where(np.isnan(dyn_result) == True)
        dyn_result[dyn_ind_zero] = 0

        qs_ind_zero = np.where(qs_result == np.inf)
        qs_result[qs_ind_zero] = 0
        qs_ind_zero = np.where(np.isnan(qs_result) == True)
        qs_result[qs_ind_zero] = 0

        max_lim = np.max([np.max(dyn_heatmap2), np.max(qs_heatmap2)])
        max_norm_lim = np.max([np.max(dyn_result), np.max(qs_result)])

        pl.figure()
        dyn_im = pl.imshow(dyn_heatmap2, extent=dyn_extent, vmin=0, vmax=max_lim)
        pl.xlabel('y-axis (meters)')
        pl.ylabel('x-axis (meters)')
        pl.title('Heatmap of Frequency of Contact Locations \n with Force Above '+str(value)+' Newtons')
        divider = make_axes_locatable(pl.gca())
        cax = divider.append_axes("right", "5%", pad="3%")
        pl.colorbar(dyn_im,cax=cax)
        pl.savefig(file_name+'_dyn_above_'+str(value)+'N.pdf')

        pl.figure()
        dyn_im = pl.imshow(dyn_result, extent=dyn_extent, vmin=0, vmax=max_norm_lim)
        pl.xlabel('y-axis (meters)')
        pl.ylabel('x-axis (meters)')
        pl.title('Normalized Heatmap of Frequency of Contact \n Locations with Force Above '+str(value)+' Newtons')
        divider = make_axes_locatable(pl.gca())
        cax = divider.append_axes("right", "5%", pad="3%")
        pl.colorbar(dyn_im,cax=cax)
        pl.savefig(file_name+'_dyn_normalized_for_above_'+str(value)+'N.pdf')


        pl.figure()
        qs_im = pl.imshow(qs_heatmap2, extent=qs_extent, vmin=0, vmax=max_lim)
        pl.xlabel('y-axis (meters)')
        pl.ylabel('x-axis (meters)')
        pl.title('Heatmap of Frequency of Contact Locations \n with Force Above '+str(value)+' Newtons')
        divider = make_axes_locatable(pl.gca())
        cax = divider.append_axes("right", "5%", pad="3%")
        pl.colorbar(qs_im,cax=cax)
        pl.savefig(file_name+'_qs_above_'+str(value)+'N.pdf')

        pl.figure()
        qs_im = pl.imshow(qs_result, extent=qs_extent, vmin=0, vmax=max_norm_lim)
        pl.xlabel('y-axis (meters)')
        pl.ylabel('x-axis (meters)')
        pl.title('Normalized Heatmap of Frequency of Contact \n Locations with Force Above '+str(value)+' Newtons')
        divider = make_axes_locatable(pl.gca())
        cax = divider.append_axes("right", "5%", pad="3%")
        pl.colorbar(qs_im,cax=cax)
        pl.savefig(file_name+'_qs_normalized_for_above_'+str(value)+'N.pdf')


    max_lim = np.max([np.max(dyn_heatmap), np.max(qs_heatmap)])

    pl.figure()
    dyn_im = pl.imshow(dyn_heatmap, extent=dyn_extent, vmin=0, vmax=max_lim)
    pl.xlabel('y-axis (meters)')
    pl.ylabel('x-axis (meters)')
    pl.title('Heatmap of Frequency of All Contact \n Locations in Cluttered Workspace')
    divider = make_axes_locatable(pl.gca())
    cax = divider.append_axes("right", "5%", pad="3%")
    pl.colorbar(dyn_im,cax=cax)
    pl.savefig(file_name+'_dyn_all_contact.pdf')


    pl.figure()
    qs_im = pl.imshow(qs_heatmap, extent=qs_extent, vmin=0, vmax=max_lim)
    pl.xlabel('y-axis (meters)')
    pl.ylabel('x-axis (meters)')
    pl.title('Heatmap of Frequency of All Contact \n Locations in Cluttered Workspace')
    divider = make_axes_locatable(pl.gca())
    cax = divider.append_axes("right", "5%", pad="3%")
    pl.colorbar(qs_im,cax=cax)
    pl.savefig(file_name+'_qs_all_contact.pdf')

