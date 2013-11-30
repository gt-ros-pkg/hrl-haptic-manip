import os
import fnmatch
import numpy as np
import scipy as sp
import scipy.signal
import scipy.optimize
import scipy.ndimage.filters
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

def get_data(filename):
    timestamps = []
    values = []
    with open(filename, 'r') as f:
        content = f.readlines()
        rows = [line.split() for line in content]
        timestamps = [np.float64(v[0]) for v in rows]
        values = [np.float64(v[1]) for v in rows]
    return (timestamps, values)

def compute_hysteresis(p1, t1, y1, p2, t2, y2): 
    prev = p1(t1[-1])
    after = p2(t2[0])
    
    #print 'taxel value is %f%% higher after'%(((after - prev)/prev)*100)
    print 'prev %f after %f difference %f %%'%(prev, after, ((after - prev)/prev)*100)
    #print 'prev * 1.0028 = %f'%(prev*1.0028)
 
    increase = 1.00012
    coeff = np.polyfit(t2, y2, 8)
    coeff[-1] = coeff[-1] - prev*increase
    #print 'Roots ', np.roots(coeff)
    #print 'To reach %f', prev*increase
    #print 'Initial time %f.'%t2[0]

for file in os.listdir('.'):
    if fnmatch.fnmatch(file, 'taxel_hysteresis4.dat'):
        print 'Opening file %s ...'%file

        timestamps, values = get_data(file)

        # Gaussian filtering        
        data = np.array(values)
        n = data.shape[0]
        filt = sp.signal.gaussian(31, 8)
        filt /= sum(filt)        
        padded = np.concatenate( (data[0]*np.ones(31//2), data, data[n-1]*np.ones(31//2)) )
        smooth = sp.signal.convolve(padded, filt, mode='valid')

        # Edge detection
        peaks = np.zeros(data.shape[0])
        sp.ndimage.filters.gaussian_filter1d(values, 10, order=2, output=peaks)
            
        # Zero crossings        
        z = np.where(np.diff(np.sign(peaks)))[0]
        xz = np.zeros(z.shape[0])
        xzs = [smooth[i] for i in z]
        
        # Curve fitting 
        y0 = np.array(values[:z[0]])
        t0 = np.array(timestamps[:z[0]])
        p0 = np.poly1d(np.polyfit(t0, y0, 8))
                
        y1 = np.array(values[z[3]:z[4]])
        t1 = np.array(timestamps[z[3]:z[4]])
        p1 = np.poly1d(np.polyfit(t1, y1, 8))
       
        y2 = np.array(values[z[-1]:n])
        t2 = np.array(timestamps[z[-1]:n])
        p2 = np.poly1d(np.polyfit(t2, y2, 8))

        compute_hysteresis(p0, t0, y0, p1, t1, y1)
        
        # Plots
        fig = plt.figure()  
        fig.subplots_adjust(hspace=0.5)
        
        ax = fig.add_subplot("411")
        ax.set_ylabel('Raw Ticks')
        ax.set_xlabel('Time')
        ax.set_title('Hysteresis')         
        ax.plot(timestamps, values, 'b-', label='Original data', markersize=2)
      
        ax = fig.add_subplot("412")
        ax.set_ylabel('Raw Ticks')
        ax.set_xlabel('Samples')
        ax.plot(smooth, 'b')
        ax.plot(z, xzs, 'o', color='r')        
                
        ax = fig.add_subplot("413")
        ax.set_ylabel('Response')
        ax.set_xlabel('Samples')
        ax.axhline(y=0, linewidth=1, color='black', linestyle='dashed')
        ax.plot(peaks, 'b')        
        ax.plot(z, xz, 'o', color='r')
      
        ax = fig.add_subplot("414")
        ax.set_ylabel('Raw Ticks')
        ax.set_xlabel('Time')
        ax.plot(timestamps, values, 'b-', markersize=2)
        ax.plot(t0, p0(t0), '-', color='g', linewidth=2)
        ax.plot(t1, p1(t1), '-', color='g', linewidth=2)
        ax.plot(t2, p2(t2), '-', color='g', linewidth=2)
                          
        #for t in np.arange(t1[0], 89.7, 0.05):
        #    print '%f    %f'%(p1(t)/p0(t0[-1]), t - t1[0])
                                
        canvas = FigureCanvasAgg(fig)
        canvas.print_figure('%s_hysteresis.svg'%file)
        canvas.print_figure('%s_hysteresis.png'%file, dpi=500) 
        
        plt.show()
