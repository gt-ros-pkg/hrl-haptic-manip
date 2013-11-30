import numpy as np
import matplotlib.pyplot as plt

with open('decay.dat', 'r') as file:
    data = np.array([np.float64(v.split()) for v in file.readlines()])
    
    y = np.array(data[:, 0])
    t = np.array(data[:, 1])
    t = t[t < 42]
    y = y[:t.shape[0]]
    p = np.poly1d(np.polyfit(t, y, 8))
        
    plt.figure()    
    plt.xlabel('Time (s)')
    plt.ylabel('Relative Increase (%)')
    plt.plot(t, p(t), '-', color='g', linewidth=2)
    plt.title('Hysteresis')
    plt.grid(True)    
    plt.savefig('relative_increase.svg')      
    plt.savefig('relative_increase.png')      
              
    c = 29.29 # ticks per g
    w0 = 20946.454562
    yw = y[:] * 20946.454562
    yw[:] -= 20946.454562
    yw[:] /= 29.29
    
    plt.figure()    
    plt.xlabel('Time (s)')
    plt.ylabel('Weight change measured (g)')
    plt.plot(t, yw, '-', color='g', linewidth=2)    
    plt.title('Hysteresis')    
    plt.grid(True)        
    plt.savefig('weight_change.svg')
    plt.savefig('weight_change.png')      
            
    #plt.show()