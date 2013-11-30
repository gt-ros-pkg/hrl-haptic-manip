import os
import fnmatch
import numpy as np
import taxelhistogram as th
import statisticstable as st
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.backends.backend_pdf import FigureCanvasPdf
from matplotlib.backends.backend_agg import FigureCanvasAgg

def get_data(filename):
    timestamps = []
    values = []
    with open(filename, 'r') as f:
        content = f.readlines()
        rows = [line.split() for line in content]
        timestamps = [v[0] for v in rows]
        values = [v[1:] for v in rows]
    return (timestamps, values)
   
def get_cdc_line(cdc):
    return np.floor(cdc/4)

def get_cdc_offset(cdc):
    return int((get_cdc_line(cdc)*48) + (cdc%4)*3)

def get_taxel_offset(cdc, taxel=0):
    return int(get_cdc_offset(cdc) + np.floor(taxel/3)*12 + (taxel%3))
        
def get_values_per_cdc(values):
    # Iterate through the columns so that 12 arrays (one cdc) are 
    # rendered simultenously by the TaxelHistogram
    cdcs = []
    for i in range(values.shape[1]/12):
        taxels = values[:, get_taxel_offset(i, 0)]
        taxels = taxels.reshape(len(taxels), 1)

        for j in range(1, 12):
            idx = get_taxel_offset(i, j)
            col = values[:, idx]     
            col = col.reshape(len(col), 1)
            taxels = np.append(taxels, col, axis=1)
           
        cdcs.append(taxels)
    
    return np.array(cdcs)
            
def create_stats_table(stats, filename):
    stats = [item for sublist in stats for item in sublist]
    stats = sorted(stats, key=lambda k: k['sigma']) 
    table = st.StatisticsTable(stats, filename)
    table.render_pdf()
            
if __name__ == '__main__':
    for file in os.listdir('.'):
        if fnmatch.fnmatch(file, 'taxel_*.dat'):
            print 'Opening file %s ...'%file

            timestamps,values = get_data(file)
            values = np.array(values, dtype=float)
            
            if values.shape[1] >= 12:
                cdcs = get_values_per_cdc(values)
                pp = PdfPages('foo.pdf')
                
                i = 1
                cdc_stats = []
                for cdc in cdcs:
                    fig = th.TaxelHistogram(cdc, i)
                    fig.set_size_inches(23.39, 33.11)
                    canvas = FigureCanvasPdf(fig)
                    fig.savefig(pp, format='pdf')
                    i += 1    
                    cdc_stats.append(fig.get_statistics())
                pp.close()
                
                # Create a statistics table from 
                # the values computed above
                create_stats_table(cdc_stats, file)
            else:   
                fig = th.TaxelHistogram(values)
                canvas = FigureCanvasAgg(fig)
                canvas.print_figure('%s.png'%file,dpi=500)
