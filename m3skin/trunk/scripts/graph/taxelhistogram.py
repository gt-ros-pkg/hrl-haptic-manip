# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor Boston, MA 02110-1301,  USA
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

class TaxelHistogram(Figure):
    def __init__(self, values, cidx=1):
        Figure.__init__(self)
        
        self.stats = []
        
        if values.shape[1] < 12:
            self.draw_histogram(values[:, 0])
        else:
            for i in range(values.shape[1]):
                self.draw_histogram(values[:, i], 3, 4, i+1, False, cidx)

    def get_statistics(self):
        return self.stats

    def draw_histogram(self, values, ncols=1, nrows=1, pidx=1, display_text=True, cidx=1):
        mu = values.mean()
        median = np.median(values)
        sigma = values.std()
        
        ax = self.add_subplot(nrows, ncols, pidx)

        if display_text:
            ax.set_ylabel('Frequency')
            ax.set_xlabel('Raw Ticks')
            ax.set_title(r'$\mathrm{Histogram\ of\ the\ Raw\ Taxel\ Ticks}$')

        major_formatter = plt.FormatStrFormatter('%d')
        ax.xaxis.set_major_formatter(major_formatter)
        self.autofmt_xdate()

        n, bins, patches = ax.hist(values, bins=50, facecolor='green')

        # these are matplotlib.patch.Patch properies
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)

        textstr = '$\mu=%f$\n$ \mu_{1/2}=%d$\n$\sigma=%f$\n$n=%d$'%(mu, median, sigma, len(values))

        # place a text box in upper left in axes coords
        ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=14,
                verticalalignment='top', bbox=props)

        # Locate the mean by a vertical line
        ax.axvline(x=mu, linewidth=1)

        # Show the area spanned by one point of the standard deviation
        ax.axvspan(mu-sigma, mu+sigma, facecolor='0.5', alpha=0.5)

        # Annotate the above area with the coverage
        coverage = 0
        for v in values:
            if v >= (mu - sigma) and v <= (mu + sigma):
                coverage += 1
        
        coverage = (coverage/float(len(values)))*100
        print 'Coverage %f'%coverage

        ax.annotate('$%2.2f\%%$'%coverage, xy=(mu-sigma, n.max()),  xycoords='data')
        self.stats.append({'mu' : mu, 'sigma' : sigma, 'median' : median, 'taxel' : pidx, 'cdc' : cidx})
