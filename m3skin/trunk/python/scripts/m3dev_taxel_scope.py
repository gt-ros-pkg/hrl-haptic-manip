#! /usr/bin/python

#Copyright  2008, Meka Robotics
#All rights reserved.
#http://mekabot.com

#Redistribution and use in source and binary forms, with or without
#modification, are permitted.


#THIS SOFTWARE IS PROVIDED BY THE Copyright HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#Copyright OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES INCLUDING,
#BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#POSSIBILITY OF SUCH DAMAGE.

import time
import datetime
import math
import numpy as np
from numpy import arange, sin, pi

import gtk
import gobject
import matplotlib.artist as mpla
from matplotlib.lines import Line2D
from matplotlib.figure import Figure
from matplotlib.backends.backend_gtkagg import FigureCanvasGTKAgg as FigureCanvas
from matplotlib.ticker import Formatter, Locator, NullLocator, FixedLocator, NullFormatter, FormatStrFormatter

import m3.toolbox as m3t
import m3.rt_proxy as m3p
import m3skin.taxel_array_ec as mta

gobject.threads_init()

class M3SkinEmitter:
    ''' Pumps taxel data from the M3 server.

    This class connect to the M3 server via a proxy and access the taxel_value
    array from the status messages of the m3taxel_array_ec01 component

    '''
    def __init__(self):
        self.proxy = m3p.M3RtProxy()
        self.components = []
        self.comp = None
        self.taxel_value = []

    def stop(self):
        self.proxy.stop()

    def start(self):
        self.proxy.start()
        self.comp = mta.M3TaxelArrayEc('m3taxel_array_ec01')
        self.proxy.subscribe_status(self.comp)
        self.proxy.make_operational_all()
        self.proxy.step()

    def step(self):
        self.taxel_value = self.comp.status.taxel_value
        self.proxy.step()

    def get_taxel_value(self):
        return self.taxel_value

class TaxelScope(FigureCanvas):
    ''' Scope for a single taxel '''
    def __init__(self):
        f = Figure(figsize=(5,4), dpi=100)

        self.background = None
        self.line = None
        self.ax = None

        self.ydata = [0]
        self.tdata = [0]
        self.max_sample_duration = 1.0
        self.time_start = time.time()

        self.create_plot(f)

        FigureCanvas.__init__(self, f)

        self.mpl_connect('draw_event', self.update_background_cb)

    def register(self, callback):
        self.mpl_connect('button_press_event', callback)

    def update_background_cb(self, event):
        self.background = self.copy_from_bbox(self.ax.bbox)

    def button_pressed_cb(self, event):
        print 'Figure clicked with button ', event.button

    def create_plot(self, figure):
        self.ax = figure.add_subplot(1,1,1)
        self.ax.patch.set_facecolor('white')
        self.ax.set_ylabel('Raw Value', fontsize=6)
        self.ax.set_xlabel('Time', fontsize=6)
        self.ax.set_ylim(0, 65535)
        locator = self.ax.yaxis.get_major_locator()
        self.ax.set_ylim(locator.autoscale())
        #self.ax.autoscale_view(scalex=None)
        self.ax.set_xlim(0, self.max_sample_duration)

        formatter = FormatStrFormatter('%2.1f')
        self.ax.xaxis.set_major_formatter(formatter)

        mpla.setp(self.ax.get_xticklabels(), rotation='vertical', fontsize=6)
        mpla.setp(self.ax.get_yticklabels(), fontsize=6)

        self.line = Line2D(self.tdata, self.ydata, animated=True)
        self.ax.add_line(self.line)

    def get_seconds_elapsed(self):
        return time.time() - self.time_start

    def reset(self):
        self.tdata = [self.tdata[-1]]
        self.ydata = [self.ydata[-1]]
        self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.max_sample_duration)
        self.ax.figure.canvas.draw()

    def plot(self, value):
        # Check if we should scroll
        if self.tdata[-1] > self.tdata[0] + self.max_sample_duration:
            self.reset()

        self.restore_region(self.background)

        # Update the data
        self.ydata.append(value)
        self.tdata.append(self.get_seconds_elapsed())
        self.line.set_data(self.tdata, self.ydata)
        #self.ax.autoscale_view(scalex=None)
        #self.ax.relim()
        self.ax.draw_artist(self.line)
        self.blit(self.ax.bbox)

class CdcScope(gtk.VBox):
    ''' A scope that displays all 12 taxels of a CDC chip'''
    def __init__(self):
        gtk.VBox.__init__(self)
        self.offset = 0
        self.cdc = 0

        # Add frame to main vbox
        self.taxel_frame = gtk.Frame("Capacitive-to-Digital Converter 1/32")
        self.taxel_frame.set_border_width(5)
        self.pack_start(self.taxel_frame)

        self.table = gtk.Table(4, 3, True)
        self.table.set_border_width(10)
        self.table.set_col_spacings(5)
        self.table.set_row_spacings(5)
        self.taxel_frame.add(self.table)

        # Add the scopes to the vbox
        self.scope = []
        for i in range(4):
            for j in range(3):
                self.scope.append(TaxelScope())
                self.scope[-1].register(self.taxel_scope_selected_cb)
                self.table.attach(self.scope[-1], j, j+1, i, i+1)

        # Add a combo box to the window
        align = gtk.Alignment(1.0, 0.0, 0, 0)
        self.pack_start(align, False, False)
        
        self.lower_vbox = gtk.VBox(True, 5)
        self.lower_vbox.set_border_width(5)
        align.add(self.lower_vbox)

        self.cdc_combo = gtk.combo_box_new_text()

        self.lower_vbox.pack_start(self.cdc_combo, False, False)
        for i in range(32):
            self.cdc_combo.append_text('Capacitive-to-Digital Converter %d'%int(i+1))
        self.cdc_combo.set_active(0)
        self.cdc_combo.connect('changed', self.cdc_combo_changed_cb)

        # Add button to switch between CDCs
        self.button_box = gtk.HBox(True, 5)
        self.lower_vbox.pack_start(self.button_box, False, False)
      
        self.forward_button = gtk.Button(stock=gtk.STOCK_GO_FORWARD)
        self.forward_button.connect('clicked', self.forward_button_clicked_cb)
        self.backward_button = gtk.Button(stock=gtk.STOCK_GO_BACK)
        self.backward_button.connect('clicked', self.back_button_clicked_cb)
                               
        self.button_box.pack_start(self.backward_button)
        self.button_box.pack_start(self.forward_button)
        
        # Define a new signal when a scope is selected
        gobject.signal_new("scope-clicked", CdcScope, gobject.SIGNAL_RUN_LAST, gobject.TYPE_NONE, (gobject.TYPE_PYOBJECT,))

    def taxel_scope_selected_cb(self, event):
        self.emit('scope-clicked', self.scope.index(event.canvas))

    def cdc_combo_changed_cb(self, w):
        model = w.get_model()
        index = w.get_active()
        self.cdc = index
        self.reset_frame()
        return

    def reset(self):
        for i in range(12):
            self.scope[i].reset()

    def reset_frame(self):
        self.offset = self.get_offset(self.cdc)

        self.reset()

        self.taxel_frame.set_label('Capacitive-to-Digital Converter %d/32'%int(self.cdc+1))

    def forward_button_clicked_cb(self, widget):
        self.cdc += 1
        if self.cdc == 32:
            self.cdc = 0
 
        self.reset_frame()

    def back_button_clicked_cb(self, widget):
        self.cdc -= 1
        if self.cdc == -1:
            self.cdc = 31

        self.reset_frame()

    def get_line(self, cdc):
        return math.floor(cdc/4)

    def get_offset(self, cdc):
        return int((self.get_line(cdc)*48) + (cdc%4)*3)

    def get_current_offset(self, taxel=0):
        return int(self.offset + math.floor(taxel/3)*12 + (taxel%3))

    def update(self, taxel_value, sleep=10):
        taxel_offset = self.offset

        idx = 0
        for i in range(4):
            for j in range(3):
                self.scope[idx].plot(taxel_value[taxel_offset + j])
                idx += 1
            taxel_offset += 12

        time.sleep(1.0/sleep)
        return True

class StatisticsPanel(gtk.VBox):
    def __init__(self):
        gtk.VBox.__init__(self, False)
        self.set_size_request(240, -1)
        self.set_spacing(5)
        self.set_border_width(5)

        # Add stats info
        info_table = gtk.Table(7, 2, False)
        info_table.set_row_spacings(5)
        info_table.set_col_spacings(5)
        self.pack_start(info_table, False, True)

        label = gtk.Label("Variance:")
        self.variance_label = gtk.Label("Nil")
        info_table.attach(label, 0, 1, 0, 1)
        info_table.attach(self.variance_label, 1, 2, 0, 1)

        label = gtk.Label("Standard deviation:")
        self.stdev_label = gtk.Label("Nil")
        info_table.attach(label, 0, 1, 1, 2)
        info_table.attach(self.stdev_label, 1, 2, 1, 2)

        label = gtk.Label("Mean:")
        self.average_label = gtk.Label("Nil")
        info_table.attach(label, 0, 1, 2, 3)
        info_table.attach(self.average_label, 1, 2, 2, 3)

        label = gtk.Label("Number of samples:")
        self.nbsamples_label = gtk.Label("Nil")
        info_table.attach(label, 0, 1, 3, 4)
        info_table.attach(self.nbsamples_label, 1, 2, 3, 4, gtk.EXPAND)

        label = gtk.Label("Current value:")
        self.currentvalue_label = gtk.Label("Nil")
        info_table.attach(label, 0, 1, 4, 5)
        info_table.attach(self.currentvalue_label, 1, 2, 4, 5, gtk.EXPAND)

        label = gtk.Label("Stop after (samples):")
        info_table.attach(label, 0, 1, 5, 6)
        self.spin_button = gtk.SpinButton(gtk.Adjustment(lower=0, upper=10000, step_incr=1))
        self.spin_button.set_value(0)
        info_table.attach(self.spin_button, 1, 2, 5, 6, gtk.EXPAND)

        label = gtk.Label("Cue recording (secs):")
        info_table.attach(label, 0, 1, 6, 7)
        self.spin_button_timer = gtk.SpinButton(gtk.Adjustment(lower=0, upper=3600, step_incr=1))
        self.spin_button_timer.set_value(0)
        info_table.attach(self.spin_button_timer, 1, 2, 6, 7, gtk.EXPAND)

        label = gtk.Label("Sample all taxels:")
        info_table.attach(label, 0, 1, 7, 8)
        self.sample_all_checkbutton = gtk.CheckButton()
        self.sample_all_checkbutton.connect('toggled', self.sample_all_cb)
        info_table.attach(self.sample_all_checkbutton, 1, 2, 7, 8, gtk.EXPAND)

        # Add close button        
        align = gtk.Alignment(0.0, 0.0, 0, 0)
        self.pack_start(align, True, True)
        self.button_box = gtk.HBox()
        align.add(self.button_box)

        self.button_start = gtk.Button(stock=gtk.STOCK_MEDIA_RECORD)
        self.button_start.set_use_stock(True)
        self.button_start.connect('clicked', self.start_button_cb)
        self.button_box.pack_start(self.button_start)

        self.button_saveas = gtk.Button(stock=gtk.STOCK_SAVE_AS)
#       self.button_saveas.set_sensitive(False)
#       self.button_saveas.connect('clicked', self.saveas_button_cb)
#       self.button_box.pack_start(self.button_saveas)

        self.samples = []
        self.timestamps = []
        self.recording = False
        self.filename = None

        self.time_start = None

    def sample_all_cb(self, w):
       if w.get_active() == True:
           print 'Sampling all'

    def timer_cb(self, w):
       print 'Timeout \n'
       self.start_recording()
       return False

    def saveas_button_cb(self, w):
       dialog = gtk.FileChooserDialog('Save Raw Taxel Values', None, gtk.FILE_CHOOSER_ACTION_SAVE, (gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL, gtk.STOCK_SAVE, gtk.RESPONSE_OK))
       dialog.set_filename('test.dat')
       dialog.run()
       print 'File name %s', dialog.get_filename()
       self.filename = dialog.get_filename()
       dialog.destroy()

    def analyze(self):
        filename = self.filename
        if self.filename == None:
            filename = 'taxel_raw_%s.dat'%str(datetime.datetime.now().isoformat())

        print 'Dumping to file %s'%filename
        fout = open(filename, 'w')
        content = ""

        for i in range(len(self.samples)):
            # One taxel sampling
            if len(self.samples[0]) == 1:
                line = '%f    %f\n'%(self.timestamps[i], self.samples[i][0])
            else:
                line = '%f    '%self.timestamps[i]
                for j in range(len(self.samples[i])):
                    line = line + '    %f'%self.samples[i][j]
                line = line + '\n'
            content = content + line

        fout.write(content)
        fout.close()

        if len(self.samples[0]) == 1:
            samples = [u for v in self.samples for u in v]
            self.variance_label.set_text(str(np.var(samples)))
            self.stdev_label.set_text(str(np.std(samples)))
            self.average_label.set_text(str(np.mean(samples)))
        else:
            variances = [np.var(s) for s in self.samples]
            devs = [np.std(s) for s in self.samples]
            avgs = [np.mean(s) for s in self.samples]

            self.variance_label.set_text(str(np.mean(variances)))
            self.stdev_label.set_text(str(np.mean(devs)))
            self.average_label.set_text(str(np.mean(avgs)))

    def start_recording(self):
            self.samples = []
            self.timestamps = []
            self.button_start.set_label('Stop')
            self.time_start = time.time()
            self.recording = True
            self.button_saveas.set_sensitive(False)
            self.sample_all_checkbutton.set_sensitive(False)

    def stop_recording(self):
            self.button_start.set_label('Record')
            self.recording = False
            self.analyze()
            self.button_saveas.set_sensitive(True)
            self.sample_all_checkbutton.set_sensitive(True)

    def start_button_cb(self, w):
        if self.recording == False:
            self.timer = gobject.timeout_add(self.spin_button_timer.get_value_as_int()*1000, self.timer_cb, self)
        else:
            self.stop_recording()

    def get_seconds_elapsed(self):
        return time.time() - self.time_start

    def update(self, taxel_values, index):
        value = taxel_values[index]
        self.currentvalue_label.set_text(str(value))
                
        if self.recording == False:
            return
        else:
            # Check if we have enough samples and must stop
            if self.spin_button.get_value_as_int() != 0:
                 if len(self.samples) == self.spin_button.get_value_as_int():
                     self.stop_recording()
                     return

            # Sampling all taxels
            if self.sample_all_checkbutton.get_active():
                self.samples.append(taxel_values)
            else:
                  self.samples.append([value])

            self.timestamps.append(self.get_seconds_elapsed())
            self.nbsamples_label.set_text(str(len(self.samples)))               

class MekaAboutDialog(gtk.AboutDialog):
    ''' The About Dialog for Meka '''
    def __init__(self):
        gtk.AboutDialog.__init__(self)
        self.set_name("Taxel Scope")
        self.set_version("1.0rc1")
        self.set_copyright("Meka Robotics LLC 2011")
        #self.set_license("GPL2")
        self.set_website("http://mekabot.com")
        self.set_authors(['Pierre-Luc Bacon <pierrelucbacon@mekabot.com>'])


class ScopeWindow(gtk.Window):
    ''' A GTK Window that contains either a CdcScope or a TaxelScopeControl'''
    def __init__(self):
        gtk.Window.__init__(self)
        self.connect("destroy", lambda x: gtk.main_quit())
        self.set_default_size(800, 600)
        self.set_title("Taxel Scope")

        self.rate = 10
        self.taxel_selected = 0

        # Add menu bar to main vbox
        self.menu_items = (
            ( "/_File",         None,         None, 0, "<Branch>" ),
            #( "/File/_New",     "<control>N", self.print_hello, 0, None ),
            #( "/File/_Open",    "<control>O", self.print_hello, 0, None ),
            #( "/File/_Save",    "<control>S", self.print_hello, 0, None ),
            #( "/File/Save _As", None,         None, 0, None ),
            #( "/File/sep1",     None,         None, 0, "<Separator>" ),
            ( "/File/Quit",     "<control>Q", gtk.main_quit, 0, None ),
            ( "/_Options",      None,         None, 0, "<Branch>" ),
            ( "/Options/Display Rate",  None, self.change_display_rate_cb, 0, None ),
            ( "/_Help",         None,         None, 0, "<Branch>" ),
            ( "/_Help/About",   None,         self.show_about_cb, 0, None ),
            )

        menu_bar = self.get_main_menu()

        # Add window vbox to window
        self.window_vbox = gtk.VBox()
        self.add(self.window_vbox)

        # Add menu bar to window vbox
        self.window_vbox.pack_start(menu_bar, False, False)

        # Add pane to window
        self.pane = gtk.HPaned()
        self.window_vbox.pack_start(self.pane, True, True)

        # Add statistics panel to the left
        self.stats_panel = StatisticsPanel()
        self.pane.pack1(self.stats_panel)

        # Add vbox to pane right
        self.main_vbox = gtk.VBox(False, 10)
        self.pane.pack2(self.main_vbox)

        # Add notebook to vbox
        self.notebook = gtk.Notebook()
        self.notebook.set_show_tabs(False)
        self.notebook.set_show_border(False)
        self.notebook.connect('switch-page', self.notebook_page_cb)
        self.main_vbox.pack_start(self.notebook, True, True)

        # Add taxel grid to notebook
        self.cdc_scope = CdcScope()
        self.cdc_scope.connect('scope-clicked', self.zoom_in_scope_cb)
        self.notebook.insert_page(self.cdc_scope)

        # Add single scope to notebook
        self.taxel_scope = TaxelScope()
        self.taxel_scope.register(self.zoom_out_scope_cb)
        self.notebook.insert_page(self.taxel_scope)

        #self.main_vbox.pack_start(self.cdc_scope, True, True)

        # Proxy to M3 server
        self.emitter = M3SkinEmitter()
        self.emitter.start()
        self.emitter.step()
        gobject.idle_add(self.update)

    def __del__(self):
        self.emitter.stop()
        print 'Deleted'
        
    def notebook_page_cb(self, notebook, page, page_num):
        if page_num == 0:
            self.stats_panel.hide()
        else:
            self.stats_panel.show()

    def zoom_in_scope_cb(self, w, index):
        self.notebook.set_current_page(1)
        self.taxel_selected = index

    def zoom_out_scope_cb(self, event):
        self.notebook.set_current_page(0)
        self.taxel_selected = 0

    def show_about_cb(self, w, data):
        about = MekaAboutDialog()

        about.set_transient_for (self)
        about.set_position (gtk.WIN_POS_CENTER_ON_PARENT)
        about.run()
        about.destroy()

    def change_display_rate_cb(self, w, data):
        dialog = self.get_rate_dialog()
        dialog.show_all()
        response_id = dialog.run()
        dialog.destroy()

    def get_main_menu(self):
        accel_group = gtk.AccelGroup()

        item_factory = gtk.ItemFactory(gtk.MenuBar, "<main>", accel_group)

        item_factory.create_items(self.menu_items)

        self.add_accel_group(accel_group)

        self.item_factory = item_factory

        return item_factory.get_widget("<main>")

    def rate_spinbox_changed_cb(self, w):
        self.rate = w.get_value_as_int()

    def get_rate_dialog(self):
        dialog = gtk.Dialog("Open Display", self, gtk.DIALOG_MODAL, (gtk.STOCK_CLOSE, gtk.RESPONSE_CLOSE))
  
        dialog.set_default_response(gtk.RESPONSE_OK)

        spin_button = gtk.SpinButton(gtk.Adjustment(lower=0, upper=30, step_incr=1))
        spin_button.set_value(self.rate)
        spin_button.connect('value-changed', self.rate_spinbox_changed_cb)
        dialog_label = gtk.Label("Set the display rate (in Hertz) ")
  
        dialog.vbox.add(dialog_label)
        dialog.vbox.add(spin_button)
  
        return dialog
        
    def update(self):
        self.emitter.step()

        if self.notebook.get_current_page() == 0:
            self.cdc_scope.update(self.emitter.get_taxel_value(), self.rate)
        else:
            self.taxel_scope.plot(self.emitter.get_taxel_value()[self.cdc_scope.get_current_offset(self.taxel_selected)])
            self.stats_panel.update(self.emitter.get_taxel_value(), self.cdc_scope.get_current_offset(self.taxel_selected))
        return True

if __name__ == '__main__':
    print "Starting..."
    scope = ScopeWindow()
    scope.show_all()

    gtk.main()
