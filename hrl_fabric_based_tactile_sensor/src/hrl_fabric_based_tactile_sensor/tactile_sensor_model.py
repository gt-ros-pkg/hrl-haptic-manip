#!/usr/bin/python

# Charlie Kemp's initial attempt to model the force -> digital signal
# curves for a single taxel. 
#
# + First version written on June 4, 2012. 
# + Cleaned up, documented, and made minor edits June 5, 2012

import matplotlib.pylab as pl

def logistic(t):
    return(1.0/(1.0 + pl.exp(-t)))

def norm_logistic(t):
    return(2.0 * (logistic(t)-0.5))

def abbot_curve(t):
    # total hack in attempt to make curve that looks like the Abbot's
    # Curve in the paper haven't visualized it, yet...
    #
    # assume t in in range [0, 1]
    tmp = logistic((t-0.5) * 12.0)
    return(tmp)

class TaxelModel:
    '''Attempts to model the digital signal that results from a normal
    force applied to a taxel. It assumes that the force is uniformly
    distributed over an area. The contact area is specified as a
    percentage of the taxel area.'''
    def __init__(self, contact_area_percent=50.0):

        ######################################
        # begin: parameters to be specified

        self.contact_area_percent = contact_area_percent

        # resistor that is in series with the taxel (Ohms)
        self.r1 = 47.0 

        # total voltage across the taxel and r1, which are in serise (Volts)
        self.vtot = 5.0 

        # the maximum resistance of the taxel when no pressure is applied (Ohms)
        self.rtax_max = 50.0 

        # the minimum force that will be applied to the taxel (Newtons)
        self.fz_min = 0.0 

        # the maximum force that will be applied to the taxel (Newtons)
        self.fz_max = 45.0 

        # the number of bits for the analog to digital conversion 
        self.adc_bits = 10 

        # the pressure sensitive area of the taxel (meters^2)
        self.taxel_area = 0.04 * 0.04 

        # pressure that results in minimum resistance after which
        # further pressure does not result in a reduction in the
        # signal, since the sensor is saturated (Pascals = N/m^2)
        self.pressure_max = self.fz_max/(0.4 * self.taxel_area) 

        # hack to specify the minimum resistance of the taxel, which
        # is associated with the maximum pressure. for now, it's
        # specified as a percentage of the maximum resistance, which
        # is associated with 0 applied pressure (no contact)
        self.r_min_percent_of_r_no_contact = 0.001 #

        # end
        ######################################

        self.r_no_contact = self.taxel_area * self.rtax_max 
        self.r_min = self.r_no_contact * (self.r_min_percent_of_r_no_contact/100.0)
        self.fz_array = pl.arange(self.fz_min, self.fz_max, 0.001) # N
        self.adc_range = pow(2.0, self.adc_bits)
        self.volts_per_adc_unit = self.vtot/self.adc_range # V 
        self.contact_area = self.taxel_area * (self.contact_area_percent/100.0) # m^2
        self.no_contact_area = self.taxel_area - self.contact_area # m^2
        self.pressure_array = pl.array([f/self.contact_area for f in self.fz_array]) # Pascals = N/m^2
        self.rtax_array = pl.array([self.rtax(f) for f in self.pressure_array])
        self.vdigi_array = pl.array([self.output_voltage(r) for r in self.rtax_array])
        self.vdigi_max = self.output_voltage(self.rtax_max)
        self.adc_bias = self.vdigi_max/self.volts_per_adc_unit
        self.adc_array = self.vdigi_array/self.volts_per_adc_unit
        self.adc_plot = self.adc_bias - self.adc_array


    def output_voltage(self, r): 
        '''given the resistance for the entire taxel, this returns the
        voltage across the taxel, which is what the analog to digital
        converter reads'''
        return( (self.vtot/(self.r1 + r)) * r )


    def pressure2resistance(self, p):
        '''given an applied pressure, returns the resistivity of the
        contacted region of the taxel. this uses a simple linear
        model, where: 
             0 Pascals   -> r_no_contact
             pressure max -> r_min Ohms
        '''
        r = ((self.r_no_contact-self.r_min) * ((self.pressure_max - p)/self.pressure_max)) + self.r_min
        if r < self.r_min: 
            print "r<r_min = %f<%f" % (r, self.r_min)
            r = self.r_min
        elif r > self.r_no_contact:
            r = self.r_no_contact
            print "r>r_no_contact"
        return(r)


    def pressure2resistance_2(self, p):
        '''given an applied pressure, returns the resistivity of the
        contacted region of the taxel. this uses a logistic model,
        where:
             0 Pascals   -> r_no_contact
             pressure max -> r_min Ohms
        '''
        p = self.pressure_max * norm_logistic(6.0 * (p/self.pressure_max))
        norm_pressure = (self.pressure_max - p)/self.pressure_max
        r = ((self.r_no_contact - self.r_min) * norm_pressure) + self.r_min
        if r < self.r_min: 
            print "r<r_min = %f<%f" % (r, self.r_min)
            r = self.r_min
        elif r > self.r_no_contact:
            r = self.r_no_contact
            print "r>r_no_contact"
        return(r)


    def pressure2resistance_3(self, p):
        '''given an applied pressure, returns the resistivity of the
        contacted region of the taxel. this was a quick attempt to use
        a model similar to "The Working Principle of Resistive Tactile
        Sensor Cells" by Karsten Weiss and Heinz Worn. It doesn't
        work, yet?
        '''

        r_surface_resistance = self.r_min
        
        # wikipedia: 
        # young's modulud (elastic_modulus) of nylon is 2-4 GPa = 2-4 x 10^9 Pa
        # 
        # according to 
        # "Spandex Fiber Reinforced Shape Memory Polymer Composites and their Mechanical Properties"
        # Journal	Advanced Materials Research (Volume 410)
        # Volume	Processing and Fabrication of Advanced Materials
        # Online since	November, 2011
        # Authors	Jian Sun, Yan Yi Xu, Yi Jin Chen, Yan Ju Liu, Jin Song Leng
        #
        # the elastic modulus of spandex is 25 MPa = 25 x 10^6 Pa
        #
        # according to the material data sheet for EEONTEX LR-SL-PA-10E5
        # it is 69% nylon + 31% spandex
        
        e_nylon = 3.0 * 10**9
        e_spandex = 25.0 * 10**6
        # this is a very poor model and should be improved with simple
        # parallel fibers and constant displacement model or something
        elastic_modulus = (0.69 * e_nylon) + (0.31 * e_spandex)

        # uses hacked qualitative Abbot's curve, which I have not even
        # checked by visualizing
        r = (1.0/abbot_curve(1000.0 * (p/elastic_modulus))) * (10.0*r_surface_resistance)

        if r < self.r_min: 
            print "r<r_min = %f<%f" % (r, self.r_min)
            r = self.r_min
        elif r > self.r_no_contact:
            print "r>r_no_contact = %f<%f" % (r, self.r_no_contact)
            r = self.r_no_contact
        return(r)


    def rtax(self, p): 
        '''given the pressure uniformly applied across the contact
        area this returns the resistance for the entire taxel. 

        it essentially models the taxel as parallel resistors, where
        resistors in the contact area have a resistance dependent on
        the applied pressure, and resistors in the non-contact area
        have the maximum resistance. i started with a discrete model,
        and then made a continuous approximation, which appears to
        correspond with using two volumes in parallel with different
        resistivities.
        
        based on wikipedia, it looks like i've been using something
        called resistivity (rho). so, this model can use the equation
        r = rho*(length/area). if we assume length is constant, then
        this leads to r_contact = rho_contact/area_contact and the
        same for not_contact. assuming that they are in parallel. then
        R_total = 1/((area_contact/rho_contact) +
        (area_non_contact/rho_non_contact)). if we assume length
        changes, then we can make rho_contact = resistivity_contact *
        length_contact.

        this should be more carefully investigated, but
        it seems right...

        the biggest unknown is the function that converts pressure to
        resistivity. there are currently three models of this function
        in this code. fitting a parametric model to the data would be
        a good next step. probably use an optimizer like Nelder-Mead
        that only requires function evaluations and use a cost
        function that compares the fz->adc mapping to empirically
        collected data.

        '''

        # the function that converts pressure to resistivity appears
        # to be the big unknown for this model. based on the data, it
        # seems like it needs to be a non-linear function. so far,
        # i've had the best success with a logistic model

        #r_contact = self.pressure2resistance(p)
        r_contact = self.pressure2resistance_2(p) # best performance, so far
        #r_contact = self.pressure2resistance_3(p)

        r = 1.0/((self.contact_area/r_contact) + 
                 (self.no_contact_area/self.r_no_contact))

        return(r)


    def plot_fz_adc(self):
        '''plot the curve relating applied normal force to the analog
        to digital converter output. this corresponds with the
        empirically generated scatter plots from a real taxel'''
        pl.plot(self.adc_plot, self.fz_array, label="contact area = {0:.0f}%".format(self.contact_area_percent))
        pl.xlabel("ADC bias - ADC (adc_bias - adc)")
        pl.ylabel("FT_z (Force applied to tactile sensor, fz)")

    def plot_pressure_z_adc(self):
        '''plot the curve relating applied normal force to the analog
        to digital converter output. this corresponds with the
        empirically generated scatter plots from a real taxel'''
        pl.plot(self.adc_plot, self.fz_array / self.contact_area_percent, label="contact area = {0:.0f}%".format(self.contact_area_percent))
        pl.xlabel("ADC bias - ADC (adc_bias - adc)")
        pl.ylabel("pressure_z (Force applied to tactile sensor, pz)")


    def plot_rtax_vout(self):
        '''plot the curve relating the total taxel resistance and the
        voltage across the taxel, which corresponds to the voltage
        converted to a digital signal'''
        pl.plot(self.vdigi_array, self.rtax_array, label="contact area = {0:.0f}%".format(self.contact_area_percent))
        pl.xlabel("Volts at digitizer (vdigi) proportional to ADC")
        pl.ylabel("Resistance of tactile sensor (rtax)")


    def debug(self):
        '''print out many of the key member variables of the TaxelModel object'''
        print "fz_array", self.fz_array
        print "pressure_array =", self.pressure_array
        print "adc_range", self.adc_range
        print "rtax_array =", self.rtax_array
        print "volts_per_adc_unit =", self.volts_per_adc_unit
        print "vdigi_array =", self.vdigi_array
        print "adc_bias =", self.adc_bias
        print "adc_array =", self.adc_array
        print "adc_plot =", self.adc_plot



pl.title("Single Taxel Model")
for area_percent in [100.0, 80.0, 60.0, 40.0, 30.0, 20.0, 10.0, 5.0, 2.0, 1.0, 0.1, 0.001]:
    tm = TaxelModel(area_percent)
    #tm.plot_rtax_vout()
    #tm.plot_fz_adc()
    tm.plot_pressure_z_adc()
    #tm.debug()
pl.legend(loc="upper left", prop={'size':8})
pl.show()
