import roslib
roslib.load_manifest('sandbox_marc_darpa_m3')
import rospy
from hrl_haptic_manipulation_in_clutter_msgs.msg import SkinContact
import threading

class CountContactNumbers():
    def __init__(self, skin_topic):
        rospy.init_node('counting_contact_occurrences')
        #self.var_imped_pub = rospy.Publisher('/sim_arm/command/joint_impedance', MechanicalImpedanceParams)
        self.contact_count_sub = rospy.Subscriber(skin_topic, SkinContact, self.contact_count_callback)
        self.num_change_of_contacts = 0.
        self.num_control_steps_contact_changes = 0.
        self.end_time = 0.0
        self.prev_num_of_contacts = None
        self.start_time = None
        self.lock = threading.RLock()
        
    def contact_count_callback(self, msg):
        self.lock.acquire()
        if self.prev_num_of_contacts == None:
            self.start_time = rospy.Time.now().to_time()
            self.prev_num_of_contacts = len(msg.link_names)
        else:
            if abs(self.prev_num_of_contacts - len(msg.link_names)) > 0:
                self.num_control_steps_contact_changes = self.num_control_steps_contact_changes + 1.
            self.num_change_of_contacts = self.num_change_of_contacts + abs(self.prev_num_of_contacts - len(msg.link_names))
            self.prev_num_of_contacts = len(msg.link_names)
        self.end_time = rospy.Time.now().to_time()
        self.lock.release()

if __name__ == '__main__':
    # import optparse
    # p = optparse.OptionParser()

    # p.add_option('--optimize', action='store_true', dest='optimize',
    #              default=False, help='run continuous optimization to vary compliance')

    # opt, args = p.parse_args()

    skin_topic = '/skin/contacts'
    contact_counter = CountContactNumbers(skin_topic)

    while not rospy.is_shutdown():
        if contact_counter.start_time != None:
            print "number of total change in contacts : \t", contact_counter.num_change_of_contacts 
            print "number of times any change in contact state :\t", contact_counter.num_control_steps_contact_changes
            print "elapsed time : \t", contact_counter.end_time-contact_counter.start_time
        rospy.sleep(0.01)
