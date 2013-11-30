#include "simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_arm");
    ros::NodeHandle n;
    Simulator simulator(n);
    ros::Subscriber sub1 = n.subscribe("/sim_arm/command/jep", 100, &Simulator::JepCallback, &simulator);
    ros::Subscriber sub2 = n.subscribe("/sim_arm/command/joint_impedance", 100, &Simulator::ImpedanceCallback, &simulator);

    //these are used for synchronization with controller
    // ros::Publisher waiting_pub = n.advertise<std_msgs::Bool>("/epc_skin/local_controller/got_start_optimization", 100);
    // ros::Publisher done_waiting_pub = n.advertise<std_msgs::Bool>("/epc_skin/local_controller/got_finish_optimization", 100);

    std_msgs::Bool wait_state;
    std_msgs::Bool running_state;

    tf::TransformBroadcaster br;                                                
    tf::Transform tf_transform;
    double q_pub_time(0.0);
    double calc_torque_time(0.0);
    double skin_update_time(0.0);
    double clock_pub_time(0.0);
    double last_contact_time(0.0);

    ROS_INFO("Before most things \n");

    int resol;
    //    float resolution;
    while (n.getParam("/m3/software_testbed/resolution", resol) == false)
    {
        sleep(0.1);
    }
    double resolution = (double) resol;

    ROS_INFO("After getting first parameter \n");

    dInitODE();
    // setup pointers to drawstuff callback functions

    world.setGravity(0, 0, 0);

    ROS_INFO("Before create_robot \n");

    // make robot and go to starting configuration.
    simulator.create_robot();
    ROS_INFO("Before going to initial position \n");
    simulator.go_initial_position(); // initial jep defined inside this function.
    ROS_INFO("After going to initial position \n");

    // add obstacles.
    simulator.create_movable_obstacles(); //call this first for stupid ROS param server sync.
    simulator.create_compliant_obstacles(); 
    simulator.create_fixed_obstacles();
    ROS_INFO("Starting Simulation now ... \n");

    double t_now = get_wall_clock_time() - timestep;
    double t_expected;

    while (ros::ok())
    {
        space.collide(&simulator, &simulator.nearCallback);

        // simulation will not run faster than real-time.
        t_expected = t_now + timestep;
        t_now = get_wall_clock_time();
        if (t_now < t_expected)
            usleep(int((t_expected - t_now)*1000000. + 0.5));

        world.step(timestep);

        cur_time += timestep;
        rosgraph_msgs::Clock c;
        c.clock.sec = int(cur_time);
        c.clock.nsec = int(1000000000*(cur_time-int(cur_time)));

        simulator.sense_forces();

	if(cur_time - clock_pub_time >= 0.002)
        {
	    clock_pub_time = cur_time;
            simulator.clock_pub.publish(c);
        }

        simulator.get_joint_data();

	// if (cur_time - simulator.last_jep_time >= 0.01)
	//   {
	//     while (simulator.waiting == true) //&& cur_time - simulator.last_jep_time >= 0.01)
	//     {
	//       ros::spinOnce();
	//       wait_state.data = simulator.waiting;
	//       running_state.data = simulator.running;
	//       waiting_pub.publish(wait_state);
	//       done_waiting_pub.publish(running_state);
	//     }
	//   }



	if (cur_time - q_pub_time >= 0.01) 
        {
	    q_pub_time = cur_time;
	    
            simulator.publish_angle_data();

            tf_transform.setOrigin(tf::Vector3(0, 0, 0.0));
            tf_transform.setRotation(tf::Quaternion(0, 0, 0));

            br.sendTransform(tf::StampedTransform(tf_transform,
						  ros::Time::now(), "/world",
						  "/torso_lift_link"));
        }

        simulator.update_friction_and_obstacles();

	if (cur_time - skin_update_time >= 0.01)
        {
	    skin_update_time = cur_time;
            simulator.update_linkage_viz();
            simulator.update_taxel_simulation(resolution);
            simulator.publish_imped_skin_viz();
        }


	//if (cur_time - simulator.last_jep_time >= 0.01 && cur_time > 20.0)
	if (cur_time - simulator.last_jep_time >= 0.01 && simulator.controller_running == true)
	  {
	    m.lock();
	    simulator.controller_running = false;
	    m.unlock();

	    int counter = 0;
	    //while (cur_time - simulator.last_jep_time >= 0.01  and counter <= 100)
	    //while (cur_time - simulator.last_jep_time >= 0.01 or simulator.controller_running == true)
	    while (cur_time - simulator.last_jep_time >= 0.01 and counter <= 100000)  // and what????!!!!)
	      {
		ros::spinOnce();
		counter = counter + 1;
		//usleep(10000);
		//sleep(0.010000);
	      }

	    // m.lock();
	    // simulator.controller_running = false;
	    // m.unlock();


	  }


	if (cur_time - calc_torque_time >= 0.001)
        {
	    calc_torque_time = cur_time;
            simulator.calc_torques();
	    // m.lock();
	    // // wait_state.data = simulator.waiting;
	    // // running_state.data = simulator.running;
	    // // waiting_pub.publish(wait_state);
	    // // done_waiting_pub.publish(running_state);
            // m.unlock();
        }

	if (cur_time - last_contact_time >= 1.0 && simulator.controller_running == true)
          {
            last_contact_time = cur_time;
            simulator.publish_contact_table();
          }

        simulator.set_torques();
        simulator.clear();

        ros::spinOnce();
    }

    dCloseODE();
}


