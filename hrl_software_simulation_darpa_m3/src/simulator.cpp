#include "simulator.h"

//double get_wall_clock_time()
//{
//    timeval tim;
//    gettimeofday(&tim, NULL);
//    double t1=tim.tv_sec+(tim.tv_usec/1000000.0);
//    return t1;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_arm");
    ros::NodeHandle n;
    Simulator simulator(n);
    ros::Subscriber sub1 = n.subscribe("/sim_arm/command/jep", 100, &Simulator::JepCallback, &simulator);
    ros::Subscriber sub2 = n.subscribe("/sim_arm/command/joint_impedance", 100, &Simulator::ImpedanceCallback, &simulator);
    ros::Subscriber sub3 = n.subscribe("/sim_arm/command/base_ep", 100, &Simulator::BaseEpCallback, &simulator);
    ros::Subscriber sub4 = n.subscribe("/sim_arm/command/base_joint_impedance", 100, &Simulator::BaseImpedanceCallback, &simulator);

    tf::TransformBroadcaster br;                                                
    tf::Transform tf_transform;
    int torque_step(0);
    int q_pub_step(0);
    int skin_step(0);
    int clock_pub_step(0);
    double last_contact_time(0.0);

    ROS_INFO("Before most things \n");

    ROS_INFO("After getting first parameter \n");

    dInitODE();
    // setup pointers to drawstuff callback functions

    //simulator.world.setGravity(-9.8, 0, 0);
    simulator.world.setGravity(0, 0, 0);

    ROS_WARN("Before create_robot \n");

    // make robot and go to starting configuration.
    simulator.create_robot();
    ROS_WARN("Before going to initial position \n");

    simulator.go_initial_position(); // initial jep defined inside this function.
    ROS_INFO("After going to initial position \n");
    
    // add obstacles. Order should be fixed.
    simulator.create_movable_obstacles(); //call this first for stupid ROS param server sync.
    simulator.create_compliant_obstacles(); 
    simulator.create_fixed_obstacles();
    ROS_INFO("Starting Simulation now ... \n");

    double t_now = get_wall_clock_time() - simulator.timestep;
    double t_expected;

    while (ros::ok())
    {
        simulator.space.collide(&simulator, &simulator.nearCallback);
        // simulation will not run faster than real-time. - advait 2011
	/* this section may no longer be necessary though because
	   we are no synchronizing the mpc controller and simulation 
	   at least in some branches of the git code. - marc Sept 2012 */
        t_expected = t_now + simulator.timestep; //sec
        t_now = get_wall_clock_time();

        if (t_now < t_expected)
            usleep(int((t_expected - t_now)*1000000. + 0.5));
        simulator.world.step(simulator.timestep);

        simulator.cur_time += simulator.timestep;
        rosgraph_msgs::Clock c;
        c.clock.sec = int(simulator.cur_time);
        c.clock.nsec = int(1000000000*(simulator.cur_time-int(simulator.cur_time)));
        simulator.sense_forces();

        clock_pub_step++;
        torque_step++;
        q_pub_step++;
        skin_step++;
        if (clock_pub_step >= 0.002/simulator.timestep)
        {
            simulator.clock_pub.publish(c);
            clock_pub_step = 0;
        }

        simulator.get_joint_data();


        if (q_pub_step >= 0.01/simulator.timestep)
        {
            simulator.publish_angle_data();
            q_pub_step = 0;

            simulator.publish_base_pos_data();

            if (simulator.use_mobile_base == false)
            {              
                tf_transform.setOrigin(tf::Vector3(0, 0, 0.0));
                tf_transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));

                br.sendTransform(tf::StampedTransform(tf_transform,
                                                      ros::Time::now(), "/world",
                                                      "/torso_lift_link"));
            }            
            else{
                
                tf_transform.setOrigin(tf::Vector3(0.,0.,0.));
                tf_transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));
              
                br.sendTransform(tf::StampedTransform(tf_transform,
                                                      ros::Time::now(), "/world",
                                                      "/base_link"));

                //printf("%f -- %f \n", simulator.mobile_base_pos.data[0], simulator.mobile_base_pos.data[1]);

                tf_transform.setOrigin(tf::Vector3(simulator.mobile_base_pos.data[0], simulator.mobile_base_pos.data[1], 0.0));
                tf_transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));

                br.sendTransform(tf::StampedTransform(tf_transform,
                                                      ros::Time::now(), "/base_link",
                                                      "/torso_lift_link"));

                tf_transform.setOrigin(tf::Vector3(simulator.mobile_base_pos.data[0], simulator.mobile_base_pos.data[1], 0.0));
                tf_transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));

                br.sendTransform(tf::StampedTransform(tf_transform,
                                                      ros::Time::now(), "/world",
                                                      "/torso_lift_link"));

            }
        }

        simulator.update_friction_and_obstacles();

        if (skin_step >= 0.01/simulator.timestep)
        {
            simulator.update_linkage_viz();
            simulator.update_taxel_simulation();
	    simulator.update_proximity_simulation();
            simulator.publish_imped_skin_viz();
            skin_step = 0;
        }

        if (torque_step >= 0.001/simulator.timestep)
        {
            simulator.calc_torques();
            torque_step = 0;
        }

	if (simulator.cur_time - last_contact_time >= 1.0)
          {
            last_contact_time = simulator.cur_time;
            simulator.publish_contact_table();
          }


        simulator.set_torques();
        simulator.clear();

        ros::spinOnce();
    }

    dCloseODE();
}


