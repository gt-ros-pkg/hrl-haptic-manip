#include "simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sim_arm");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;                                                
    tf::Transform tf_transform;

    tf_transform.setOrigin(tf::Vector3(0., 0., 0.0));
    tf_transform.setRotation(tf::Quaternion(0., 0., 0.));

    br.sendTransform(tf::StampedTransform(tf_transform,
					  ros::Time::now(), "/world",
					  "/torso_lift_link"));

    Simulator simulator(n);
    ros::Subscriber sub1 = n.subscribe("/sim_arm/command/jep", 100, &Simulator::JepCallback, &simulator);
    ros::Subscriber sub2 = n.subscribe("/sim_arm/command/joint_impedance", 100, &Simulator::ImpedanceCallback, &simulator);
    int torque_step(0);
    int q_pub_step(0);
    int skin_step(0);
    int clock_pub_step(0);


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
    //ROS_INFO("Before going to initial position \n");
    //simulator.go_initial_position(); // initial jep defined inside this function.
    //ROS_INFO("After going to initial position \n");

    // add obstacles.

    ROS_INFO("Starting Simulation now ... \n");

    double t_now = get_wall_clock_time() - timestep;
    double t_expected;

    while (ros::ok())
    {
      //space.collide(&simulator, &simulator.nearCallback);

        //simulation will not run faster than real-time.
        t_expected = t_now + timestep;
        t_now = get_wall_clock_time();
        if (t_now < t_expected)
            usleep(int((t_expected - t_now)*1000000. + 0.5));

        world.step(timestep);

        cur_time += timestep;
        rosgraph_msgs::Clock c;
        c.clock.sec = int(cur_time);
        c.clock.nsec = int(1000000000*(cur_time-int(cur_time)));

        //simulator.sense_forces();

        clock_pub_step++;
        torque_step++;
        q_pub_step++;
        skin_step++;

        if (clock_pub_step >= 0.002/timestep)
        {
            simulator.clock_pub.publish(c);
            clock_pub_step = 0;
        }

        simulator.get_joint_data();
        if (q_pub_step >= 0.01/timestep)
        {
            simulator.publish_angle_data();
            q_pub_step = 0;

            tf_transform.setOrigin(tf::Vector3(0., 0., 0.0));
            tf_transform.setRotation(tf::Quaternion(0., 0., 0.));

            br.sendTransform(tf::StampedTransform(tf_transform,
                        ros::Time::now(), "/world",
                        "/torso_lift_link"));
        }

        //simulator.update_friction_and_obstacles();

        if (skin_step >= 0.01/timestep)
        {
            simulator.update_linkage_viz();
            simulator.update_taxel_simulation(resolution);
            simulator.publish_imped_skin_viz();
            skin_step = 0;
        }

        if (torque_step >= 0.001/timestep)
        {
            simulator.calc_torques();
            torque_step = 0;
        }

        simulator.set_torques();
        simulator.clear();

        ros::spinOnce();
    }

    dCloseODE();
}


