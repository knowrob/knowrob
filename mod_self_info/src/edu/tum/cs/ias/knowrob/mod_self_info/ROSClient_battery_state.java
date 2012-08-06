package edu.tum.cs.ias.knowrob.mod_self_info;

import ros.*;
import ros.communication.*;
import ros.pkg.geometry_msgs.msg.Point;
import ros.pkg.geometry_msgs.msg.Quaternion;
import ros.pkg.pr2_msgs.msg.*;

public class ROSClient_battery_state {

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n;
	static Subscriber.QueueingCallback<ros.pkg.pr2_msgs.msg.PowerState> callback;
	static Subscriber<ros.pkg.pr2_msgs.msg.PowerState> sub;
	static double r;


        /**
         * Constructor: initializes the ROS environment
         *
         * @param node_name A unique node name
         */
        public ROSClient_battery_state(String node_name) 
	{
                initRos(node_name);
        }


        /**
         * Initialize the ROS environment if it has not yet been initialized
         *
         * @param node_name A unique node name
         */
        protected static void initRos(String node_name) 
	{

                ros = Ros.getInstance();

                if(!Ros.getInstance().isInitialized()) {
                        ros.init(node_name);
                }
                n = ros.createNodeHandle();


		callback = new Subscriber.QueueingCallback<ros.pkg.pr2_msgs.msg.PowerState>();
	  	
		try 
		{
			sub = n.subscribe("/power_state", new ros.pkg.pr2_msgs.msg.PowerState(), callback, 10);
		}
		catch (RosException e) 
		{
                        ros.logError("ROSClient: Error while subscribing power_state topicc");
                }
		r = 0;	
                n.spinOnce();
               
        }


        /**
         * Call the table_objects service and return the result
         * @return An array of CollisionObjects referring to the objects detected on the table
         */
        public double getPower() 
	{
		
		try
		{	
			while (!callback.isEmpty()) r = callback.pop().relative_capacity;
		}
		catch (java.lang.InterruptedException e)
		{
                        ros.logError("ROSClient: Stack is empty");
		}
		n.spinOnce();
                return r;
        }


	public double getPowerByPooling() 
	{
		double state = 0;
		
		while (state == 0)
		{
			state = getPower();
                	// System.out.println(res);
                }
                return state;
        }

        /*
         * Test method: call the tabletop_object_detector and print results
         */
        public static void main(String[] args) 
        {

                ROSClient_battery_state d = new ROSClient_battery_state("knowrob_battery_state_test_123");
		int i =1;
		while (i == 1)
		{
			double res = d.getPowerByPooling();
                	System.out.println(res);
                }
        }

}
