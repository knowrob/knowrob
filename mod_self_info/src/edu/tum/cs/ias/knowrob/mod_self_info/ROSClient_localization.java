package edu.tum.cs.ias.knowrob.mod_self_info;

import ros.*;
import ros.communication.*;
import ros.pkg.geometry_msgs.msg.*;

public class ROSClient_localization {

        static Boolean rosInitialized = false;
        static Ros ros;
        static NodeHandle n;
	static Subscriber.QueueingCallback<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped> callback;
	static Subscriber<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped> sub;
	static double[] r;


        /**
         * Constructor: initializes the ROS environment
         *
         * @param node_name A unique node name
         */
        public ROSClient_localization(String node_name) 
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

		try 
		{
			callback = new Subscriber.QueueingCallback<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped>();
	  		sub = n.subscribe("/amcl_pose", new ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped(), callback, 10);
		}
		catch (RosException e) 
		{
                        ros.logError("ROSClient: Call to service /amcl_pose failed");
                }
		
		r = new double[7];

		r[0] = -1;
		r[1] = -1;
		r[2] = -1;
		r[3] = -1;
		r[4] = -1;
		r[5] = -1;
		r[6] = -1;

		n.spinOnce();

        }


        /**
         * Call the table_objects service and return the result
         * @return An array of CollisionObjects referring to the objects detected on the table
         */
        public double[] getLocation() 
	{
		try
		{
			while (!callback.isEmpty())
			{
				ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped data = callback.pop();
				r[0] = data.pose.pose.position.x;
				r[1] = data.pose.pose.position.y;
				r[2] = data.pose.pose.position.z;
				r[3] = data.pose.pose.orientation.x;
				r[4] = data.pose.pose.orientation.y;
				r[5] = data.pose.pose.orientation.z;
				r[6] = data.pose.pose.orientation.w;
			}

		}
		catch (java.lang.InterruptedException e)
		{
                        ros.logError("ROSClient: Call to service /amcl_pose failed");
		}

  		n.spinOnce();              

                return r;
        }

	public double[] getLocationByPooling() 
	{
		double[] state = new double[7];
		state[0] = -1;
		state[1] = -1;
		state[2] = -1;
		state[3] = -1;
		state[4] = -1;
		state[5] = -1;
		state[6] = -1;


		while (state[0] == -1 && state[1] == -1 && state[2] == -1 &&
			state[3] == -1 && state[4] == -1 && state[5] == -1 && state[6] == -1)
		{
			double[] currentPose = getLocation();
			state[0] = currentPose[0];
			state[1] = currentPose[1];
			state[2] = currentPose[2];			
			state[3] = currentPose[3];
			state[4] = currentPose[4];
			state[5] = currentPose[5];
			state[6] = currentPose[6];
                }
		double[] matrix = quaternionToMatrix(state);
                return matrix;
        }

	protected static double[] quaternionToMatrix(double[] pose) 
	{

	    double[] m = new double[16];

	    double xx = pose[3] * pose[3];
	    double xy = pose[3] * pose[4];
	    double xz = pose[3] * pose[5];
	    double xw = pose[3] * pose[6];

	    double yy = pose[4] * pose[4];
	    double yz = pose[4] * pose[5];
	    double yw = pose[4] * pose[6];

	    double zz = pose[5] * pose[5];
	    double zw = pose[5] * pose[6];

	    m[0]  = 1 - 2 * ( yy + zz );
	    m[1]  =     2 * ( xy - zw );
	    m[2]  =     2 * ( xz + yw );
	    m[3]  = pose[0];

	    m[4]  =     2 * ( xy + zw );
	    m[5]  = 1 - 2 * ( xx + zz );
	    m[6]  =     2 * ( yz - xw );
	    m[7]  = pose[1];

	    m[8]  =     2 * ( xz - yw );
	    m[9]  =     2 * ( yz + xw );
	    m[10] = 1 - 2 * ( xx + yy );
	    m[11] = pose[2];

	    m[12]=0;
	    m[13]=0;
	    m[14]=0;
	    m[15]=1;
	    return m;
	}

        /*
         * Test method: call the tabletop_object_detector and print results
         */
        public static void main(String[] args) 
	{
		ROSClient_localization d = new ROSClient_localization("knowrob_localization_test_123");

                double[] res = d.getLocationByPooling();

                System.out.println(res[0]);
                
        }

}
