package org.knowrob.vis;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import ros.NodeHandle;
import ros.Publisher;
import ros.Ros;
import ros.RosException;
import ros.pkg.data_vis_msgs.msg.DataVis;
import ros.pkg.data_vis_msgs.msg.ValueList;


/**
 * Visualization module for the KnowRob knowledge base
 *
 * The class publishes diagram messages to be visualized by a
 * Web-based diagram widget
 *
 * @author tenorth@cs.uni-bremen.de
 *
 */
public class DiagramVisualization {

	static Ros ros;
	public static NodeHandle n;
	Thread diagramPublisher;

	/**
	 * Store the messages to be published
	 */
	protected Map<String, DataVis> diagrams;


	/**
	 * Constructor. Starts the marker publisher in a parallel thread.
	 */
	public DiagramVisualization() {

		initRos();

		diagrams =  new ConcurrentHashMap<String, DataVis>(8, 0.9f, 1);

		// spawn new thread that publishes the markers in the HashMap
		diagramPublisher = new Thread( new PublisherThread() );
		diagramPublisher.start();
	}


	/**
	 * Add diagram to the visualization.
	 *
	 */
	public void addDiagram(String id, String title, String type, String xlabel, String ylabel, int width, int height, String fontsize, String[][][] values) {

		DataVis data = new DataVis();

		data.id = id;
		data.title = title;
		data.xlabel = xlabel;
		data.ylabel = ylabel;
		data.width = width;
		data.height = height;
		data.fontsize = fontsize;

		if(type.equals("piechart")) {
			data.type = DataVis.TYPE_PIECHART;
		} else if(type.equals("barchart")) {
			data.type = DataVis.TYPE_BARCHART;
		} else if(type.equals("treechart")) {
			data.type = DataVis.TYPE_TREECHART;
		}

		for(String[][] val_list : values) {
			ValueList v = new ValueList();
			v.value1 = new ArrayList<String>(Arrays.asList(val_list[0]));
			v.value2 = new ArrayList<String>(Arrays.asList(val_list[1]));
			data.values.add(v);
		}

		// add marker to map
		if(data!=null) {
			synchronized (diagrams) {
				diagrams.put(id, data);
			}
		}
	}


	/**
	 * Remove diagram 'id' from the visualization.
	 *
	 * @param id Identifier of the diagram to be removed
	 */
	public void removeDiagram(String id) {

		// remove the object from the list
		synchronized (diagrams) {
			diagrams.remove(id); 
		}


		// send empty data to id to remove it
		DataVis data = new DataVis();

		data.id = id;
		data.title = "";
		data.xlabel = "";
		data.ylabel = "";
		data.width = 0;
		data.height = 0;
		data.fontsize = "";
		data.type = 0;
		ValueList v = new ValueList();
		v.value1 = new ArrayList<String>();
		v.value2 = new ArrayList<String>();
		data.values.add(v);

		// publish data just once
		// TODO: working, but probably not the most elegant way
		try {
			Publisher<DataVis> pub = n.advertise("data_vis_msgs", new DataVis(), 100);

			pub.publish(data);

			n.spinOnce();
			pub.shutdown();
  		} catch (RosException e) {
 			e.printStackTrace();
		}

	}

	/**
	 * Remove all objects from the visualization
	 */
	public void clear() {
		/*synchronized (diagrams) {
			diagrams.clear();
		}*/
		for(String mrk : diagrams.keySet()) {
			removeDiagram(mrk);
		}
	}




	// // // // // // // // // // // // // // // // // // // // // // // // // // //
	//
	// Helper methods
	//

	/**
	 * Thread-safe ROS initialization
	 */
	protected static void initRos() {

		ros = Ros.getInstance();

		if(!Ros.getInstance().isInitialized()) {
			ros.init("knowrob_data_vis");
		}
		n = ros.createNodeHandle();

	}


	/**
	 * Thread that publishes the current state of the marker set
	 * to the visualization_marker topic.
	 *
	 * @author tenorth@cs.uni-bremen.de
	 *
	 */
	public class PublisherThread implements Runnable {

		@Override
		public void run() {

			try {
				Publisher<DataVis> pub = n.advertise("data_vis_msgs", new DataVis(), 100);

				while(n.isValid()) {

					synchronized (diagrams) {
						for(String mrk : diagrams.keySet())
							pub.publish(diagrams.get(mrk));
					}
					n.spinOnce();
					Thread.sleep(2000);
				}

				pub.shutdown();

			} catch (RosException e) {
				e.printStackTrace();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}


	public static void main(String args[]) {

//		MarkerVisualization vis = new MarkerVisualization();
//		vis.addObjectWithChildren("http://ias.cs.tum.edu/kb/ias_semantic_map.owl#SemanticEnvironmentMap0", "http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_1377766542");
//		vis.highlight("http://ias.cs.tum.edu/kb/knowrob.owl#Refrigerator67", true, 150, 0, 0, 180);

//		PrologInterface.executeQuery("mng_robot_pose_at_time(pr2:'PR2Robot1', '/map', knowrob:timepoint_1392799360, Pose)");
//		PrologInterface.executeQuery("add_object_with_children(pr2:'PR2Robot1', knowrob:timepoint_1392799360)");

//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_1377766542");


//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_1377766542");

	}
}
