package org.knowrob.vis;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import data_vis_msgs.DataVis;
import data_vis_msgs.ValueList;


/**
 * Visualization module for the KnowRob knowledge base
 *
 * The class publishes diagram messages to be visualized by a
 * Web-based diagram widget
 *
 * @author tenorth@cs.uni-bremen.de
 *
 */
public class DiagramVisualization extends AbstractNodeMain {


	Publisher<DataVis> pub;
	ConnectedNode node;

	/**
	 * Store the messages to be published
	 */
	protected Map<String, DataVis> diagrams;


	/**
	 * Constructor. Starts the marker publisher in a parallel thread.
	 */
	public DiagramVisualization() {
		diagrams =  new ConcurrentHashMap<String, DataVis>(8, 0.9f, 1);
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		node = connectedNode;
		pub = connectedNode.newPublisher("data_vis_msgs", data_vis_msgs.DataVis._TYPE);
	}

	
	
	/**
	 * Add diagram to the visualization.
	 *
	 */
	public void addDiagram(String id, String title, String type, String xlabel, String ylabel, int width, int height, String fontsize, String[][][] values) {

		DataVis data = node.getTopicMessageFactory().newFromType(data_vis_msgs.DataVis._TYPE);

		data.setId(id);
		data.setTitle(title);
		data.setXlabel(xlabel);
		data.setYlabel(ylabel);
		data.setWidth(width);
		data.setHeight(height);
		data.setFontsize(fontsize);

		if(type.equals("piechart")) {
			data.setType(DataVis.TYPE_PIECHART);
		} else if(type.equals("barchart")) {
			data.setType(DataVis.TYPE_BARCHART);
		} else if(type.equals("treechart")) {
			data.setType(DataVis.TYPE_TREECHART);
		}

		for(String[][] val_list : values) {
			ValueList v = node.getTopicMessageFactory().newFromType(data_vis_msgs.ValueList._TYPE);
			v.setValue1(new ArrayList<String>(Arrays.asList(val_list[0])));
			v.setValue2(new ArrayList<String>(Arrays.asList(val_list[1])));
			data.getValues().add(v);
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

		// send empty data to id to remove it
		DataVis data =  node.getTopicMessageFactory().newFromType(data_vis_msgs.DataVis._TYPE);
		data.setId(id);

		
		// publish empty message
		synchronized (diagrams) {
			diagrams.put(id, data);
		}
		publishDiagrams();
		
		// remove the object from the list		
		synchronized (diagrams) {
			diagrams.remove(id); 
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

	public void publishDiagrams() {

		synchronized (diagrams) {

			synchronized (diagrams) {
				for(String mrk : diagrams.keySet())
					pub.publish(diagrams.get(mrk));
			}
		}
	}



	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_data_vis");
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
