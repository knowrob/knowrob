/*
 * Copyright (c) 2013-15 Moritz Tenorth, Arne Stefes
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/
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
		pub = connectedNode.newPublisher("/data_vis_msgs", data_vis_msgs.DataVis._TYPE);
	}


	
	/**
	 * Add diagram to the visualization.
	 *
	 */
	public void addDiagram(String id, String title, String type, String xlabel, String ylabel, int width, int height, String fontsize, String[][][] values) {
		this.addDiagram(id, title, type, xlabel, ylabel, width, height, fontsize, null, values);
	}

	public void addDiagram(String id, String title, String type, String xlabel, String ylabel, int width, int height, String fontsize, String[] labels, String[][][] values) {

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
		} else if(type.equals("timeline")) {
			data.setType(DataVis.TYPE_TIMELINE);
		} else if(type.equals("linechart")) {
			data.setType(DataVis.TYPE_LINECHART);
		}

		if(labels!=null && labels.length != values.length) {
			System.err.println("ERROR: Number of labels and data rows differs.");
		}

		for(int i =0; i< values.length; i++) {
			
			String[][] val_list = values[i];

			ValueList v = node.getTopicMessageFactory().newFromType(data_vis_msgs.ValueList._TYPE);

			if(labels != null) {
				v.setLabel(labels[i]);
			}

			v.setValue1(new ArrayList<String>(Arrays.asList(val_list[0])));
			v.setValue2(new ArrayList<String>(Arrays.asList(val_list[1])));
			data.getValues().add(v);
		}

		// add marker to map
		if(data!=null) {
			synchronized (diagrams) {
				diagrams.put(id, data);
			}
			pub.publish(data);
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
		pub.publish(data);
		
		// remove the object from the list		
		synchronized (diagrams) {
			diagrams.remove(id); 
		}
	}

	/**
	 * Remove all objects from the visualization
	 */
	public void clear() {
		for(String mrk : diagrams.keySet()) {
			removeDiagram(mrk);
		}
	}




	// // // // // // // // // // // // // // // // // // // // // // // // // // //
	//
	// Helper methods
	//


	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_data_vis");
	}

	public static void main(String args[]) {

		//		MarkerVisualization vis = new MarkerVisualization();
		//		vis.addObjectWithChildren("http://knowrob.org/kb/ias_semantic_map.owl#SemanticEnvironmentMap0", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");
		//		vis.highlight("http://knowrob.org/kb/knowrob.owl#Refrigerator67", true, 150, 0, 0, 180);

		//		PrologInterface.executeQuery("mng_robot_pose_at_time(pr2:'PR2Robot1', '/map', knowrob:timepoint_1392799360, Pose)");
		//		PrologInterface.executeQuery("add_object_with_children(pr2:'PR2Robot1', knowrob:timepoint_1392799360)");

		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");


		//		vis.addObjectWithChildren("pr2:'PR2Robot1'", "http://knowrob.org/kb/knowrob.owl#timepoint_1377766542");

	}
}
