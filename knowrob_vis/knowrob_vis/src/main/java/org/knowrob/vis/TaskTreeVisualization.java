/*
 * Copyright (c) 2014-15 Asil Kaan Bozcuoglu
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

import java.util.List;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.ArrayList;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import data_vis_msgs.TaskTree;
import data_vis_msgs.Task;


/**
 * Task Tree Visualization module for Robot Memory Replay 
 *
 * The class publishes task trees to be visualized by a
 * Web-based diagram widget
 *
 * @author asil@cs.uni-bremen.de
 *
 */
public class TaskTreeVisualization extends AbstractNodeMain {


	Publisher<TaskTree> pub;
	ConnectedNode node;

	/**
	 * Store the tree to be published
	 */
	protected TaskTree data;


	/**
	 * Constructor.
	 */
	public TaskTreeVisualization() {
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {

		node = connectedNode;
		pub = connectedNode.newPublisher("/task_tree_msgs", data_vis_msgs.TaskTree._TYPE);
	}


	
	/**
	 * Add/update the tree into/in the visualization. (This method uses a greedy approach which
	 * assumes that the root is always at the end of 'task' array and '
	 * task[x] hight >= tasks[y]'s  hight such that x > y
	 *
	 * 'tasks' is a 2D Tree array such that:
	 *  tasks[x][0] contains task goal 
         *  tasks[x][1] contains task ID
	 *  tasks[x][2] contains parent task's ID
	 *  tasks[x][3] contains task type  
	 */
	public void addTaskTree(String[][] tasks, String[] highlightedTasks, String[] allowedTypes) {

		data = node.getTopicMessageFactory().newFromType(data_vis_msgs.TaskTree._TYPE);

		data.setWidth(960);
		data.setHeight(500);
		ArrayList<Task> tree = new ArrayList<Task>();
		


		for(int i = tasks.length - 1; i >= 0; i--) 
                {
			Task t = node.getTopicMessageFactory().newFromType(data_vis_msgs.Task._TYPE);
			t.setInfo( tasks[i][0]);
			t.setColor("#000000");
			t.setId(i);
                        t.setParent(i);
			t.setType(tasks[i][3]);

			// checking if the current task should be visualized in the tree
			boolean isVisualizationDesired = false;
			for(int j = 0; j < allowedTypes.length; j++)
			{
				if(allowedTypes[j].equals(tasks[i][3]))
				{
					isVisualizationDesired = true;
					break;
				}
			}


			// if it should be visualized, set parent and color of the node, then, add to the tree.
			if(isVisualizationDesired)
			{
				
				for(int j = tasks.length - 1; j > i; j--)
				{
					if(tasks[j][1].equals(tasks[i][2]))
					{
						t.setParent(j);
						break;
					}
				}

			

				for(int j = 0; j < highlightedTasks.length; j++)
				{
					if(highlightedTasks[j].equals(tasks[i][1]))
					{
						t.setColor("#FF0000");
						break;
					}
				}
				tree.add( t);
			
			}
			else
			{
				for(int j = i - 1; j >= 0; j--) 
                		{
					if(tasks[i][1].equals(tasks[j][2]))
					{
						tasks[j][2] = tasks[i][2]; 
					}
				}
			}
			
			
		

			
		}
		data.setTree(tree);

		publishTree();
	}

	public void removeTaskTree() 
	{
		data = null;
	}

	public void publishTree() 
	{
		pub.publish(data);
	}



	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_task_tree");
	}

	public static void main(String args[]) {

		
	}
}
