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
