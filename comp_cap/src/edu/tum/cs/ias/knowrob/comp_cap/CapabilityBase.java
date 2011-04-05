package edu.tum.cs.ias.knowrob.comp_cap;

import java.io.*;
import java.util.ArrayList;

public class CapabilityBase {
	CapROSClient ros;

	public CapabilityBase() {
		// ros = new CapROSClient("CapRosClient");
	}

	public CapabilityBase(String nodeName) {
		// ros = new CapROSClient(nodeName);
	}

	public boolean cap_move_arm() {
		// String[] published = { "move_arm/feedback", "move_arm/status",
		// "move_arm/result" };
		// String[] subscribed = { "move_arm/goal", "move_arm/cancel" };

		return (this.cap_move_left_arm() && this.cap_move_right_arm());
	}
	
	public boolean cap_move_base() {
		String[] published = { "/move_base/feedback", "/move_base/status",
				"/move_base/result", "/cmd_vel" };
		String[] subscribed = { "/move_base/goal", "/move_base/cancel",
				"/move_base_simple/goal" };

		if(this.comp_capability(published, subscribed).length == 0)
			return true;
		return false;
	}

	public boolean cap_move_left_arm() {
		String[] published = { "/move_left_arm/feedback",
				"/move_left_arm/status", "/move_left_arm/result" };
		String[] subscribed = { "/move_left_arm/goal", "/move_left_arm/cancel" };

		if(this.comp_capability(published, subscribed).length == 0)
			return true;
		return false;
	}

	public boolean cap_move_right_arm() {
		String[] published = { "/move_right_arm/feedback",
				"/move_right_arm/status", "/move_right_arm/result" };
		String[] subscribed = { "/move_right_arm/goal",
				"/move_right_arm/cancel" };

		if(this.comp_capability(published, subscribed).length == 0)
			return true;
		return false;
	}

	public boolean cap_grasp_execution() {
		// müssen noch rausgefunden werden
		String[] published = { };
		String[] subscribed = { };
		
		if(this.comp_capability(published, subscribed).length == 0)
			return true;
		return false;
	}

	
	/*
	 * generic function to check if a capability is given or not
	 * return a Array containing the missing Topics
	 */
	public String[] comp_capability(String[] published, String[] subscribed){
		ArrayList<String> missing = new ArrayList<String>();
		ArrayList<String> help;
		
		String[] dependencies = new String[published.length + subscribed.length];
		int index_pub = 0;
		int index_sub = 0;
		int index_dep = 0;		

		
		while(index_pub < published.length){
			dependencies[index_dep] = published[index_pub];
			index_dep++;
			index_pub++;
		}
		while(index_sub < subscribed.length){
			dependencies[index_dep] = subscribed[index_sub];
			index_dep++;
			index_sub++;
		}

		help = this.checkDependencies(dependencies, this.executeCommand("rostopic list"));
		for(String s : help){
			if(!this.existLineInFile(s, missing)){
				missing.add(s);
			}
		}
		
		help = this.checkPublishedTopics(published);
		for(String s : help){
			if(!this.existLineInFile(s, missing)){
				missing.add(s);
			}
		}
		
		help = this.checkSubscribedTopics(subscribed);
		for(String s : help){
			if(!this.existLineInFile(s, missing)){
				missing.add(s);
			}
		}
		
		String[] res = new String[missing.size()];
		return missing.toArray(res);
	}
	
	private ArrayList<String> checkDependencies(String[] dependencies,
			ArrayList<String> topics) {
		ArrayList<String> missing = new ArrayList<String>();
		for (String d : dependencies) {
			if (!this.existLineInFile(d, topics))
				missing.add(d);
		}
		return missing;
	}
	
	private ArrayList<String> checkPublishedTopics(String[] published) {
		ArrayList<String> missing = new ArrayList<String>();
		ArrayList<String> topic_info = null;
		for (String p : published) {
			try {
				topic_info = this.executeCommand("rostopic info " + p);
			} catch (Exception e) {
			}

			if (topic_info.contains("Publishers: None"))
				missing.add(p);
		}
		return missing;
	}

	private ArrayList<String> checkSubscribedTopics(String[] subscribed) {
		ArrayList<String> missing = new ArrayList<String>();
		ArrayList<String> topic_info = null;
		for (String s : subscribed) {
			try {
				topic_info = this.executeCommand("rostopic info " + s);
			} catch (Exception e) {
			}

			if (topic_info.contains("Subscribers: None"))
				missing.add(s);
		}
		return missing;
	}
	
	private boolean existLineInFile(String dependence, ArrayList<String> file) {
		for (String s : file) {
			if (s.equalsIgnoreCase(dependence))
				return true;
		}
		return false;
	}
	
	private ArrayList<String> executeCommand(String command) {
		ArrayList<String> output = new ArrayList<String>();
		String s = null;

		try {
			String[] command1 = { "bash", "-c", command };
			Process p = Runtime.getRuntime().exec(command1);

			BufferedReader stdInput = new BufferedReader(new InputStreamReader(
					p.getInputStream()));
			BufferedReader stdError = new BufferedReader(new InputStreamReader(
					p.getErrorStream()));

			// read the output from the command and write it to output
			while ((s = stdInput.readLine()) != null) {
				output.add(s);
			}

			// read any errors from the attempted command
			while ((s = stdError.readLine()) != null) {
			}

			return output;
		} catch (IOException e) {
			System.out.println("exception happened - here's what I know: ");
			e.printStackTrace();
			return null;
		}
	}
}
