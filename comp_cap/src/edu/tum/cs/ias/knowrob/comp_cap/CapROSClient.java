package edu.tum.cs.ias.knowrob.comp_cap;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collection;

import ros.*;

public class CapROSClient {
	boolean isInitialized = false;

	Ros ros;
	NodeHandle node;

	public CapROSClient(String node_name) {
		ros = Ros.getInstance();

		if (!Ros.getInstance().isInitialized()) {
			ros.init(node_name);
		}
		node = ros.createNodeHandle();
	}

	/*
	 * returns published topics
	 */
	public ArrayList<String> getPublishedTopics() {
		ArrayList<String> topics = new ArrayList<String>();
		Collection<Topic> help;

		help = node.getTopics();// seems to have the same function as 'rostopic
								// list -p'
		for (Topic t : help) {
			topics.add(t.getName());
		}

		return topics;
	}

	public void destroy() {
		node.shutdown();
	}

	public ArrayList<String> getMsgTypesOfPublishedTopics() {
		ArrayList<String> publishedTopics = this.getPublishedTopics();
		ArrayList<String> msgTypes = new ArrayList<String>();

		ArrayList<String> help;
		for (String p : publishedTopics) {
			help = this.executeCommand("rostopic info " + p); // bis ich rausgefunden habe wie ich das mit rosjava hinbekomme
			for(String s : help){
				if(s.contains("Type:")){
					msgTypes.add(s.replace("Type:", ""));
				}
			}
		}

		return msgTypes;
	}

	public ArrayList<String> getMsgTypesOfSubscribedTopics(){
		ArrayList<String> subscribedTopics = this.executeCommand("rostopic list -s");
		ArrayList<String> msgTypes = new ArrayList<String>();
		
		ArrayList<String> help;
		for (String p : subscribedTopics) {
			help = this.executeCommand("rostopic info " + p);
			for(String s : help){
				if(s.contains("Type:")){
					msgTypes.add(s.replace("Type:", ""));
				}
			}
		}		
		
		return msgTypes;
	}

	public ArrayList<String> executeCommand(String command) {
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
				System.out.println(s);
			}

			return output;
		} catch (IOException e) {
			System.out.println("exception happened - here's what I know: ");
			e.printStackTrace();
			return null;
		}
	}

}
