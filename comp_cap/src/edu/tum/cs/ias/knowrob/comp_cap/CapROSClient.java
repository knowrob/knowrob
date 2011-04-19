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
	public Collection<Topic> getPublishedTopics() {
		return node.getTopics();// seems to have the same function as 'rostopic
		// list -p
	}

	public void destroy() {
		node.shutdown();
	}

	public ArrayList<String> getMsgTypesOfPublishedTopics() {
		Collection<Topic> publishedTopics = this.getPublishedTopics();
		ArrayList<String> msgTypes = new ArrayList<String>();

		for (Topic p : publishedTopics) {
			msgTypes.add(p.getDatatype());
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
					break;
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
