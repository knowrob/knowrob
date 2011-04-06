package edu.tum.cs.ias.knowrob.comp_cap;

import java.io.*;
import java.util.ArrayList;

public class CapabilityBase {
	CapROSClient ros;

	public CapabilityBase() {
		ros = new CapROSClient("CapRosClient");
	}

	public CapabilityBase(String nodeName) {
		ros = new CapROSClient(nodeName);
	}

	/*
	 * generic function to check if a capability is given or not return a Array
	 * containing the missing Topics
	 */
	public String[] comp_capability(String[] published, String[] subscribed) {
		ArrayList<String> missing = new ArrayList<String>();
		ArrayList<String> help;

		published = this.convertTopicNames(published);
		subscribed = this.convertTopicNames(subscribed);

		String[] dependencies = new String[published.length + subscribed.length];
		int index_pub = 0;
		int index_sub = 0;
		int index_dep = 0;

		while (index_pub < published.length) {
			dependencies[index_dep] = published[index_pub];
			index_dep++;
			index_pub++;
		}
		while (index_sub < subscribed.length) {
			dependencies[index_dep] = subscribed[index_sub];
			index_dep++;
			index_sub++;
		}

		help = this.checkDependencies(dependencies, this
				.executeCommand("rostopic list"));
		for (String s : help) {
			if (!this.existLineInFile(s, missing)) {
				missing.add(s);
			}
		}

		help = this.checkDependencies(published, ros.getPublishedTopics());
		for (String s : help) {
			if (!this.existLineInFile(s, missing)) {
				missing.add(s);
			}
		}

		help = this.checkDependencies(subscribed, this
				.executeCommand("rostopic list -s"));
		for (String s : help) {
			if (!this.existLineInFile(s, missing)) {
				missing.add(s);
			}
		}

		ros.destroy();
		String[] res = new String[missing.size()];
		res = missing.toArray(res);
		return res;
	}

	private String[] convertTopicNames(String[] input) {
		String[] output = new String[input.length];
		int i = 0;
		for (String help : input) {
			help = help.replace("http://ias.cs.tum.edu/kb/comp_cap.owl#", "/");
			help = help.replace("_._", "/");
			output[i] = help;
			i++;
		}
		return output;
	}

	private ArrayList<String> checkDependencies(String[] dependencies,
			ArrayList<String> topics) {
		ArrayList<String> missing = new ArrayList<String>();
		for (String s : dependencies) {
			if (!this.existLineInFile(s, topics))
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
