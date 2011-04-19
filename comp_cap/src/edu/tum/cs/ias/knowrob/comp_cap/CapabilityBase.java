package edu.tum.cs.ias.knowrob.comp_cap;

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

		published = this.convertOwlNameToTopicName(published);
		subscribed = this.convertOwlNameToTopicName(subscribed);

		help = this.checkDependencies(published, ros.getMsgTypesOfPublishedTopics());
		for (String s : help) {
			if (!this.existLineInFile(s, missing)) {
				missing.add(s);
			}
		}

		help = this.checkDependencies(subscribed, ros.getMsgTypesOfSubscribedTopics());
		for (String s : help) {
			if (!this.existLineInFile(s, missing)) {
				missing.add(s);
			}
		}

		ros.destroy();

		String[] res = new String[missing.size()];
		res = missing.toArray(res);
		res = this.convertTopicNameToOwlName(res);
		return res;
	}

	private String[] convertOwlNameToTopicName(String[] input) {
		String[] output = new String[input.length];
		int i = 0;
		for (String help : input) {
			help = help.replace("http://ias.cs.tum.edu/kb/comp_cap.owl#", "");
			help = help.replace("_._", "/");
			output[i] = help;
			i++;
		}
		return output;
	}
	
	private String[] convertTopicNameToOwlName(String[] input) {
		String[] output = new String[input.length];
		int i = 0;
		for (String help : input) {
			help = help.replace("/", "_._");
			help = "http://ias.cs.tum.edu/kb/comp_cap.owl#" + help;
			output[i] = help;
			i++;
		}
		return output;
	}

	private ArrayList<String> checkDependencies(String[] dependencies,
			ArrayList<String> topics) {
		ArrayList<String> missing = new ArrayList<String>();
		for (String s : dependencies) {
			if (!this.existLineInFile(s, topics)){
				missing.add(s);
			}				
		}
		return missing;
	}

	private boolean existLineInFile(String dependence, ArrayList<String> file) {
		String s;
		for (int i = 0; i < file.size(); i++) {
			s = file.get(i);
			if (s.contains(dependence) || s.equalsIgnoreCase(dependence)){
				file.remove(i);
				return true;
			}			
		}
		return false;
	}


}
