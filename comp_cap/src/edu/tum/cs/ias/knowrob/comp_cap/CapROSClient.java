package edu.tum.cs.ias.knowrob.comp_cap;

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


	public ArrayList<String> getTopics() {
		ArrayList<String> topics = new ArrayList<String>();
		Collection<Topic> help;

		help = node.getTopics();

		for (Topic t : help) {
			topics.add(t.getName());
			System.out.println(t.getName());
		}
		return topics;
	}

}
