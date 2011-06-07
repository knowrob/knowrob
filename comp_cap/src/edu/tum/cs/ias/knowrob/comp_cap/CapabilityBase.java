package edu.tum.cs.ias.knowrob.comp_cap;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collection;

import ros.NodeHandle;
import ros.Ros;
import rosjava.Master;
import rosjava.Service;
import rosjava.Topic;

public class CapabilityBase {
	Ros ros;
	NodeHandle node;
	Master rosmaster;

	public CapabilityBase() {
		ros = Ros.getInstance();
		if(!ros.isInitialized()){
			ros.init("comp_cap");
		}
		node = ros.createNodeHandle("comp_cap");
		rosmaster = new Master(node.getMasterHost(), node.getMasterPort());
	}

	private String[] convertOwlNameToTopicName(String[] input) {
		String[] output = new String[input.length];
		int i = 0;
		for (String help : input) {
			help = help.substring(help.indexOf("#") + 1);
			help = help.replace("_._", "/");
			output[i] = help;
			i++;
		}
		return output;
	}

	public String[] comp_cap_all(String[] published, String[] subscribed,
			String[] services) {
		ArrayList<String> result = new ArrayList<String>();
		String[] pubRes = this.testTopics(published, false, true);
		for (String s : pubRes) {
			result.add(s);
		}

		String[] subRes = this.testTopics(subscribed, true, false);
		for (String s : subRes) {
			result.add(s);
		}

		String[] srvsRes = testServices(services);
		for (String s : srvsRes) {
			result.add(s);
		}

		return result.toArray(new String[result.size()]);
	}

	public String[] comp_cap_pub_sub(String[] published, String[] subscribed) {
		ArrayList<String> result = new ArrayList<String>();
		String[] pubRes = this.testTopics(published, false, true);
		for (String s : pubRes) {
			result.add(s);
		}

		String[] subRes = this.testTopics(subscribed, true, false);
		for (String s : subRes) {
			result.add(s);
		}

		return result.toArray(new String[result.size()]);
	}

	public String[] comp_cap_sub_srvs(String[] subscribed, String[] services) {
		ArrayList<String> result = new ArrayList<String>();
		String[] subRes = this.testTopics(subscribed, true, false);
		for (String s : subRes) {
			result.add(s);
		}

		String[] srvsRes = testServices(services);
		for (String s : srvsRes) {
			result.add(s);
		}

		return result.toArray(new String[result.size()]);
	}

	public String[] comp_cap_srvs(String[] services) {
		return this.testServices(services);
	}

	public String[] comp_cap_pub(String[] published) {
		return this.testTopics(published, false, true);
	}

	public String[] testServices(String[] srvs) {
		ArrayList<String> found = new ArrayList<String>();
		Collection<Service> service = rosmaster.getServices();

		srvs = this.convertOwlNameToTopicName(srvs);

		for (int j = 0; j < srvs.length; j++) {
			String s = srvs[j];
			for (Service t : service) {
				if (t.getDatatype().equals(s)) {
					found.add(t.getDatatype());
				}
			}
		}

		return found.toArray(new String[found.size()]);
	}

	public String[] testTopics(String[] topicTypes, boolean subscribed,
			boolean published) {
		ArrayList<String> found = new ArrayList<String>();
		Collection<Topic> topics = null;

		topicTypes = convertOwlNameToTopicName(topicTypes);
		if (subscribed) {
			topics = rosmaster.getSubscribedTopics();
		}
		if (published) {
			topics = rosmaster.getPublishedTopics();
		}

		for (int j = 0; j < topicTypes.length; j++) {
			String s = topicTypes[j];
			for (Topic t : topics) {
				if (t.getDatatype().equals(s)) {
					found.add(t.getDatatype());
					break;
				}
			}
		}

		return found.toArray(new String[found.size()]);
	}

	public void roslaunch(String[] rospackage, String[] launchFile) {
		String[] commands = new String[launchFile.length];
		int i = 0;
		for (String s : launchFile) {
			commands[i] = launch(rospackage[0], s);
			i++;
		}
	}

	public String launch(String rospackage, String launchFile) {
		// ArrayList<String> output = new ArrayList<String>();
		// String s = null;
		String command = "roslaunch "
				+ rospackage.substring(rospackage.indexOf("#") + 1) + " "
				+ launchFile.substring(launchFile.indexOf("#") + 1) + " &";

		try {
			String[] command1 = { "bash", "-c", command };
			@SuppressWarnings("unused")
			Process p = Runtime.getRuntime().exec(command1);

			 BufferedReader stdInput = new BufferedReader(new
			 InputStreamReader(
			 p.getInputStream()));
			 
			 String line = stdInput.readLine();
			 
			 System.out.println(line);
			 
			// BufferedReader stdError = new BufferedReader(new
			// InputStreamReader(
			// p.getErrorStream()));
			//
			// // read the output from the command and write it to output
			// while ((s = stdInput.readLine()) != null) {
			// output.add(s);
			// }
			//
			// // read any errors from the attempted command
			// while ((s = stdError.readLine()) != null) {
			// System.out.println(s);
			// }
			//
			return command;
		} catch (IOException e) {
			System.out.println("exception happened - here's what I know: ");
			e.printStackTrace();
			return "Error";
		}
	}
}
