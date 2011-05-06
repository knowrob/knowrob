package edu.tum.cs.ias.knowrob.comp_cap;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;

import ros.Topic;

public class CapabilityBase {
	CapROSClient ros;
	ArrayList<String> usedNamespace;

	public CapabilityBase() {
		ros = new CapROSClient("CapRosClient");
		usedNamespace = new ArrayList<String>();
	}

	public CapabilityBase(String nodeName) {
		ros = new CapROSClient(nodeName);
		usedNamespace = new ArrayList<String>();
	}

	public void resetUsedNamespace() {
		usedNamespace = new ArrayList<String>();
	}

	public void setNamespaceUsed(String namespace) {
		if (usedNamespace.contains(namespace)) {
			usedNamespace.add(namespace);
		}
	}

	public boolean isNamespaceUsed(String namespace) {
		for (String s : usedNamespace) {
			if (s.contains(namespace)) {
				return true;
			}
		}
		return false;
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
		String[] pubRes = this.testTopics(subscribed, false, true);
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
		String[] pubRes = this.testTopics(subscribed, false, true);
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
		Collection<Service> service = ros.getService();

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
			topics = ros.getSubscribedTopics();
		}
		if (published) {
			topics = ros.getPublishedTopics();
		}

		for (int j = 0; j < topicTypes.length; j++) {
			String s = topicTypes[j];
			for (Topic t : topics) {
				if (t.getDatatype().equals(s)) {
					found.add(t.getDatatype());
				}
			}
		}

		return found.toArray(new String[found.size()]);
	}

	public void roslaunch(String[] rospackage, String[] launchFile) {
		// ArrayList<String> output = new ArrayList<String>();
		// String s = null;
		String command = "roslaunch "
				+ rospackage[0].substring(rospackage[0].indexOf("#") + 1) + " "
				+ launchFile[0].substring(launchFile[0].indexOf("#") + 1)
				+ " &";

		try {
			String[] command1 = { "bash", "-c", command };
			@SuppressWarnings("unused")
			Process p = Runtime.getRuntime().exec(command1);

			// BufferedReader stdInput = new BufferedReader(new
			// InputStreamReader(
			// p.getInputStream()));
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
			// return output;
		} catch (IOException e) {
			System.out.println("exception happened - here's what I know: ");
			e.printStackTrace();
			// return null;
		}
	}
}
