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

	public boolean cap_move_base() {

		/*
		 * Create String-Arrays
		 */
		String[] dependencies = { "/move_base/goal", "/move_base/cancel",
				"/move_base/feedback", "/move_base/status",
				"/move_base/result", "/move_base_simple/goal", "/cmd_vel" };
		String[] published = { "/move_base/feedback", "/move_base/status",
				"/move_base/result", "/cmd_vel" };
		String[] subscribed = { "/move_base/goal", "/move_base/cancel",
				"/move_base_simple/goal" };

		return (this.checkDependencies(dependencies, this
				.executeCommand("rostopic list"))
				&& this.checkPublishedTopics(published) && this
				.checkSubscribedTopics(subscribed));
	}

	public boolean cap_move_arm() {
		// String[] dependencies = { "move_arm/goal", "move_arm/cancel",
		// "move_arm/feedback", "move_arm/status", "move_arm/result" };
		// String[] published = { "move_arm/feedback", "move_arm/status",
		// "move_arm/result" };
		// String[] subscribed = { "move_arm/goal", "move_arm/cancel" };

		return (this.cap_move_left_arm() && this.cap_move_right_arm());
	}

	public boolean cap_move_left_arm() {
		String[] dependencies = { "/move_left_arm/goal",
				"/move_left_arm/cancel", "/move_left_arm/feedback",
				"/move_left_arm/status", "/move_left_arm/result" };
		String[] published = { "/move_left_arm/feedback",
				"/move_left_arm/status", "/move_left_arm/result" };
		String[] subscribed = { "/move_left_arm/goal", "/move_left_arm/cancel" };

		return (this.checkDependencies(dependencies, this
				.executeCommand("rostopic list"))
				&& this.checkPublishedTopics(published) && this
				.checkSubscribedTopics(subscribed));
	}

	public boolean cap_move_right_arm() {
		String[] dependencies = { "/move_right_arm/goal",
				"/move_right_arm/cancel", "/move_right_arm/feedback",
				"/move_right_arm/status", "/move_right_arm/result" };
		String[] published = { "/move_right_arm/feedback",
				"/move_right_arm/status", "/move_right_arm/result" };
		String[] subscribed = { "/move_right_arm/goal",
				"/move_right_arm/cancel" };

		return (this.checkDependencies(dependencies, this
				.executeCommand("rostopic list"))
				&& this.checkPublishedTopics(published) && this
				.checkSubscribedTopics(subscribed));
	}

	public boolean cap_grasp_execution(){
		return true;
	}
	
	private boolean checkPublishedTopics(String[] published) {
		ArrayList<String> topic_info = null;
		for (String p : published) {
			try {
				topic_info = this.executeCommand("rostopic info " + p);
			} catch (Exception e) {
			}

			if (topic_info.contains("Publishers: None"))
				return false;

		}
		return true;
	}

	private boolean checkSubscribedTopics(String[] subscribed) {
		ArrayList<String> topic_info = null;
		for (String s : subscribed) {
			try {
				topic_info = this.executeCommand("rostopic info " + s);
			} catch (Exception e) {
			}

			if (topic_info.contains("Subscribers: None"))
				return false;

		}
		return true;
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

	private boolean existLineInFile(String dependence, ArrayList<String> file) {
		for (String s : file) {
			if (s.equalsIgnoreCase(dependence))
				return true;
		}
		return false;
	}

	private boolean checkDependencies(String[] dependencies,
			ArrayList<String> topics) {
		for (String d : dependencies) {
			if (!this.existLineInFile(d, topics))
				return false;
		}
		return true;
	}

	@Deprecated
	private ArrayList<String> FileReader(String path) throws Exception {
		String line;
		ArrayList<String> listLines = new ArrayList<String>();

		try {
			File file = new File(path);
			BufferedReader in = new BufferedReader(new FileReader(file));

			if (!file.exists()) {
				throw new ExistsException("File doesn't exist.");
			}
			if (!file.isFile()) {
				throw new IsFileException("Is not a File");
			}
			if (!file.canRead()) {
				throw new ReadException("Cannot read");
			}

			while ((line = in.readLine()) != null) {
				listLines.add(line);
			}

			in.close();

			return listLines;
		} catch (ExistsException e) {
			System.out.println("ExistException");
		} catch (IsFileException e) {
			System.out.println("IsFileException");
		} catch (ReadException e) {
			System.out.println("ReadException");
		}
		return null;
	}

}
