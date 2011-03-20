package edu.tum.cs.ias.knowrob.comp_cap;

import java.io.*;
import java.util.ArrayList;

public class CapabilityBase {
	CapROSClient ros;

	String fileName;
	String directory;

	public CapabilityBase() {
		directory = "/tmp";
		fileName = "topics.txt";

		ros = new CapROSClient("CapRosClient");
	}

	public CapabilityBase(String directory, String filename, String nodeName) {
		this.fileName = filename;
		this.directory = directory;

		ros = new CapROSClient(nodeName);
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

		// /*
		// * check for subscriped Topics
		// */
		// this.publishedTopics = this.checkPublishedTopics(published);
		// if (!this.publishedTopics)
		// return false;
		//
		// /*
		// * check for published Topics
		// */
		// this.subscripedTopics = this.checkSubscribedTopics(subscribed);
		// if (!this.subscripedTopics)
		// return false;
		//
		// /*
		// * return result
		// */
		return (this.checkDependencies(dependencies, this.ros.getTopics()));
	}

	public boolean cap_move_arm() {
		// String[] dependencies = { "move_arm/goal", "move_arm/cancel",
		// "move_arm/feedback", "move_arm/status", "move_arm/result" };
		// String[] published = { "move_arm/feedback", "move_arm/status",
		// "move_arm/result" };
		// String[] subscribed = { "move_arm/goal", "move_arm/cancel" };

		return (this.cap_move_l_arm() && this.cap_move_r_arm());
	}

	public boolean cap_move_l_arm() {
		/*
		 * Create String-Arrays
		 */
		String[] dependencies = { "/move_left_arm/goal",
				"/move_left_arm/cancel", "/move_left_arm/feedback",
				"/move_left_arm/status", "/move_left_arm/result" };
		String[] published = { "/move_left_arm/feedback",
				"/move_left_arm/status", "/move_left_arm/result" };
		String[] subscribed = { "/move_left_arm/goal", "/move_left_arm/cancel" };

		// /*
		// * check for subscriped Topics
		// */
		// this.publishedTopics = this.checkPublishedTopics(published);
		// if (!this.publishedTopics)
		// return false;
		//
		// /*
		// * check for published Topics
		// */
		// this.subscripedTopics = this.checkSubscribedTopics(subscribed);
		// if (!this.subscripedTopics)
		// return false;
		//
		// /*
		// * return result
		// */

		// return (existTopics && publishedTopics && subscripedTopics);
		return (this.checkDependencies(dependencies, this.ros.getTopics()));
	}

	public boolean cap_move_r_arm() {
		/*
		 * Create String-Arrays
		 */
		String[] dependencies = { "/move_right_arm/goal",
				"/move_right_arm/cancel", "/move_right_arm/feedback",
				"/move_right_arm/status", "/move_right_arm/result" };
		String[] published = { "/move_right_arm/feedback",
				"/move_right_arm/status", "/move_right_arm/result" };
		String[] subscribed = { "/move_right_arm/goal",
				"/move_right_arm/cancel" };

		// /*
		// * check for subscriped Topics
		// */
		// this.publishedTopics = this.checkPublishedTopics(published);
		// if (!this.publishedTopics)
		// return false;
		//
		// /*
		// * check for published Topics
		// */
		// this.subscripedTopics = this.checkSubscribedTopics(subscribed);
		// if (!this.subscripedTopics)
		// return false;
		//
		// /*
		// * return result
		// */

		// return (existTopics && publishedTopics && subscripedTopics);
		return (this.checkDependencies(dependencies, this.ros.getTopics()));
	}

	private boolean checkPublishedTopics(String[] published) {
		ArrayList<String> topic_info = null;
		for (String p : published) {
			this.executeCommand("rostopic info " + p + " > " + this.directory
					+ "/published.txt");
			try {
				topic_info = this.FileReader(this.directory + "/published.txt");
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
			this.executeCommand("rostopic info " + s + " > " + this.directory
					+ "/subscribed.txt");
			try {
				topic_info = this
						.FileReader(this.directory + "/subscribed.txt");
			} catch (Exception e) {
			}

			if (topic_info.contains("Subscribers: None"))
				return false;

		}
		return true;
	}

	private void executeCommand(String command) {
		try {
			Runtime rt = Runtime.getRuntime();
			Process p = rt.exec(command);
			p.waitFor();
		} catch (Exception e) {
		}
	}

	private boolean existLineInFile(String dependence, ArrayList<String> file) {
		for(String s : file){
			if(s.equalsIgnoreCase(dependence))
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
