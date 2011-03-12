package edu.tum.cs.ias.knowrob.comp_cap;

import java.io.*;

public class CapabilityBase {

	String pathFile = "/home/neumanns/topics.txt";

	public CapabilityBase() {
	}

	public CapabilityBase(String path) {
		pathFile = path;
	}

	private String[] readFile(String path) throws Exception {
		int length = 0;
		String line;
		String[] listLines;

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

			while (in.readLine() != null) {
				length++;
			}

			BufferedReader in2 = new BufferedReader(new FileReader(file));
			listLines = new String[length];

			for (int i = 0; i < length; i++) {
				line = in2.readLine();
				listLines[i] = line;
			}

			in.close();
			in2.close();

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

	private boolean check(String[] d, String topicFile) {
		String[] topics = null;
		String[] dependencies = d;
		boolean[] exists = new boolean[dependencies.length];
		boolean result = false;

		try {
			topics = this.readFile(topicFile);
		} catch (Exception e) {
			System.out.println("Wrong Filepath");
		}

		if (topics != null) {
			for (int i = 0; i < dependencies.length; i++) {
				for (int j = 0; j < topics.length; j++) {
					if (topics[j] == dependencies[i]) {
						exists[i] = true;
						break;
					}
				}
			}
		}

		for (int i = 0; i < exists.length; i++) {
			if (!exists[i]) {
				result = false;
				break;
			}
			result = true;
		}

		return result;
	}

	private int executeCommand(String[] command) {
		@SuppressWarnings("unused")
		Process proc;
		try {
			proc = java.lang.Runtime.getRuntime().exec(command);
		} catch (IOException e) {
			System.out.println("Wrong Shell-Command");
			return -1;
		}

		return 1;
	}

	public boolean cap_move_base(String debug) {
		if (debug == "-1") {
			return false;
		}
		if (debug == "1") {
			return true;
		}

		String[] depency = { "move_base/goal", "move_base/cancel",
				"move_base/feedback", "move_base/status", "move_base/result",
				"move_base_simple/goal", "cmd_vel" };
		String[] published = { "move_base/feedback", "move_base/status",
				"move_base/result", "cmd_vel" };
		String[] subscriped = { "move_base/goal", "move_base/cancel",
				"move_base_simple/goal" };

		this.executeCommand(new String[] { "bash", "-c",
				"rostopic list >>  " + pathFile });

		return check(depency, pathFile);
	}

	public boolean cap_move_arm(String debug) {
		if (debug == "-1") {
			return false;
		}
		if (debug == "1") {
			return true;
		}

		String[] depency = {};
		String[] published = {};
		String[] subscriped = {};

		this.executeCommand(new String[] { "bash", "-c",
				"rostopic list >>  " + pathFile });

		return check(depency, pathFile);
	}

}
