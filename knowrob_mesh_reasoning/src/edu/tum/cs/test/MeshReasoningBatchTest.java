/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.test;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;

import javax.swing.WindowConstants;

import edu.tum.cs.vis.model.MeshReasoning;
import edu.tum.cs.vis.model.parser.ModelParser;

/**
 * Class for simple batch loading of CAD models and showing mesh reasoning view. Load all models of
 * a specified folder.
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshReasoningBatchTest {

	/**
	 * Currently shown mesh reasoning views to show only maximum 3 windows per time
	 */
	public static int		currentWindowCount	= 0;

	/**
	 * Monitor for multi threaded window opening
	 */
	public static Object	monitor				= new Object();

	static String			subfolder			= "vessel";

	/**
	 * Main entry point
	 * 
	 * @param args
	 *            not used
	 */
	public static void main(String[] args) {
		String folder = "models/batch/" + subfolder + "/";
		File dir = new File(folder);
		recursiveTraversal(dir);

		while (currentWindowCount > 0) {
			synchronized (monitor) {
				try {
					monitor.wait();
				} catch (InterruptedException e1) {
					e1.printStackTrace();
				}
			}
		}
		System.exit(0);
	}

	/**
	 * Parse and show specified file
	 * 
	 * @param f
	 *            File object to load
	 * 
	 */
	public static void processFile(File f) {
		if (ModelParser.findParser(f.getAbsolutePath()) == null)
			return;

		final String path = f.getAbsolutePath();
		final String filename = path.substring(path.lastIndexOf(File.separatorChar) + 1);

		while (currentWindowCount > 1) {
			synchronized (monitor) {
				try {
					monitor.wait();
				} catch (InterruptedException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
			}

		}
		currentWindowCount++;

		final MeshReasoning mr = MeshReasoning.initMeshReasoning(true);
		mr.frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		mr.frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {

				/*File f = new File(path);
				String input = (String) JOptionPane.showInputDialog(mr.frame, "New File name",
						"Filename", JOptionPane.PLAIN_MESSAGE, null, null, filename);
				if (input == null) {

					f.delete();
				} else {
					File nf = new File("models/batch_ok/" + subfolder + "/");
					nf.mkdirs();
					nf = new File("models/batch_ok/" + subfolder + "/" + input);
					f.renameTo(nf);
				}*/
				currentWindowCount--;
				synchronized (monitor) {
					monitor.notifyAll();
				}
				mr.frame.dispose();

			}
		});

		mr.setFrameTitle(filename);
		mr.setDefaultImageFilename(filename);
		mr.analyseByPath(path);

	}

	/**
	 * Traverse recursively the given directory or process file if <tt>fileObject<tt> is a file.
	 * 
	 * @param fileObject
	 *            Directory or file object
	 * 
	 * 
	 */
	public static void recursiveTraversal(File fileObject) {
		if (fileObject.isDirectory()) {
			File allFiles[] = fileObject.listFiles();
			for (File aFile : allFiles) {
				recursiveTraversal(aFile);
			}
		} else if (fileObject.isFile()) {
			processFile(fileObject);
		}
	}

}
