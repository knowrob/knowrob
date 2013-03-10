/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;

import javax.swing.WindowConstants;

import edu.tum.cs.vis.model.MeshReasoning;
import edu.tum.cs.vis.model.parser.ModelParser;

/**
 * @author Stefan Profanter
 * 
 */
public class ModelImageGenerator {

	public static final String	MODEL_DIR			= "/home/stefan/work/models";
	public static final boolean	ALSO_EXISTING_ONES	= false;
	public static Object		monitor				= new Object();

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		File dir = new File(MODEL_DIR);
		if (!dir.exists()) {
			System.err.println("Couldn't find directory: " + MODEL_DIR);
			return;
		}
		recursiveTraversal(dir);
	}

	/**
	 * Parse and show specified file
	 * 
	 * @param f
	 *            File object to load
	 * 
	 */
	public static void processFile(File f) {
		if (ModelParser.findParser(f.getAbsolutePath()) == null) {
			// System.out.println("Skipping file: " + f.getAbsolutePath() + " (no parser found)");
			return;
		}
		final String path = f.getAbsolutePath();
		final String filename = path.substring(path.lastIndexOf(File.separatorChar) + 1);
		final String fullWithoutExt = path.substring(0, path.lastIndexOf('.'));

		if (!ALSO_EXISTING_ONES && (new File(fullWithoutExt + ".png")).exists()
				&& (new File(fullWithoutExt + "_seg.png")).exists()) {
			return;
		}

		final MeshReasoning mr = MeshReasoning.initMeshReasoning(true);
		final ImageGeneratorState imageGeneratorMonitor = new ImageGeneratorState();
		mr.frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		mr.frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				System.out.println("Close");
				imageGeneratorMonitor.currentState = ImageGeneratorState.Stop;
				synchronized (imageGeneratorMonitor) {
					imageGeneratorMonitor.notifyAll();
				}
				synchronized (monitor) {
					System.out.println("notify");
					monitor.notifyAll();
				}
				mr.frame.dispose();
			}
		});

		mr.setFrameTitle(filename);

		mr.setDefaultImageFilename(fullWithoutExt);
		mr.analyseByPath(path, imageGeneratorMonitor);

		synchronized (monitor) {
			try {
				monitor.wait();
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			System.out.println("After wait");
		}
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
