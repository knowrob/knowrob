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
import java.util.ArrayList;

import javax.swing.WindowConstants;

import edu.tum.cs.vis.model.MeshReasoning;
import edu.tum.cs.vis.model.parser.ModelParser;

/**
 * @author Stefan Profanter
 * 
 */
public class MeshReasoningBatchTest {

	public static ArrayList<String>	files				= new ArrayList<String>();

	public static int				currentWindowCount	= 0;

	public static Object			monitor				= new Object();

	public static int				skip				= 0;
	public static int				currSkip			= 0;

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		String folder = "models/batch/bowl/";
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

	public static void processFile(File f) {
		if (ModelParser.findParser(f.getAbsolutePath()) == null)
			return;

		String path = f.getAbsolutePath();
		String filename = path.substring(path.lastIndexOf(File.separatorChar) + 1);

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

	ArrayList<Thread>	threads	= new ArrayList<Thread>();

}
