/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools;

/**
 * Wrapper class for a new MeshReasoning process. Generates images for the given model file.
 * 
 * @author Stefan Profanter
 * 
 * @see edu.tum.cs.tools.ModelImageGenerator
 * 
 */
public class ModelImageGeneratorProcess {
	/**
	 * Main method called by a child process of ModelImageGenerator.
	 * 
	 * @param args
	 *            Takes 3 arguments: inputFolder outputFolder file inputFolder is the root model
	 *            directory, outputFolder the root image directory, file the model file to process.
	 * 
	 * 
	 * @see edu.tum.cs.tools.ModelImageGenerator
	 */
	public static void main(String[] args) {
		if (args.length != 3) {
			System.err.println("usage: inputFolder outputFolder file");
			System.exit(-1);
		}
		String inputFolder = args[0];

		String outputFolder = args[1];
		String path = args[2];
		ModelImageGenerator.execProcess(inputFolder, outputFolder, path);
	}
}
