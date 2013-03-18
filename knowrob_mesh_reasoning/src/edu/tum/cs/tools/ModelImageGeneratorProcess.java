/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools;


/**
 * @author Stefan Profanter
 * 
 */
public class ModelImageGeneratorProcess {
	/**
	 * @param args
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
