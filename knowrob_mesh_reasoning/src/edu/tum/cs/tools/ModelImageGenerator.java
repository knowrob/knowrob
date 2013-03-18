/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools;

import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.Reader;

import org.apache.log4j.xml.DOMConfigurator;

import edu.tum.cs.tools.ImageGenerator.ImageGeneratorSettings;
import edu.tum.cs.util.PrintUtil;
import edu.tum.cs.vis.model.MeshReasoning;
import edu.tum.cs.vis.model.parser.ModelParser;

/**
 * @author Stefan Profanter
 * 
 */
public class ModelImageGenerator {

	public static final String	MODEL_DIR			= "/home/stefan/work/models";
	public static final String	IMAGE_DIR			= "/home/stefan/work/model_images";
	public static final boolean	ALSO_EXISTING_ONES	= true;

	// static ImageGeneratorSettings settings;

	@SuppressWarnings("rawtypes")
	public static int exec(Class klass, String file) throws IOException, InterruptedException {
		String javaHome = System.getProperty("java.home");
		String javaBin = javaHome + File.separator + "bin" + File.separator + "java";
		String classpath = System.getProperty("java.class.path");
		String className = klass.getCanonicalName();

		ProcessBuilder builder = new ProcessBuilder(javaBin, "-cp", classpath, className,
				MODEL_DIR, IMAGE_DIR, file);
		builder.redirectErrorStream(true);
		Process process = builder.start();
		Reader reader = new InputStreamReader(process.getInputStream());
		int ch;
		while ((ch = reader.read()) != -1)
			System.out.print((char) ch);
		reader.close();
		process.waitFor();
		return process.exitValue();
	}

	public static void execProcess(String inputFolder, String outputFolder, String path) {
		// System.err.println("Starting generator with setting:\n\tinput:  " + inputFolder
		// + "\n\toutput: " + outputFolder + "\n\tfile:   " + path);
		final String filename = path.substring(path.lastIndexOf(File.separatorChar) + 1);

		ImageGeneratorSettings imageGeneratorSettings = new ImageGeneratorSettings(new File(
				inputFolder), new File(outputFolder));
		final MeshReasoning mr = MeshReasoning.initMeshReasoning(true, imageGeneratorSettings);
		mr.setFrameTitle(filename);
		mr.analyseByPath(path);
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		DOMConfigurator.configureAndWatch("log4j.xml", 60 * 1000);

		long processStartTime = System.currentTimeMillis();
		File dir = new File(MODEL_DIR);
		if (!dir.exists()) {
			System.err.println("Couldn't find directory: " + MODEL_DIR);
			return;
		}
		recursiveTraversal(dir);
		long processDuration = System.currentTimeMillis() - processStartTime;
		System.out.println("###### Ended. Took: " + PrintUtil.prettyMillis(processDuration)
				+ " ######");
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
		final String fullWithoutExt = path.substring(0, path.lastIndexOf('.'));
		final String outPath = new File(IMAGE_DIR, fullWithoutExt.substring(MODEL_DIR.length()))
				.getAbsolutePath();

		File outParentFolder = new File(outPath).getParentFile();
		File allFiles[] = outParentFolder.listFiles();

		String fileRegex = ".*" + new File(outPath).getName() + "-[a-zA-Z0-9]+\\.png$";

		// check if there are already images of this model
		if (!ALSO_EXISTING_ONES) {
			for (File fileCheck : allFiles) {
				if (!fileCheck.isFile())
					continue;
				if (fileCheck.getAbsolutePath().matches(fileRegex))
					return;
			}
		}

		// Delete already existing ones
		for (File fileCheck : allFiles) {
			if (!fileCheck.isFile())
				continue;
			if (fileCheck.getAbsolutePath().matches(fileRegex)) {
				fileCheck.delete();
			}
		}

		// execProcess(MODEL_DIR, IMAGE_DIR, path);

		try {
			if (exec(ModelImageGeneratorProcess.class, path) != 123) {
				System.err.println("Stopping... Last process didn't exit properly");
				System.exit(-1);
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
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
			System.gc();
		}
	}

}
