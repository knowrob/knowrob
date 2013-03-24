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
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Pattern;

import org.apache.log4j.Logger;
import org.apache.log4j.xml.DOMConfigurator;

import edu.tum.cs.ias.knowrob.utils.ThreadPool;
import edu.tum.cs.tools.ImageGenerator.ImageGeneratorSettings;
import edu.tum.cs.util.PrintUtil;
import edu.tum.cs.vis.model.MeshReasoning;
import edu.tum.cs.vis.model.parser.ModelParser;

/**
 * @author Stefan Profanter
 * 
 */
public class ModelImageGenerator {

	public static final String			MODEL_DIR			= "/home/stefan/work/models";
	public static final String			IMAGE_DIR			= "/home/stefan/work/model_images";
	// public static final String MODEL_DIR = "/home/stefan/Dropbox/work/spoon";
	// public static final String IMAGE_DIR = "/home/stefan/Dropbox/work/spoon_images";

	public static final boolean			ALSO_EXISTING_ONES	= false;

	// static ImageGeneratorSettings settings;

	private static List<Callable<Void>>	threads				= new LinkedList<Callable<Void>>();
	static Logger						logger				= Logger.getLogger(ModelImageGenerator.class);

	final static AtomicInteger			currentCount		= new AtomicInteger(0);

	static int							totalCount			= 0;

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
			logger.error("Couldn't find directory: " + MODEL_DIR);
			return;
		}
		logger.debug("Reading directory, please wait ... (" + MODEL_DIR + ")");

		recursiveTraversal(dir);
		totalCount = threads.size();
		ThreadPool.executeInPool(threads, 2);
		threads.clear();
		long processDuration = System.currentTimeMillis() - processStartTime;
		logger.debug("###### Ended. Took: " + PrintUtil.prettyMillis(processDuration) + " ######");
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
		if (ModelParser.findParser(f.getAbsolutePath()) == null) {
			// System.out.println("Skipping file: " + f.getAbsolutePath() + " (no parser found)");
			return;
		}
		final String path = f.getAbsolutePath();
		final String fullWithoutExt = path.substring(0, path.lastIndexOf('.'));
		final String outPath = new File(IMAGE_DIR, fullWithoutExt.substring(MODEL_DIR.length()))
				.getAbsolutePath();

		final File outParentFolder = new File(outPath).getParentFile();
		File allFiles[] = outParentFolder.listFiles();
		final String fileRegex = ".*" + Pattern.quote(new File(outPath).getName())
				+ "-[a-zA-Z0-9]+\\.png$";

		File xmlFile = new File(outPath + ".xml");

		// check if there are already images of this model
		if (!ALSO_EXISTING_ONES && xmlFile.exists()) {
			for (File fileCheck : allFiles) {
				if (!fileCheck.isFile())
					continue;
				if (fileCheck.getAbsolutePath().matches(fileRegex))
					return;
			}
		}

		threads.add(new Callable<Void>() {

			@Override
			public Void call() {
				File filesCheck[] = outParentFolder.listFiles();

				// Delete already existing ones
				for (File fileCheck : filesCheck) {
					if (!fileCheck.isFile())
						continue;
					if (fileCheck.getAbsolutePath().matches(fileRegex)) {
						fileCheck.delete();
					}
				}
				// execProcess(MODEL_DIR, IMAGE_DIR, path);

				try {
					exec(ModelImageGeneratorProcess.class, path);
					/*
						logger.error("Stopping... Last process didn't exit properly");
						System.exit(-1);
					}*/
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				int count = currentCount.incrementAndGet();
				logger.debug("###### Process: " + count + "/" + totalCount + " "
						+ String.format("%.2f%%", count / (float) totalCount * 100f) + " ######");
				return null;
			}

		});

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
			logger.debug("Next directory: " + fileObject.getAbsolutePath());
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
