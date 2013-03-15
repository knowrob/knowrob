/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools;

import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.util.Arrays;

import javax.swing.WindowConstants;

import edu.tum.cs.tools.ImageGenerator.ImageGeneratorAction;
import edu.tum.cs.tools.ImageGenerator.ImageGeneratorSettings;
import edu.tum.cs.vis.model.MeshReasoning;
import edu.tum.cs.vis.model.parser.ModelParser;
import edu.tum.cs.vis.model.uima.analyser.ContainerAnalyser;
import edu.tum.cs.vis.model.uima.analyser.PrimitiveAnalyser;
import edu.tum.cs.vis.model.uima.annotation.ContainerAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.util.ContainerAnnotationVolumeComarator;
import edu.tum.cs.vis.model.util.PrimitiveAnnotationAreaComparator;

/**
 * @author Stefan Profanter
 * 
 */
public class ModelImageGenerator {

	public static final String		MODEL_DIR			= "/home/stefan/work/models";
	public static final String		IMAGE_DIR			= "/home/stefan/work/model_images";
	public static final boolean		ALSO_EXISTING_ONES	= false;
	public static Object			monitor				= new Object();

	static ImageGeneratorSettings	settings;

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		settings = new ImageGeneratorSettings(new File(MODEL_DIR), new File(IMAGE_DIR));
		settings.setDrawAxis(false);
		settings.setWhiteBackground(true);
		// If there is a view settings file, load those settings
		settings.setInitViewFromFile(true);
		// If no view settings exist yet, allow to align model and then continue by pressing Ctrl+S
		settings.setSaveView(true);
		settings.setSavePlainModel(true);
		settings.setSaveCurvatureColor(true);
		settings.addAnalyserToSave(PrimitiveAnalyser.class, "segmented");

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

		/**
		 * Analyser actions begin
		 */
		settings.clearAnalyserActions();
		settings.addAnalyserAction(PrimitiveAnalyser.class, new ImageGeneratorAction() {

			@Override
			public void trigger(ImageGeneratorSettings localSettings) {
				mr.clearHightlight();
				@SuppressWarnings("rawtypes")
				PrimitiveAnnotation[] cones = mr.findAnnotationsCone().toArray(
						new PrimitiveAnnotation[0]);
				Arrays.sort(cones, new PrimitiveAnnotationAreaComparator());
				int max = 3;
				for (int i = 0; i < max && i < cones.length; i++) {
					mr.highlightAnnotation(cones[i],
							Color.getHSBColor(1 / 6f, 1f, 1f - (0.5f / max) * i));
				}
				localSettings.waitSaved("cones");
				mr.clearHightlight();
			}
		});
		settings.addAnalyserAction(PrimitiveAnalyser.class, new ImageGeneratorAction() {

			@Override
			public void trigger(ImageGeneratorSettings localSettings) {
				mr.clearHightlight();
				@SuppressWarnings("rawtypes")
				PrimitiveAnnotation[] spheres = mr.findAnnotationsPlane().toArray(
						new PrimitiveAnnotation[0]);
				Arrays.sort(spheres, new PrimitiveAnnotationAreaComparator());
				int max = 3;
				for (int i = 0; i < max && i < spheres.length; i++) {
					mr.highlightAnnotation(spheres[i],
							Color.getHSBColor(0.75f, 1f, 1f - (0.5f / max) * i));
				}
				localSettings.waitSaved("spheres");
				mr.clearHightlight();
			}
		});
		settings.addAnalyserAction(PrimitiveAnalyser.class, new ImageGeneratorAction() {

			@Override
			public void trigger(ImageGeneratorSettings localSettings) {
				mr.clearHightlight();
				@SuppressWarnings("rawtypes")
				PrimitiveAnnotation[] spheres = mr.findAnnotationsSphere().toArray(
						new PrimitiveAnnotation[0]);
				Arrays.sort(spheres, new PrimitiveAnnotationAreaComparator());
				int max = 3;
				for (int i = 0; i < max && i < spheres.length; i++) {
					mr.highlightAnnotation(spheres[i],
							Color.getHSBColor(0f, 1f, 1f - (0.5f / max) * i));
				}
				localSettings.waitSaved("spheres");
				mr.clearHightlight();
			}
		});

		settings.addAnalyserAction(ContainerAnalyser.class, new ImageGeneratorAction() {

			@Override
			public void trigger(ImageGeneratorSettings localSettings) {
				mr.clearHightlight();
				@SuppressWarnings("rawtypes")
				ContainerAnnotation[] container = mr.findAnnotations(ContainerAnnotation.class)
						.toArray(new ContainerAnnotation[0]);
				Arrays.sort(container, new ContainerAnnotationVolumeComarator());
				int max = 3;
				for (int i = 0; i < max && i < container.length; i++) {
					mr.highlightAnnotation(container[i],
							Color.getHSBColor(160f / 360f, 1f, 1f - (0.5f / max) * i));
				}
				localSettings.waitSaved("container");
				mr.clearHightlight();
			}
		});

		/**
		 * Analyser actions end
		 */

		mr.frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
		mr.frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				System.out.println("Close");
				settings.resetMonitors();
				synchronized (monitor) {
					monitor.notifyAll();
				}
				mr.frame.dispose();
			}
		});

		mr.setFrameTitle(filename);

		settings.setCurrentModel(path.substring(MODEL_DIR.length()));
		mr.analyseByPath(path, settings);

		synchronized (monitor) {
			try {
				monitor.wait();
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
		}
		mr.frame = null;
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
