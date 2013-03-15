/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools.ImageGenerator;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import peasy.PeasyCam;
import edu.tum.cs.tools.util.ViewSettings;
import edu.tum.cs.vis.model.uima.analyser.MeshAnalyser;

/**
 * @author Stefan Profanter
 * 
 */
public class ImageGeneratorSettings {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger	= Logger.getLogger(ImageGeneratorSettings.class);

	private static void resetMonitor(BooleanContainer monitor) {
		synchronized (monitor) {
			monitor.val = false;
			monitor.notifyAll(); // avoid that one is stuck. this shouldn't happen
		}
	}

	private static void triggerMonitor(BooleanContainer monitor) {
		synchronized (monitor) {

			monitor.val = true;
			monitor.notifyAll();
		}
	}

	private static void waitMonitor(BooleanContainer monitor) {
		synchronized (monitor) {
			if (monitor.val)
				return;
			try {
				monitor.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private final BooleanContainer													monitorSetup					= new BooleanContainer();

	private final Object															monitorSaved					= new Object();
	private final BooleanContainer													monitorViewInitialized			= new BooleanContainer();

	private final File																inputBasePath;
	private final File																outputBasePath;
	private String																	currentModel					= null;

	private String																	currentImageFile				= null;

	/**
	 * If set to true, view settings are loaded from file. If set to false, default values are
	 * taken.
	 */
	private boolean																	initViewFromFile				= true;

	private boolean																	savePlainModel					= true;

	private boolean																	saveCurvatureColor				= true;

	private boolean																	whiteBackground					= true;
	private boolean																	drawAxis						= false;

	private boolean																	viewInitialized					= false;
	private boolean																	saveView						= true;
	private boolean																	alwaysDrawSelectedPrimitives	= false;

	private final Map<Class<? extends MeshAnalyser>, String>						analyserSaveList				= new HashMap<Class<? extends MeshAnalyser>, String>();
	private final Map<Class<? extends MeshAnalyser>, List<ImageGeneratorAction>>	analyserActionList				= new HashMap<Class<? extends MeshAnalyser>, List<ImageGeneratorAction>>();

	public ImageGeneratorSettings(File inputBasePath, File outputBasePath) {
		this.inputBasePath = inputBasePath;
		this.outputBasePath = outputBasePath;
		if (!outputBasePath.exists()) {
			logger.error("Image output path doesn't exist. You better create it: "
					+ outputBasePath.getAbsolutePath());
		}
	}

	public void addAnalyserAction(Class<? extends MeshAnalyser> analyser,
			ImageGeneratorAction action) {
		if (analyserActionList.get(analyser) != null)
			analyserActionList.get(analyser).add(action);
		else {
			List<ImageGeneratorAction> list = new ArrayList<ImageGeneratorAction>();
			list.add(action);
			analyserActionList.put(analyser, list);
		}
	}

	public void addAnalyserToSave(Class<? extends MeshAnalyser> analyser, String append) {
		analyserSaveList.put(analyser, append);
	}

	public void clearAnalyserActions() {
		analyserActionList.clear();
	}

	/**
	 * @return the currentImageFile
	 */
	public String getCurrentImageFile() {
		return currentImageFile;
	}

	/**
	 * @return the currentModel
	 */
	public String getCurrentModel() {
		return currentModel;
	}

	/**
	 * @return the inputBasePath
	 */
	public File getInputBasePath() {
		return inputBasePath;
	}

	/**
	 * @return the monitorSetup
	 */
	public BooleanContainer getMonitorSetup() {
		return monitorSetup;
	}

	/**
	 * @return the outputBasePath
	 */
	public File getOutputBasePath() {
		return outputBasePath;
	}

	/**
	 * @param cam
	 */
	public void initView(PeasyCam cam) {
		viewInitialized = ViewSettings.loadSettings(replaceModelExtension("xml"), cam);
	}

	/**
	 * @return the alwaysDrawSelectedPrimitives
	 */
	public boolean isAlwaysDrawSelectedPrimitives() {
		return alwaysDrawSelectedPrimitives;
	}

	/**
	 * @return the drawAxis
	 */
	public boolean isDrawAxis() {
		return drawAxis;
	}

	/**
	 * @return the initViewFromFile
	 */
	public boolean isInitViewFromFile() {
		return initViewFromFile;
	}

	/**
	 * @return the saveCurvatureColor
	 */
	public boolean isSaveCurvatureColor() {
		return saveCurvatureColor;
	}

	/**
	 * @return the savePlainModel
	 */
	public boolean isSavePlainModel() {
		return savePlainModel;
	}

	/**
	 * @return the saveView
	 */
	public boolean isSaveView() {
		return saveView;
	}

	/**
	 * @return the viewInitialized
	 */
	public boolean isViewInitialized() {
		return viewInitialized;
	}

	/**
	 * @return the whiteBackground
	 */
	public boolean isWhiteBackground() {
		return whiteBackground;
	}

	private File replaceModelExtension(String extension) {
		return replaceModelExtension(null, extension);
	}

	private File replaceModelExtension(String append, String extension) {
		String newFile = currentModel.substring(0, currentModel.lastIndexOf('.'));
		if (append != null && append.length() > 0)
			newFile = newFile + "-" + append + "." + extension;
		else
			newFile = newFile + "." + extension;
		return new File(outputBasePath, newFile);
	}

	public void resetMonitors() {
		resetMonitor(monitorSetup);
		synchronized (monitorSaved) {
			monitorSaved.notifyAll();
		}
		resetMonitor(monitorViewInitialized);
		currentImageFile = null;
		viewInitialized = false;
	}

	public void saveView(PeasyCam cam, boolean force) {
		if (!force && (viewInitialized || !saveView))
			return;
		ViewSettings.saveSettings(replaceModelExtension("xml"), cam);
		viewInitialized = true;
	}

	/**
	 * @param alwaysDrawSelectedPrimitives
	 *            the alwaysDrawSelectedPrimitives to set
	 */
	public void setAlwaysDrawSelectedPrimitives(boolean alwaysDrawSelectedPrimitives) {
		this.alwaysDrawSelectedPrimitives = alwaysDrawSelectedPrimitives;
	}

	/**
	 * @param currentModel
	 *            the currentModel to set
	 */
	public void setCurrentModel(String currentModel) {
		this.currentModel = currentModel;
	}

	/**
	 * @param drawAxis
	 *            the drawAxis to set
	 */
	public void setDrawAxis(boolean drawAxis) {
		this.drawAxis = drawAxis;
	}

	/**
	 * @param initViewFromFile
	 *            the initViewFromFile to set
	 */
	public void setInitViewFromFile(boolean initViewFromFile) {
		this.initViewFromFile = initViewFromFile;
	}

	/**
	 * @param saveCurvatureColor
	 *            the saveCurvatureColor to set
	 */
	public void setSaveCurvatureColor(boolean saveCurvatureColor) {
		this.saveCurvatureColor = saveCurvatureColor;
	}

	/**
	 * @param savePlainModel
	 *            the savePlainModel to set
	 */
	public void setSavePlainModel(boolean savePlainModel) {
		this.savePlainModel = savePlainModel;
	}

	/**
	 * @param saveView
	 *            the saveView to set
	 */
	public void setSaveView(boolean saveView) {
		this.saveView = saveView;
	}

	/**
	 * @param whiteBackground
	 *            the whiteBackground to set
	 */
	public void setWhiteBackground(boolean whiteBackground) {
		this.whiteBackground = whiteBackground;
	}

	/**
	 * @param meshAnalyser
	 */
	public void triggerAnalyser(MeshAnalyser meshAnalyser) {
		for (Class<? extends MeshAnalyser> clazz : analyserSaveList.keySet()) {
			if (clazz.isInstance(meshAnalyser)) {
				String append = analyserSaveList.get(clazz);
				waitSaved(append);
			}
		}
		for (Class<? extends MeshAnalyser> clazz : analyserActionList.keySet()) {
			if (clazz.isInstance(meshAnalyser)) {
				for (ImageGeneratorAction action : analyserActionList.get(clazz))
					action.trigger(this);
			}
		}

	}

	public void triggerSaved() {
		synchronized (monitorSaved) {
			currentImageFile = null;
			monitorSaved.notifyAll();
		}
	}

	public void triggerSetup() {
		triggerMonitor(monitorSetup);
	}

	public void triggerViewInitialized() {
		triggerMonitor(monitorViewInitialized);
	}

	public void waitSaved(String append) {

		synchronized (monitorSaved) {
			if (currentImageFile != null) {
				// there is already another job which waits for saving
				try {
					monitorSaved.wait();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			currentImageFile = replaceModelExtension(append, ".png").getAbsolutePath();
			logger.debug("Waiting for saved image: " + currentImageFile);
			try {
				monitorSaved.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public void waitSetup() {
		waitMonitor(monitorSetup);
	}

	public void waitViewInitialized() {
		waitMonitor(monitorViewInitialized);
	}
}
