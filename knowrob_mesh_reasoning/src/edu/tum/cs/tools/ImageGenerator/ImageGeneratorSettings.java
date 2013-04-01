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
 * Different settings for image generator. Allows to enable/disable different image types and view
 * settings.
 * 
 * @author Stefan Profanter
 * 
 */
public class ImageGeneratorSettings {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger	= Logger.getLogger(ImageGeneratorSettings.class);

	/**
	 * Reset a boolean wrapper monitor. Sets internal value to false and calls notifyAll.
	 * 
	 * @param monitor
	 *            The monitor to reset.
	 */
	private static void resetMonitor(BooleanContainer monitor) {
		synchronized (monitor) {
			monitor.val = false;
			monitor.notifyAll(); // avoid that one is stuck. this shouldn't happen
		}
	}

	/**
	 * Triggers the boolean wrapper monitor. Sets internal value to true and calls notifyAll.
	 * 
	 * @param monitor
	 *            The monitor to trigger.
	 */
	private static void triggerMonitor(BooleanContainer monitor) {
		synchronized (monitor) {

			monitor.val = true;
			monitor.notifyAll();
		}
	}

	/**
	 * Wait for the specified monitor to be triggered. If it is already triggered (set to true),
	 * method returns immediately.
	 * 
	 * @param monitor
	 *            Monitor to wait for.
	 */
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

	/**
	 * Triggered after setup of MeshReasoningView is called.
	 */
	private final BooleanContainer													monitorSetup					= new BooleanContainer();

	/**
	 * Triggered each time an image has been saved.
	 */
	private final Object															monitorSaved					= new Object();
	/**
	 * If there is no .xml file for view settings, mesh reasoning waits for the suer to set desired
	 * view and press Ctrl+S. This monitor is used to wait and trigger if Ctrl+S is pressed.
	 */
	private final BooleanContainer													monitorViewInitialized			= new BooleanContainer();

	/**
	 * Path to the root source folder of the model.
	 */
	private final File																inputBasePath;
	/**
	 * Path to the root output folder for the images
	 */
	private final File																outputBasePath;
	/**
	 * Current model file
	 */
	private String																	currentModel					= null;

	/**
	 * File name for the new image to save.
	 */
	private String																	currentImageFile				= null;

	/**
	 * If set to true, view settings are loaded from file. If set to false, default values are
	 * taken.
	 */
	private boolean																	initViewFromFile				= true;

	/**
	 * Save plain image of the model. Image is saved directly after parsing. Contains textures (if
	 * available) and colors, but no annotations. postfix: plain
	 */
	private boolean																	savePlainModel					= true;

	/**
	 * Save model image colored by curvature. postfix: curvature
	 */
	private boolean																	saveCurvatureColor				= true;

	/**
	 * Use white background (black otherwise).
	 */
	private boolean																	whiteBackground					= true;

	/**
	 * Draw main axis (x,y,z).
	 */
	private boolean																	drawAxis						= false;

	/**
	 * Set to true after view successfully initialized from view settings file .xml
	 */
	private boolean																	viewInitialized					= false;

	/**
	 * If set to true, current view settings are saved if they don't exist yet.
	 */
	private boolean																	saveView						= true;

	/**
	 * If only one annotation is selected (highlighted) it's fitted plane/cone/sphere is drawn. If
	 * multiple annotations are selected, only the triangles are highlighted but no fitted primitive
	 * is drawn. If set to true, fitted primitives are even drawn if multiple annotations are
	 * selected (useful if you need to generate images for e.g. a spoon where you want to show
	 * fitted cone and sphere in one image).
	 */
	private boolean																	alwaysDrawSelectedPrimitives	= false;

	/**
	 * Close/Exit mesh reasoning process after all analyzer have finished their job.
	 */
	private boolean																	closeAfterFinish				= false;

	/**
	 * If an analyzer finishes and this map contains this analyzer, an image file is stored with the
	 * given postfix in the map.
	 */
	private final Map<Class<? extends MeshAnalyser>, String>						analyserSaveList				= new HashMap<Class<? extends MeshAnalyser>, String>();

	/**
	 * If an analyzer finishes and this map contains this analyzer, the given action is executed.
	 */
	private final Map<Class<? extends MeshAnalyser>, List<ImageGeneratorAction>>	analyserActionList				= new HashMap<Class<? extends MeshAnalyser>, List<ImageGeneratorAction>>();

	/**
	 * Creates new image generator settings initialized with given arguments.
	 * 
	 * @param inputBasePath
	 *            Base root folder where the model file was found
	 * @param outputBasePath
	 *            Base root folder where the image files should be stored.
	 */
	public ImageGeneratorSettings(File inputBasePath, File outputBasePath) {
		this.inputBasePath = inputBasePath;
		this.outputBasePath = outputBasePath;
		if (!outputBasePath.exists()) {
			logger.error("Image output path doesn't exist. You better create it: "
					+ outputBasePath.getAbsolutePath());
		}
	}

	/**
	 * Add an action after the specified analyzer finishes. After the analyzer finished it's job,
	 * the specified action is called. List may also contain multiple actions for the same analyzer.
	 * 
	 * @param analyser
	 *            Class of the analyzer for the action
	 * @param action
	 *            action to execute when analyzer has finished.
	 */
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

	/**
	 * Add an analyzer to store a specific image with given postfix after the specified analyzer
	 * finishes. Only one postfix per action may be added. If there is already an existing one, it
	 * will be overwritten.
	 * 
	 * @param analyser
	 *            Analyzer class
	 * @param append
	 *            postfix for the image. Image file name will be originalName-postfix.png
	 */
	public void addAnalyserToSave(Class<? extends MeshAnalyser> analyser, String append) {
		analyserSaveList.put(analyser, append);
	}

	/**
	 * Remove all analyzer actions.
	 */
	public void clearAnalyserActions() {
		analyserActionList.clear();
	}

	/**
	 * make directories for output base path and current model.
	 */
	private void createOutputPath() {
		File f = new File(outputBasePath, currentModel);
		f = f.getParentFile();
		if (!f.exists() && !f.mkdirs()) {
			logger.error("Couldn't create output directories: " + f.getAbsolutePath());
		}
	}

	/**
	 * Get currently set image file name and clear it immediately.
	 * 
	 * @return currentImageFile value
	 */
	public String getAndClearCurrentImageFile() {
		synchronized (monitorSaved) {
			String ret = currentImageFile;
			currentImageFile = null;
			return ret;
		}
	}

	/**
	 * @return the currentModel path
	 */
	public String getCurrentModel() {
		return currentModel;
	}

	/**
	 * @return the inputBasePath path
	 */
	public File getInputBasePath() {
		return inputBasePath;
	}

	/**
	 * @return the outputBasePath
	 */
	public File getOutputBasePath() {
		return outputBasePath;
	}

	/**
	 * Load view settings for current model from .xml file.
	 * 
	 * @param cam
	 *            Camera to initialize
	 * @return true if view loaded successfully from .xml file
	 */
	public boolean initView(PeasyCam cam) {
		return (viewInitialized = ViewSettings.loadSettings(replaceModelExtension("xml"), cam));
	}

	/**
	 * If only one annotation is selected (highlighted) it's fitted plane/cone/sphere is drawn. If
	 * multiple annotations are selected, only the triangles are highlighted but no fitted primitive
	 * is drawn. If set to true, fitted primitives are even drawn if multiple annotations are
	 * selected (useful if you need to generate images for e.g. a spoon where you want to show
	 * fitted cone and sphere in one image).
	 * 
	 * @return true or false
	 */
	public boolean isAlwaysDrawSelectedPrimitives() {
		return alwaysDrawSelectedPrimitives;
	}

	/**
	 * True if mesh reasoning should exit after all analyzer ended
	 * 
	 * @return the closeAfterFinish
	 */
	public boolean isCloseAfterFinish() {
		return closeAfterFinish;
	}

	/**
	 * True if main axis should be drawn.
	 * 
	 * @return the drawAxis
	 */
	public boolean isDrawAxis() {
		return drawAxis;
	}

	/**
	 * True if view should be initialized from .xml file.
	 * 
	 * @return the initViewFromFile
	 */
	public boolean isInitViewFromFile() {
		return initViewFromFile;
	}

	/**
	 * True if curvature image should be saved.
	 * 
	 * @return the saveCurvatureColor
	 */
	public boolean isSaveCurvatureColor() {
		return saveCurvatureColor;
	}

	/**
	 * True if plain model image should be saved.
	 * 
	 * @return the savePlainModel
	 */
	public boolean isSavePlainModel() {
		return savePlainModel;
	}

	/**
	 * True if current view should be stored in the corresponding .xml file if it doesn't exist yet.
	 * 
	 * @return the saveView
	 */
	public boolean isSaveView() {
		return saveView;
	}

	/**
	 * True after view successfully initialized from view settings file .xml
	 * 
	 * @return the viewInitialized
	 */
	public boolean isViewInitialized() {
		return viewInitialized;
	}

	/**
	 * True if images are stored with white background, false if black background.
	 * 
	 * @return the whiteBackground
	 */
	public boolean isWhiteBackground() {
		return whiteBackground;
	}

	/**
	 * Get output path with given extension. It is outputBasePath+modelName+extension
	 * 
	 * @param extension
	 *            desired extension
	 * @return outputBasePath+modelName+extension
	 */
	private File replaceModelExtension(String extension) {
		return replaceModelExtension(null, extension);
	}

	/**
	 * Get output path with given postifx and extension. It is
	 * outputBasePath+modelName+append+extension
	 * 
	 * @param append
	 *            posfix to add after model name
	 * @param extension
	 *            desired extension
	 * @return outputBasePath+modelName+append+extension
	 */
	private File replaceModelExtension(String append, String extension) {
		String newFile = currentModel.substring(0, currentModel.lastIndexOf('.'));
		if (append != null && append.length() > 0)
			newFile = newFile + "-" + append + "." + extension;
		else
			newFile = newFile + "." + extension;
		return new File(outputBasePath, newFile);
	}

	/**
	 * reset all monitors. Used as reinitialization of image generator settings.
	 */
	public void resetMonitors() {
		resetMonitor(monitorSetup);
		synchronized (monitorSaved) {
			monitorSaved.notifyAll();
		}
		resetMonitor(monitorViewInitialized);
		currentImageFile = null;
		viewInitialized = false;
	}

	/**
	 * Save current view into corresponding .xml file.
	 * 
	 * @param cam
	 *            Camera representing current view
	 * @param force
	 *            if true overwrites currently existing view file.
	 */
	public void saveView(PeasyCam cam, boolean force) {
		if (!force && (viewInitialized || !saveView))
			return;
		ViewSettings.saveSettings(replaceModelExtension("xml"), cam);
		viewInitialized = true;
	}

	/**
	 * If only one annotation is selected (highlighted) it's fitted plane/cone/sphere is drawn. If
	 * multiple annotations are selected, only the triangles are highlighted but no fitted primitive
	 * is drawn. If set to true, fitted primitives are even drawn if multiple annotations are
	 * selected (useful if you need to generate images for e.g. a spoon where you want to show
	 * fitted cone and sphere in one image).
	 * 
	 * @param alwaysDrawSelectedPrimitives
	 *            the alwaysDrawSelectedPrimitives to set
	 */
	public void setAlwaysDrawSelectedPrimitives(boolean alwaysDrawSelectedPrimitives) {
		this.alwaysDrawSelectedPrimitives = alwaysDrawSelectedPrimitives;
	}

	/**
	 * Set to true if process should exit after finished.
	 * 
	 * @param closeAfterFinish
	 *            true for exit
	 */
	public void setCloseAfterFinish(boolean closeAfterFinish) {
		this.closeAfterFinish = closeAfterFinish;
	}

	/**
	 * Current model path.
	 * 
	 * @param currentModel
	 *            the currentModel path to set
	 */
	public void setCurrentModel(String currentModel) {
		this.currentModel = currentModel;
		createOutputPath();
	}

	/**
	 * Set to true if main axis should be drawn and saved into the images.
	 * 
	 * @param drawAxis
	 *            true for drawing main axis
	 */
	public void setDrawAxis(boolean drawAxis) {
		this.drawAxis = drawAxis;
	}

	/**
	 * Set to true if view should be initialized from corresponding model.xml file.
	 * 
	 * @param initViewFromFile
	 *            true to initialize view
	 */
	public void setInitViewFromFile(boolean initViewFromFile) {
		this.initViewFromFile = initViewFromFile;
	}

	/**
	 * Set to true if curvature image should be saved. postfix: curvature.
	 * 
	 * @param saveCurvatureColor
	 *            true to save curvature image
	 */
	public void setSaveCurvatureColor(boolean saveCurvatureColor) {
		this.saveCurvatureColor = saveCurvatureColor;
	}

	/**
	 * Save plain image of the model. Image is saved directly after parsing. Contains textures (if
	 * available) and colors, but no annotations. postfix: plain
	 * 
	 * @param savePlainModel
	 *            true to save plain model image
	 */
	public void setSavePlainModel(boolean savePlainModel) {
		this.savePlainModel = savePlainModel;
	}

	/**
	 * 
	 * Set to true if view settings should be saved after set by user.
	 * 
	 * @param saveView
	 *            true to save view settings in model.xml
	 */
	public void setSaveView(boolean saveView) {
		this.saveView = saveView;
	}

	/**
	 * 
	 * True if images should be stored with white background, false if black background.
	 * 
	 * @param whiteBackground
	 *            the whiteBackground to set
	 */
	public void setWhiteBackground(boolean whiteBackground) {
		this.whiteBackground = whiteBackground;
	}

	/**
	 * Trigger save actions and image generator actions for specified analyzer. Should only be
	 * called after the specified analyzer has finished.
	 * 
	 * @param meshAnalyser
	 *            Analyzer which has finished.
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
				for (ImageGeneratorAction action : analyserActionList.get(clazz)) {
					System.out.println("Trigger: " + clazz);
					action.trigger(this);
				}
			}
		}

	}

	/**
	 * Trigger saved monitor. Called after an images has been saved.
	 */
	public void triggerSaved() {
		synchronized (monitorSaved) {
			currentImageFile = null;
			monitorSaved.notifyAll();
		}
	}

	/**
	 * Trigger setup monitor. Called after mesh reasoning view is set up.
	 */
	public void triggerSetup() {
		triggerMonitor(monitorSetup);
	}

	/**
	 * Trigger view initialized monitor. Called after view has been set by the user by pressing
	 * Ctrl+S
	 */
	public void triggerViewInitialized() {
		triggerMonitor(monitorViewInitialized);
	}

	/**
	 * Wait for saved monitor. This method triggers the save action so that a new image is saed with
	 * the specified postfix. It waits until the image is successfully saved. If there is already
	 * another thread waiting for a saved image, all successive calls wait until this image is saved
	 * and then continue.
	 * 
	 * @param append
	 *            The postifx to append after model name and before extension.
	 */
	public void waitSaved(String append) {
		while (currentImageFile != null) {
			// wait if there is another file currently saving
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// do nothing
			}

		}

		synchronized (monitorSaved) {
			currentImageFile = replaceModelExtension(append, "png").getAbsolutePath();
			// logger.debug("Waiting for saved image: " + currentImageFile);
			try {
				monitorSaved.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * wait until setup monitor is triggered.
	 */
	public void waitSetup() {
		waitMonitor(monitorSetup);
	}

	/**
	 * Wait until view initialized monitor is triggered.
	 */
	public void waitViewInitialized() {
		waitMonitor(monitorViewInitialized);
	}
}
