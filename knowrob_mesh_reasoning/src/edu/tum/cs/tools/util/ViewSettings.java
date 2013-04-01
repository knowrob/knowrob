/*******************************************************************************
 * Copyright (c) 2013 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2013
 ******************************************************************************/
package edu.tum.cs.tools.util;

import java.beans.XMLDecoder;
import java.beans.XMLEncoder;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.Serializable;

import peasy.PeasyCam;

/**
 * Wrapper class for image generator view settings. Provides methods for saving and loading view
 * settings.
 * 
 * @author Stefan Profanter
 * 
 */
public final class ViewSettings implements Serializable {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 4193708032337186974L;

	/**
	 * Load view settings from specified path (.xml file) and set the Camera to this view.
	 * 
	 * @param path
	 *            Path to view settings xml file
	 * @param cam
	 *            Camera to initialize with loaded values.
	 * @return false if file doesn't exist or parse error.
	 */
	public static boolean loadSettings(File path, PeasyCam cam) {
		if (!path.exists())
			return false;
		XMLDecoder dec = null;

		try {
			dec = new XMLDecoder(new FileInputStream(path));

			ViewSettings settings = (ViewSettings) dec.readObject();

			cam.setRotations(settings.rotation[0], settings.rotation[1], settings.rotation[2]);
			cam.lookAt(settings.lookAt[0], settings.lookAt[1], settings.lookAt[2]);
			cam.setDistance(settings.distance);
			cam.feed(); // update state
		} catch (Exception e) {
			e.printStackTrace();
			return false;
		} finally {
			if (dec != null)
				dec.close();
		}
		return true;
	}

	/**
	 * Save camera view settings to .xml file indicated by path. Already existing file will be
	 * overwritten.
	 * 
	 * @param path
	 *            Path for .xml file. <code>path</code> should contain the extension .xml
	 * @param cam
	 *            Current camera view
	 * @return true if successfully saved.
	 */
	public static boolean saveSettings(File path, PeasyCam cam) {
		XMLEncoder enc = null;

		try {
			enc = new XMLEncoder(new FileOutputStream(path));
			ViewSettings settings = new ViewSettings(cam);

			enc.writeObject(settings);
		} catch (IOException e) {
			e.printStackTrace();
			return false;
		} finally {
			if (enc != null)
				enc.close();
		}
		return true;
	}

	/**
	 * Camera rotation
	 */
	private float[]	rotation;

	/**
	 * Camera look at
	 */
	private float[]	lookAt;

	/**
	 * Camera distance
	 */
	private double	distance;

	/**
	 * Default constructor used by XML deserializer.
	 */
	public ViewSettings() {
		rotation = new float[3];
		lookAt = new float[3];
		distance = 50;
	}

	/**
	 * Initialize view settings with parameters of provided camera.
	 * 
	 * @param cam
	 *            Camera to initialize view settings
	 */
	public ViewSettings(PeasyCam cam) {
		rotation = cam.getRotations();
		lookAt = cam.getLookAt();
		distance = cam.getDistance();
	}

	/**
	 * Distance of camera to lookAt point.
	 * 
	 * @return the distance
	 */
	public double getDistance() {
		return distance;
	}

	/**
	 * Look at point.
	 * 
	 * @return the lookAt
	 */
	public float[] getLookAt() {
		return lookAt;
	}

	/**
	 * Camera rotation.
	 * 
	 * @return the rotation
	 */
	public float[] getRotation() {
		return rotation;
	}

	/**
	 * Set distance of camera to lookAt point.
	 * 
	 * @param distance
	 *            the distance to set
	 */
	public void setDistance(double distance) {
		this.distance = distance;
	}

	/**
	 * Look at point of camera
	 * 
	 * @param lookAt
	 *            the lookAt to set
	 */
	public void setLookAt(float[] lookAt) {
		this.lookAt = lookAt;
	}

	/**
	 * Camera rotation
	 * 
	 * @param rotation
	 *            the rotation to set
	 */
	public void setRotation(float[] rotation) {
		this.rotation = rotation;
	}

}
