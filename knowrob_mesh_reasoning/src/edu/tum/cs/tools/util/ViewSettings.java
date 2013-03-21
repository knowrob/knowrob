/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
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
 * @author Stefan Profanter
 * 
 */
public final class ViewSettings implements Serializable {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 4193708032337186974L;

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

	private float[]	rotation;

	private float[]	lookAt;

	private double	distance;

	public ViewSettings() {
		rotation = new float[3];
		lookAt = new float[3];
		distance = 50;
	}

	/**
	 * @param cam
	 */
	public ViewSettings(PeasyCam cam) {
		rotation = cam.getRotations();
		lookAt = cam.getLookAt();
		distance = cam.getDistance();
	}

	/**
	 * @return the distance
	 */
	public double getDistance() {
		return distance;
	}

	/**
	 * @return the lookAt
	 */
	public float[] getLookAt() {
		return lookAt;
	}

	/**
	 * @return the rotation
	 */
	public float[] getRotation() {
		return rotation;
	}

	/**
	 * @param distance
	 *            the distance to set
	 */
	public void setDistance(double distance) {
		this.distance = distance;
	}

	/**
	 * @param lookAt
	 *            the lookAt to set
	 */
	public void setLookAt(float[] lookAt) {
		this.lookAt = lookAt;
	}

	/**
	 * @param rotation
	 *            the rotation to set
	 */
	public void setRotation(float[] rotation) {
		this.rotation = rotation;
	}

}
