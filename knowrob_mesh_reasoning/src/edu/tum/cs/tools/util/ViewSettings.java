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

import peasy.CameraState;
import peasy.PeasyCam;

/**
 * @author Stefan Profanter
 * 
 */
public final class ViewSettings {

	public static boolean loadSettings(File path, PeasyCam cam) {
		if (!path.exists())
			return false;
		XMLDecoder dec = null;

		try {
			dec = new XMLDecoder(new FileInputStream(path));

			CameraState state = (CameraState) dec.readObject();
			cam.setState(state);

		} catch (IOException e) {
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
			enc.writeObject(cam.getState());
		} catch (IOException e) {
			e.printStackTrace();
			return false;
		} finally {
			if (enc != null)
				enc.close();
		}
		return true;
	}

}
