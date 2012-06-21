/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view;

import javax.vecmath.Point3f;

import processing.core.PApplet;
import processing.core.PGraphics3D;
import processing.core.PMatrix3D;

/**
 * PApplet which provides selection by mouse click. Uses gluUnProject.
 * 
 * Source: <a href=
 * "http://andrewmarsh.com/scripts/analysis-and-simulation/gluunproject-function-for-p3d-and-opengl-sketches"
 * >glUnProject for PGraphics3D and OpenGL</a>
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class PAppletSelection extends PApplet {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID		= -7052409450791074523L;

	/**
	 * true if last calculated mouse ray is valid
	 */
	private boolean				mouseRayValid			= false;
	/**
	 * current transformation matrix. Updated by captureViewMatrix();
	 */
	private final PMatrix3D		transformationMatrix	= new PMatrix3D();

	/**
	 * Array of current viewport. Updated by captureViewMatrix();
	 */
	private final int[]			viewportArray			= new int[4];
	/**
	 * Start point of mouse ray
	 */
	private final Point3f		mouseRayStart			= new Point3f();

	/**
	 * End point of mouse ray
	 */
	private final Point3f		mouseRayEnd				= new Point3f();

	/**
	 * previous projection matrix. used to compare if it has changed
	 */
	private PMatrix3D			prevProjectionMatrix	= new PMatrix3D();

	/**
	 * previous modelview matrix. used to compare if it has changed
	 */
	private PMatrix3D			prevModelviewMatrix		= new PMatrix3D();

	/**
	 * Calculates the mouseRayStart and mouseRayEnd points for a mouse click at the given
	 * position.Calculate positions on the near and far 3D frustum planes.
	 * 
	 * @param x
	 *            x coordinate of mouse click (0 to screen width) (eg. 0 - 1024)
	 * @param y
	 *            y coordinate of mouse click (0 to screen height) (eg. 0 - 768)
	 * @return true if calculated mouse ray is valid
	 */
	public boolean calculatePickPoints(int x, int y) {
		mouseRayValid = true; // Have to do both in order to reset PVector on error.
		if (!gluUnProject(x, y, 0.0f, mouseRayStart))
			mouseRayValid = false;
		if (!gluUnProject(x, y, 1.0f, mouseRayEnd))
			mouseRayValid = false;
		return mouseRayValid;
	}

	/**
	 * Captures the current projection and modelview matrix. Call this to capture the selection
	 * matrix after you have called perspective() or ortho() and applied your pan, zoom and camera
	 * angles - but before you start drawing or playing with the matrices any further.
	 */
	public synchronized void captureViewMatrix() {

		PGraphics3D g3d = (PGraphics3D) g;

		// Noting changed?
		if (g3d.projection.equals(prevProjectionMatrix)
				&& g3d.modelview.equals(prevModelviewMatrix))
			return;

		prevProjectionMatrix = new PMatrix3D(g3d.projection);
		prevModelviewMatrix = new PMatrix3D(g3d.modelview);

		// Capture current projection matrix.
		transformationMatrix.set(g3d.projection);

		// Multiply by current modelview matrix.
		transformationMatrix.apply(g3d.modelview);

		// Invert the resultant matrix.
		transformationMatrix.invert();

		// Store the viewport.
		viewportArray[0] = 0;
		viewportArray[1] = 0;
		viewportArray[2] = g3d.width;
		viewportArray[3] = g3d.height;

	}

	/**
	 * @return the mouseRayEnd
	 */
	public Point3f getMouseRayEnd() {
		return mouseRayEnd;
	}

	/**
	 * @return the mouseRayStart
	 */
	public Point3f getMouseRayStart() {
		return mouseRayStart;
	}

	/**
	 * Get the PGraphics3D graphics for the PAppletSelection panel initialized by the size method.
	 * The graphics context of the PAppletSelection must be instanceof PAppletSelectionGraphics
	 * otherwise this method will fail.
	 * 
	 * @return the graphics context if it is valid or null otherwise.
	 */
	public PAppletSelectionGraphics getSelectionGraphics() {
		if (g instanceof PAppletSelectionGraphics)
			return (PAppletSelectionGraphics) g;

		System.err
				.println("PAppletSelection wasn't created with PAppletSelectionGraphics. Use size(x,y,\"package.PAppletSelectionGraphics\"");
		return null;
	}

	/**
	 * OpenGL gluUnProject like method to project 2D coordinates into 3D scene.
	 * 
	 * @param winx
	 *            x position of mouse
	 * @param winy
	 *            y position of mouse
	 * @param winz
	 *            z position of mouse (set to 0 and then to 1)
	 * @param result
	 *            the resulting point for given coordinates.
	 * @return true if unproject was successfull
	 */
	public boolean gluUnProject(float winx, float winy, float winz, Point3f result) {

		float[] in = new float[4];
		float[] out = new float[4];

		// Transform to normalized screen coordinates (-1 to 1).
		in[0] = ((winx - viewportArray[0]) / viewportArray[2]) * 2.0f - 1.0f;
		in[1] = ((winy - viewportArray[1]) / viewportArray[3]) * 2.0f - 1.0f;
		in[2] = constrain(winz, 0f, 1f) * 2.0f - 1.0f;
		in[3] = 1.0f;

		// Calculate homogeneous coordinates.
		out[0] = transformationMatrix.m00 * in[0] + transformationMatrix.m01 * in[1]
				+ transformationMatrix.m02 * in[2] + transformationMatrix.m03 * in[3];
		out[1] = transformationMatrix.m10 * in[0] + transformationMatrix.m11 * in[1]
				+ transformationMatrix.m12 * in[2] + transformationMatrix.m13 * in[3];
		out[2] = transformationMatrix.m20 * in[0] + transformationMatrix.m21 * in[1]
				+ transformationMatrix.m22 * in[2] + transformationMatrix.m23 * in[3];
		out[3] = transformationMatrix.m30 * in[0] + transformationMatrix.m31 * in[1]
				+ transformationMatrix.m32 * in[2] + transformationMatrix.m33 * in[3];

		if (out[3] == 0.0f) { // Check for an invalid result.
			result.x = 0.0f;
			result.y = 0.0f;
			result.z = 0.0f;
			return false;
		}

		// Scale to world coordinates.
		out[3] = 1.0f / out[3];
		result.x = out[0] * out[3];
		result.y = out[1] * out[3];
		result.z = out[2] * out[3];
		return true;

	}

	/**
	 * @return the mouseRayValid
	 */
	public boolean isMouseRayValid() {
		return mouseRayValid;
	}

	/**
	 * @param mouseRayValid
	 *            the mouseRayValid to set
	 */
	public void setMouseRayValid(boolean mouseRayValid) {
		this.mouseRayValid = mouseRayValid;
	}
}
