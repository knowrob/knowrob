package edu.tum.cs.vis.model.view;

import javax.vecmath.Point3f;

import processing.core.PApplet;
import processing.core.PGraphics3D;
import processing.core.PMatrix3D;

public class PAppletSelection extends PApplet {

	/**
	 * 
	 */
	private static final long	serialVersionUID		= -7052409450791074523L;

	private boolean				mouseRayValid			= false;
	private final PMatrix3D		transformationMatrix	= new PMatrix3D();

	private final int[]			viewportArray			= new int[4];
	// Store the near and far ray positions.
	private final Point3f		mouseRayStart			= new Point3f();

	private final Point3f		mouseRayEnd				= new Point3f();

	private PMatrix3D			prevProjectionMatrix	= new PMatrix3D();
	private PMatrix3D			prevModelviewMatrix		= new PMatrix3D();

	public boolean calculatePickPoints(int x, int y) {
		// Calculate positions on the near and far 3D frustum planes.
		mouseRayValid = true; // Have to do both in order to reset PVector on error.
		if (!gluUnProject(x, y, 0.0f, mouseRayStart))
			mouseRayValid = false;
		if (!gluUnProject(x, y, 1.0f, mouseRayEnd))
			mouseRayValid = false;
		return mouseRayValid;
	}

	public synchronized void captureViewMatrix() {
		// Call this to capture the selection matrix after you have called perspective() or ortho()
		// and applied your pan, zoom and camera angles - but before you start drawing or playing
		// with the matrices any further.

		PGraphics3D g3d = (PGraphics3D) g;

		// Noting changed?
		if (g3d.projection.equals(prevProjectionMatrix)
				&& g3d.modelview.equals(prevModelviewMatrix))
			return;

		prevProjectionMatrix = new PMatrix3D(g3d.projection);
		prevModelviewMatrix = new PMatrix3D(g3d.modelview);

		if (g3d != null) { // Check for a valid 3D canvas.

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
