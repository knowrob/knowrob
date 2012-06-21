/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.view;

import processing.core.PGraphics3D;

/**
 * PGraphics3D graphics context for PAppletSelection to allow drawing everything with a specific
 * transparency level. Overrides fill and stroke.
 * 
 * @author Stefan Profanter
 * 
 */
public class PAppletSelectionGraphics extends PGraphics3D {
	/**
	 * Set to true if everything should be drawn with alpha value of TRANSPARENCY_ALPHA
	 */
	private boolean				drawWithTransparency	= false;
	/**
	 * Alpha channel value for drawing with transparency
	 */
	private static final float	TRASPARENCY_ALPHA		= 20f;

	@Override
	public void fill(float arg) {
		if (drawWithTransparency)
			super.fill(arg, TRASPARENCY_ALPHA);
		else
			super.fill(arg);
	}

	@Override
	public void fill(float arg, float alpha) {
		if (drawWithTransparency)
			super.fill(arg, TRASPARENCY_ALPHA);
		else
			super.fill(arg, alpha);
	}

	@Override
	public void fill(float arg, float arg_1, float arg_2) {
		if (drawWithTransparency)
			super.fill(arg, arg_1, arg_2, TRASPARENCY_ALPHA);
		else
			super.fill(arg, arg_1, arg_2);
	}

	@Override
	public void fill(float arg, float arg_1, float arg_2, float arg_3) {
		if (drawWithTransparency)
			super.fill(arg, arg_1, arg_2, TRASPARENCY_ALPHA);
		else
			super.fill(arg, arg_1, arg_2, arg_3);
	}

	@Override
	public void fill(int arg) {
		this.fill((float) arg);

	}

	@Override
	public void fill(int arg, float arg_1) {
		this.fill((float) arg, arg_1);

	}

	/**
	 * @return the drawWithTransparency
	 */
	public boolean isDrawWithTransparency() {
		return drawWithTransparency;
	}

	/**
	 * @param drawWithTransparency
	 *            the drawWithTrasnparency to set
	 */
	public void setDrawWithTransparency(boolean drawWithTransparency) {
		this.drawWithTransparency = drawWithTransparency;

		// Make also textures transparent
		if (drawWithTransparency)
			tint(255, TRASPARENCY_ALPHA);
		else
			noTint();
	}

	@Override
	public void stroke(float arg) {
		if (drawWithTransparency)
			super.stroke(arg, TRASPARENCY_ALPHA);
		else
			super.stroke(arg);
	}

	@Override
	public void stroke(float arg, float alpha) {
		if (drawWithTransparency)
			super.stroke(arg, TRASPARENCY_ALPHA);
		else
			super.stroke(arg, alpha);
	}

	@Override
	public void stroke(float arg, float arg_1, float arg_2) {
		if (drawWithTransparency)
			super.stroke(arg, arg_1, arg_2, TRASPARENCY_ALPHA);
		else
			super.stroke(arg, arg_1, arg_2);
	}

	@Override
	public void stroke(float arg, float arg_1, float arg_2, float arg_3) {
		if (drawWithTransparency)
			super.stroke(arg, arg_1, arg_2, TRASPARENCY_ALPHA);
		else
			super.stroke(arg, arg_1, arg_2, arg_3);
	}

	@Override
	public void stroke(int arg) {
		this.stroke((float) arg);

	}

	@Override
	public void stroke(int arg, float arg_1) {
		this.stroke((float) arg, arg_1);

	}

}
