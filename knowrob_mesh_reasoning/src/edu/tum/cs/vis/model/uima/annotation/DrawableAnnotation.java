/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.annotation;

import java.awt.Color;

import processing.core.PGraphics;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.util.DrawSettings;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Base class for a drawable annotation.
 * 
 * @author Stefan Profanter
 * 
 */
public abstract class DrawableAnnotation extends Annotation {
	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= 1222063742441634463L;

	/**
	 * indicates whether this annotation should be drawn or not
	 */
	private boolean				drawAnnotation		= true;

	/**
	 * The annotation color for this type of annotation.
	 */
	private final Color			annotationColor;

	/**
	 * A random annotation color. Each annotation gets also a random color.
	 */
	private final Color			randomAnnotationColor;
	/**
	 * Use random color for drawing the annotation
	 */
	private boolean				useRandomColor		= false;

	/**
	 * Constructor for drawable annotation. Initializes random annotation color and set default
	 * annotation color to given one.
	 * 
	 * @param annotationColor
	 *            default annotation color
	 */
	public DrawableAnnotation(Color annotationColor) {
		randomAnnotationColor = new Color((int) (Math.random() * 255), (int) (Math.random() * 255),
				(int) (Math.random() * 255));
		this.annotationColor = annotationColor;
	}

	/**
	 * Checks if annotation contains given triangle.
	 * 
	 * @param t
	 *            Triangle to check
	 * @return true if annotation contains <code>t</code>
	 */
	public abstract boolean containsTriangle(Triangle t);

	/**
	 * Draw the annotation with color from <code>getAnnotationColor()</code>
	 * 
	 * @param g
	 *            Applet to draw on
	 * @param drawSettings
	 *            drawSettings to draw annotation. Useful if you need to override annotation color.
	 *            Can also be null to use default values.
	 */
	public void draw(PGraphics g, DrawSettings drawSettings) {
		if (drawAnnotation || drawSettings.forceDraw)
			drawAnnotation(g, drawSettings);
	}

	/**
	 * Draw this annotation on given graphics context.
	 * 
	 * @param g
	 *            graphics context to draw on
	 * @param drawSettings
	 *            drawSettings to draw annotation. Useful if you need to override annotation color.
	 *            Can also be null to use default values.
	 */
	protected abstract void drawAnnotation(PGraphics g, DrawSettings drawSettings);

	/**
	 * Returns the color for drawing this annotation. If useRandomColor is true, the random color
	 * will be returned.
	 * 
	 * @return randomAnnotationColor if useRandomColor. Otherwise: annotationColor
	 */
	public Color getDrawColor() {
		if (useRandomColor)
			return randomAnnotationColor;

		return annotationColor;
	}

	/**
	 * true if this annotation should be drawn (is visible).
	 * 
	 * @return the drawAnnotation
	 */
	public boolean isDrawAnnotation() {
		return drawAnnotation;
	}

	/**
	 * Indicates if random color is used to draw this annotation
	 * 
	 * @return the useRandomColor
	 */
	public boolean isUseRandomColor() {
		return useRandomColor;
	}

	/**
	 * Set to true, if this annotation should be drawn (is visible).
	 * 
	 * @param drawAnnotation
	 *            the drawAnnotation to set
	 */
	public void setDrawAnnotation(boolean drawAnnotation) {
		this.drawAnnotation = drawAnnotation;
	}

	/**
	 * Set to true if random color should be used to draw annotation instead of predefined one.
	 * 
	 * @param useRandomColor
	 *            the useRandomColor to set
	 */
	public void setUseRandomColor(boolean useRandomColor) {
		this.useRandomColor = useRandomColor;
	}
}
