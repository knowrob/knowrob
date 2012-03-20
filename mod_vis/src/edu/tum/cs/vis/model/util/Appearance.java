package edu.tum.cs.vis.model.util;

import java.awt.Color;

import processing.core.PImage;

/**
 * Appearance of triangle or line.
 * May contain texture or simply a color.
 * 
 * @author Stefan Profanter
 *
 */
public class Appearance {
	/**
	 * the Filename of the texture-Image
	 */
	public String imageFileName = null;

	/**
	 * an Object reference of a loaded image. Default is null.
	 */
	public PImage imageReference = null;

	/**
	 * the color of the Triangle if the Triangle has no texture
	 * Set to null if no fill needed
	 */
	public Color colourFill = null;
	
	/**
	 * the color stroke for Triangle or Line
	 * Set to null if no stroke needed
	 */
	public Color colourLine = null;

	/**
	 * Thickness of the stroke
	 */
	public int strokeWeight = 2;

}
