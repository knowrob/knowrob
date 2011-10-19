package edu.tum.cs.vis.model.util;

import processing.core.PImage;

public class Appearance {
	/**
	 * the Filename of the texture-Image
	 */
	public String imageFileName;
	/**
	 * says if the Triangle has Texture
	 */
	public boolean containsTexture = false;
	/**
	 * an Objectreference of a loaded image. Default is null.
	 */
	public PImage imageReference;
	/**
	 * the color of the Triangle if the Triangle has no texture
	 */
	public Color colour;

}
