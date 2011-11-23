package edu.tum.cs.vis.model.util;

import java.io.File;
import java.util.HashMap;
import java.util.LinkedList;

import javax.vecmath.Point3f;

import processing.core.PApplet;
import processing.core.PImage;

/**
 * A Mesh contains list of Triangles and Lines which represent an Object.
 * 
 * @see Triangle
 * @see Line
 * 
 * @author Stefan Profanter
 *
 */
public class Mesh {
	
	/**
	 * Minimum position values of model position
	 */
	protected Float minX = null;
	protected Float maxX = null;
	protected Float minY = null;
	protected Float maxY = null;
	protected Float minZ = null;
	protected Float maxZ = null;
	
	private boolean texturesInitialized = false;
	private String textureBasePath;
		
	/**
	 * List of all triangles parsed from file
	 */
	public LinkedList<Triangle> triangles = new LinkedList<Triangle>();
	/**
	 * List of all lines parsed from file
	 */
	public LinkedList<Line> lines = new LinkedList<Line>();
	
	
	
	public String getTextureBasePath() {
		return textureBasePath;
	}

	public void setTextureBasePath(String textureBasePath) {
		this.textureBasePath = textureBasePath;
	}

	/**
	 * Draw the triangles list to the applet
	 * 
	 * @param applet
	 *            Applet to draw on
	 * @param colorOverride override the draw color an texture. Draw whole object in the given color if != 0
	 */
	protected void drawTriangles(PApplet applet, int overrideColor) {
		if (!texturesInitialized)
			setTextureImage(applet);
		// Shapes are not effected by translate
		for (Triangle tri : triangles) {
			tri.draw(applet,overrideColor);
		}
	}

	/**
	 * Draw the lines list to the applet
	 * 
	 * @param applet
	 *            Applet to draw on
	 * @param colorOverride override the draw color an texture. Draw whole object in the given color if != 0
	 */
	protected void drawLines(PApplet applet, int overrideColor) {

		// Shapes are not effected by translate
		for (Line line : lines) {

			line.draw(applet,overrideColor);
		}
	}
	
	/**
	 * Scales all coordinates by the given factor
	 * 
	 * @param factor
	 *            The scale factor
	 */
	protected void scaleMesh(float factor) {
		for (Triangle tri : triangles) {

			tri.scale(factor);
		}
		for (Line line : lines) {

			line.scale(factor);
		}
		minX *= factor;
		maxX *= factor;
		minY *= factor;
		maxY *= factor;
		minZ *= factor;
		maxZ *= factor;
	}

	/**
	 * Set absolute position of the mesh. Only used for centering model.
	 * If you want to move the model when drawing, use transformation matrix.
	 * @param pos new absolute position of model
	 */
	public void translate(Point3f translation) {
		minX += translation.x;
		maxX += translation.x;
		minY += translation.y;
		maxY += translation.y;
		minZ += translation.z;
		maxZ += translation.z;
		for (Triangle tri : triangles) {

			tri.translate(translation.x, translation.y, translation.z);
		}
		for (Line line : lines) {

			line.translate(translation.x, translation.y, translation.z);
		}
	}
	
	/**
	 * Draws the bounding box around the model with the current style
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void drawBoundingBox(PApplet applet) {
		if (triangles.size() == 0 && lines.size()==0)
			return; 
		// Save current translation
		applet.pushMatrix();
		applet.translate(maxX-getWidth()/2f, maxY-getHeight()/2f, maxZ-getDepth()/2f);
		applet.box(getWidth(), getHeight(), getDepth());

		// Restore last translation
		applet.popMatrix();
	}
	
	/**
	 * Returns the height of the model by searching the biggest distance on the
	 * y-axis between the vectors.
	 * 
	 * @return float as height of the model
	 */
	public float getHeight() {
		if (minY != null && maxY != null)
			return Math.abs(maxY - minY);
		if (triangles.size() == 0)
		{
			minY = 0f;
			maxY = 0f;
		} else {
			minY = Float.MAX_VALUE;
			maxY = Float.MIN_VALUE;

			for (Triangle tri : triangles) {
				for (int v = 0; v < 3; v++) {
					minY = Math.min(tri.position[v].y, minY);
					maxY = Math.max(tri.position[v].y, maxY);
				}
			}
			for (Line line : lines) {
				for (int v = 0; v < 3; v++) {
					minY = Math.min(line.position[v].y, minY);
					maxY = Math.max(line.position[v].y, maxY);
				}
			}
		}
		return maxY - minY;
	}

	/**
	 * Returns the width of the model by searching the biggest distance on the
	 * x-axis between the vectors.
	 * 
	 * @return float as width of the model
	 */
	public float getWidth() {
		if (minX != null && maxX != null)
			return Math.abs(maxX - minX);
		if (triangles.size() == 0)
		{
			minX = 0f;
			maxX = 0f;
		} else {
			minX = Float.MAX_VALUE;
			maxX = Float.MIN_VALUE;
	
			for (Triangle tri : triangles) {
				for (int v = 0; v < 3; v++) {
					minX = Math.min(tri.position[v].x, minX);
					maxX = Math.max(tri.position[v].x, maxX);
				}
			}
			for (Line line : lines) {
				for (int v = 0; v < 3; v++) {
					minX = Math.min(line.position[v].x, minX);
					maxX = Math.max(line.position[v].x, maxX);
				}
			}
		}
		return maxX - minX;
	}

	/**
	 * Returns the depth of the model by searching the biggest distance on the
	 * z-axis between the vectors.
	 * 
	 * @return float as depth of the model
	 */
	public float getDepth() {
		if (minZ != null && maxZ != null)
			return Math.abs(maxZ - minZ);
		if (triangles.size() == 0)
		{
			minZ = 0f;
			maxZ = 0f;
		} else {
			minZ = Float.MAX_VALUE;
			maxZ = Float.MIN_VALUE;
	
			for (Triangle tri : triangles) {
				for (int v = 0; v < 3; v++) {
					minZ = Math.min(tri.position[v].z, minZ);
					maxZ = Math.max(tri.position[v].z, maxZ);
				}
			}
			for (Line line : lines) {
				for (int v = 0; v < 3; v++) {
					minZ = Math.min(line.position[v].z, minZ);
					maxZ = Math.max(line.position[v].z, maxZ);
				}
			}
		}
		return maxZ - minZ;
	}

	public Float getMinX() {
		return minX;
	}

	public Float getMaxX() {
		return maxX;
	}

	public Float getMinY() {
		return minY;
	}

	public Float getMaxY() {
		return maxY;
	}

	public Float getMinZ() {
		return minZ;
	}

	public Float getMaxZ() {
		return maxZ;
	}
	
	/**
	 * Convert a file path into a File object with an absolute path relative to
	 * a passed in root. If path is absolute then a file object constructed from
	 * new File(path) is returned, otherwise a file object is returned from new
	 * File(root, path) if root is not null, otherwise null is returned.
	 * @param root root file path
	 * @param path relative file path
	 */
	public static String getAbsoluteFilePath(String root, String path) {
		File file = new File(path);
		if (file.isAbsolute())
			return file.getAbsolutePath();

		if (root == null)
			return null;

		return new File(new File(root), path).getAbsolutePath();
	}

	/**
	 * Only used for initialization!
	 * per definition is: textureProcessing x = textureSketchup x *
	 * picture.width textureProcessing y = picture.height - textureSketchup y *
	 * picture.height it also creates an PImage to each Triangle (if it contains
	 * any Texture)
	 */
	private void setTextureImage(PApplet applet) {
		// load all Texture-Images only once (memory efficiency)
		HashMap<String, PImage> pictures = new HashMap<String, PImage>();
		for (Triangle tri : triangles) {
			if (!tri.appearance.containsTexture)
				continue;
			String texfile = getAbsoluteFilePath(textureBasePath,
					tri.appearance.imageFileName);
			if (tri.appearance.containsTexture && pictures.get(texfile) == null) {
				PImage img = applet.loadImage(texfile);
				pictures.put(texfile, img);
			}
		}

		for (Triangle tri : triangles) {
			if (tri.appearance.containsTexture) {
				String texfile = getAbsoluteFilePath(textureBasePath,
						tri.appearance.imageFileName);
				// PImage tex = applet.loadImage(texfile);
				PImage tex = pictures.get(texfile);
				float AprocX = tri.texPosition[0].x * tex.width;
				float AprocY = tex.height - tri.texPosition[0].y * tex.height;
				float BprocX = tri.texPosition[1].x * tex.width;
				float BprocY = tex.height - tri.texPosition[1].y * tex.height;
				float CprocX = tri.texPosition[2].x * tex.width;
				float CprocY = tex.height - tri.texPosition[2].y * tex.height;

				tri.texPosition[0].x = AprocX;
				tri.texPosition[0].y = AprocY;
				tri.texPosition[1].x = BprocX;
				tri.texPosition[1].y = BprocY;
				tri.texPosition[2].x = CprocX;
				tri.texPosition[2].y = CprocY;

				tri.appearance.imageReference = tex;
			}
		}
		texturesInitialized = true;
	}
}
