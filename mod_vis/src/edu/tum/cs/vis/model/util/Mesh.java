package edu.tum.cs.vis.model.util;

import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;

import javax.imageio.ImageIO;
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
	
	
	/**
	 * Return base path for relative paths of texture images.
	 * @return the base path
	 */
	public String getTextureBasePath() {
		return textureBasePath;
	}

	/**
	 * Set base path for relative paths of texture images.
	 * @param textureBasePath the base path
	 */
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
				for (int v = 0; v < 2; v++) {
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
				for (int v = 0; v < 2; v++) {
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
				for (int v = 0; v < 2; v++) {
					minZ = Math.min(line.position[v].z, minZ);
					maxZ = Math.max(line.position[v].z, maxZ);
				}
			}
		}
		return maxZ - minZ;
	}

	/**
	 * Set minimum x coordinate of this mesh. No coordinate of all its children should be smaller than this value.
	 * Used for drawing a bounding box.
	 * @return minimum x
	 */
	public Float getMinX() {
		return minX;
	}

	/**
	 * Set maximum x coordinate of this mesh. No coordinate of all its children should be bigger than this value.
	 * Used for drawing a bounding box.
	 * @return maximum x
	 */
	public Float getMaxX() {
		return maxX;
	}
	
	/**
	 * Set minimum y coordinate of this mesh. No coordinate of all its children should be smaller than this value.
	 * Used for drawing a bounding box.
	 * @return minimum y
	 */
	public Float getMinY() {
		return minY;
	}

	/**
	 * Set maximum y coordinate of this mesh. No coordinate of all its children should be bigger than this value.
	 * Used for drawing a bounding box.
	 * @return maximum y
	 */
	public Float getMaxY() {
		return maxY;
	}

	/**
	 * Set minimum z coordinate of this mesh. No coordinate of all its children should be smaller than this value.
	 * Used for drawing a bounding box.
	 * @return minimum z
	 */
	public Float getMinZ() {
		return minZ;
	}

	/**
	 * Set maximum z coordinate of this mesh. No coordinate of all its children should be bigger than this value.
	 * Used for drawing a bounding box.
	 * @return maximum z
	 */
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
			if (tri.appearance.imageFileName == null)
				continue;		
			String texfile = getAbsoluteFilePath(textureBasePath,
					tri.appearance.imageFileName);
			if (pictures.get(texfile) == null) {
				BufferedImage bimg = null;
				try {
					bimg = ImageIO.read(new File(texfile));
				} catch (IOException e) {
					System.err.println("Couldn't read file: " + texfile);
					e.printStackTrace();
				}	
				
				// Convert BufferedImage to Image otherwise PImage constructor will fail!!
				
				Image i = bimg.getScaledInstance(bimg.getWidth(), bimg.getHeight(), 0);
				
				PImage pImg = new PImage(i);
				
				pictures.put(texfile, pImg);
			}
		}

		//Now remove offset of texture coordinates because there is a bug with P3D when texture should repeated
		for (Triangle tri : triangles) {
			
			if (tri.appearance.imageFileName == null)
				continue;		
			
			String texfile = getAbsoluteFilePath(textureBasePath,
					tri.appearance.imageFileName);
			//PImage tex = applet.loadImage(texfile);
			PImage tex = pictures.get(texfile);
			
			double xMin = Double.MAX_VALUE;
			double yMin = Double.MAX_VALUE;
			
			for (int i=0; i< 3; i++)
			{

				double x = tri.texPosition[i].x * tex.width;
				double y = tex.height - tri.texPosition[i].y * tex.height;
				tri.texPosition[i].x = (float)x;
				tri.texPosition[i].y = (float)y;
				xMin = Math.min(x, xMin);
				yMin = Math.min(y, yMin);
			}
			
			//Remove offset of texture coordinate if all coordinates are greater than texture
			xMin = Math.floor(xMin/tex.width);
			yMin = Math.floor(yMin/tex.height);
			
			for (int i=0; i< 3; i++)
			{
				tri.texPosition[i].x -= xMin * tex.width;
				tri.texPosition[i].y -= yMin * tex.height;
			}
			

			tri.appearance.imageReference = tex;
		}
		texturesInitialized = true;
	}
	
	/**
	 * Mirrors whole group and children on the x coordinate.by setting each x to the inverse -x
	 */
	public void mirrorX() {


		minX *= (-1);
		float tmp = minX;
		minX = maxX * (-1);
		maxX = tmp;
		for (Triangle tri : triangles) {
			for (int v = 0; v < 3; v++) {
				tri.position[v].x *= (-1);
			}
		}
		for (Line line : lines) {
			for (int v = 0; v < 2; v++) {
				line.position[v].x *= (-1);
			}
		}
	}
}
