package edu.tum.cs.vis.model.util;

import java.awt.Color;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

import javax.imageio.ImageIO;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import processing.core.PApplet;
import processing.core.PGraphics;
import processing.core.PImage;
import edu.tum.cs.util.FileUtil;

/**
 * A Mesh contains list of Triangles and Lines which represent an Object.
 * 
 * @see Triangle
 * @see Line
 * 
 * @author Stefan Profanter
 * 
 */
public class Mesh implements Serializable {

	/**
	 * auto generated
	 */
	private static final long	serialVersionUID	= -2761386921853967131L;

	/**
	 * Minimum x-position of model
	 */
	protected Float				minX				= null;
	/**
	 * Maximum x-position of model
	 */
	protected Float				maxX				= null;
	/**
	 * Minimum y-position of model
	 */
	protected Float				minY				= null;
	/**
	 * Maximum y-position of model
	 */
	protected Float				maxY				= null;
	/**
	 * Minimum z-position of model
	 */
	protected Float				minZ				= null;
	/**
	 * Maximum z-position of model
	 */
	protected Float				maxZ				= null;
	/**
	 * Is false until <code>setTextureImage</code> is called and the textures of all polygons are
	 * initialized.
	 */
	private boolean				texturesInitialized	= false;

	/**
	 * Base path for relative paths in texture filenames.
	 */
	private String				textureBasePath;
	/**
	 * List of all polygons parsed from file
	 */
	private ArrayList<Polygon>	polygons			= new ArrayList<Polygon>();

	/**
	 * List of all lines parsed from file
	 */
	private ArrayList<Line>		lines				= new ArrayList<Line>();

	/**
	 * Draws the bounding box around the model with the current style
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void drawBoundingBox(PApplet applet) {
		if (polygons.size() == 0 && lines.size() == 0)
			return;
		// Save current translation
		applet.pushMatrix();
		applet.translate(maxX - getWidth() / 2f, maxY - getHeight() / 2f, maxZ - getDepth() / 2f);
		applet.box(getWidth(), getHeight(), getDepth());

		// Restore last translation
		applet.popMatrix();
	}

	/**
	 * Draw the lines list to the applet
	 * 
	 * @param g
	 *            Applet to draw on
	 * @param overrideColor
	 *            override the draw color an texture. Draw whole object in the given color if !=
	 *            null
	 */
	public void drawLines(PGraphics g, Color overrideColor) {
		for (Line line : lines) {

			line.draw(g, overrideColor);
		}
	}

	/**
	 * Draw the polygons list to the applet
	 * 
	 * @param applet
	 *            Applet to draw on
	 * @param overrideColor
	 *            override the draw color an texture. Draw whole object in the given color if !=
	 *            null
	 */
	public void drawPolygons(PGraphics g, Color overrideColor) {
		if (!texturesInitialized)
			setTextureImage();
		for (Polygon tri : polygons) {
			tri.draw(g, overrideColor);
		}
	}

	/**
	 * Returns the depth of the model by searching the biggest distance on the z-axis between the
	 * vectors.
	 * 
	 * @return float as depth of the model
	 */
	public float getDepth() {
		if (minZ != null && maxZ != null)
			return Math.abs(maxZ - minZ);
		if (polygons.size() == 0) {
			minZ = 0f;
			maxZ = 0f;
		} else {
			minZ = Float.MAX_VALUE;
			maxZ = Float.MIN_VALUE;

			for (Polygon tri : polygons) {
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
	 * Returns the height of the model by searching the biggest distance on the y-axis between the
	 * vectors.
	 * 
	 * @return float as height of the model
	 */
	public float getHeight() {
		if (minY != null && maxY != null)
			return Math.abs(maxY - minY);
		if (polygons.size() == 0) {
			minY = 0f;
			maxY = 0f;
		} else {
			minY = Float.MAX_VALUE;
			maxY = Float.MIN_VALUE;

			for (Polygon tri : polygons) {
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
	 * Returns a new empty mesh where textureInitialized and textureBasePath are set to the values
	 * of this mesh.
	 * 
	 * @return a new mesh
	 */
	public Mesh getInitializedChildMesh() {
		Mesh m = new Mesh();
		m.textureBasePath = textureBasePath;
		m.texturesInitialized = texturesInitialized;
		return m;
	}

	public void getIntersectedPolygons(Point3f rayStart, Point3f rayEnd,
			ArrayList<Polygon> intersectedPolygons, Point3f intersect) {

		for (Polygon tri : polygons) {
			if (tri.intersectsRay(rayStart, rayEnd, intersect))
				intersectedPolygons.add(tri);
		}

	}

	/**
	 * @return the lines
	 */
	public ArrayList<Line> getLines() {
		return lines;
	}

	/**
	 * Set maximum x coordinate of this mesh. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum x
	 */
	public Float getMaxX() {
		return maxX;
	}

	/**
	 * Set maximum y coordinate of this mesh. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum y
	 */
	public Float getMaxY() {
		return maxY;
	}

	/**
	 * Set maximum z coordinate of this mesh. No coordinate of all its children should be bigger
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return maximum z
	 */
	public Float getMaxZ() {
		return maxZ;
	}

	/**
	 * Set minimum x coordinate of this mesh. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum x
	 */
	public Float getMinX() {
		return minX;
	}

	/**
	 * Set minimum y coordinate of this mesh. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum y
	 */
	public Float getMinY() {
		return minY;
	}

	/**
	 * Set minimum z coordinate of this mesh. No coordinate of all its children should be smaller
	 * than this value. Used for drawing a bounding box.
	 * 
	 * @return minimum z
	 */
	public Float getMinZ() {
		return minZ;
	}

	/**
	 * @return the polygons
	 */
	public ArrayList<Polygon> getPolygons() {
		return polygons;
	}

	/**
	 * Return base path for relative paths of texture images.
	 * 
	 * @return the base path
	 */
	public String getTextureBasePath() {
		return textureBasePath;
	}

	/**
	 * Returns the width of the model by searching the biggest distance on the x-axis between the
	 * vectors.
	 * 
	 * @return float as width of the model
	 */
	public float getWidth() {
		if (minX != null && maxX != null)
			return Math.abs(maxX - minX);
		if (polygons.size() == 0) {
			minX = 0f;
			maxX = 0f;
		} else {
			minX = Float.MAX_VALUE;
			maxX = Float.MIN_VALUE;

			for (Polygon tri : polygons) {
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
	 * Mirrors whole group and children on the x coordinate.by setting each x to the inverse -x
	 */
	public void mirrorX() {

		minX *= (-1);
		float tmp = minX;
		minX = maxX * (-1);
		maxX = tmp;
		for (Polygon tri : polygons) {
			for (int v = 0; v < 3; v++) {
				tri.position[v].x *= (-1);
			}
			tri.updateNormalVector();
		}
		for (Line line : lines) {
			for (int v = 0; v < 2; v++) {
				line.position[v].x *= (-1);
			}
		}
	}

	/**
	 * Scales all coordinates by the given factor
	 * 
	 * @param factor
	 *            The scale factor
	 */
	protected void scaleMesh(float factor) {
		for (Polygon tri : polygons) {

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
	 * @param lines
	 *            the lines to set
	 */
	public void setLines(ArrayList<Line> lines) {
		this.lines = lines;
	}

	/**
	 * @param polygons
	 *            the polygons to set
	 */
	public void setPolygons(ArrayList<Polygon> polygons) {
		this.polygons = polygons;
	}

	/**
	 * Set base path for relative paths of texture images.
	 * 
	 * @param textureBasePath
	 *            the base path
	 */
	public void setTextureBasePath(String textureBasePath) {
		this.textureBasePath = textureBasePath;
	}

	/**
	 * Only used for initialization! per definition is: textureProcessing x = textureSketchup x *
	 * picture.width textureProcessing y = picture.height - textureSketchup y * picture.height it
	 * also creates an PImage to each Triangle (if it contains any Texture)
	 */
	private void setTextureImage() {
		// load all Texture-Images only once (memory efficiency)
		HashMap<String, PImage> pictures = new HashMap<String, PImage>();
		for (Polygon tri : polygons) {
			if (tri.appearance.getImageFileName() == null)
				continue;
			String texfile = FileUtil.getAbsoluteFilePath(textureBasePath,
					tri.appearance.getImageFileName());
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

		// Now remove offset of texture coordinates because there is a bug with P3D when texture
		// should repeated
		for (Polygon tri : polygons) {

			if (tri.appearance.getImageFileName() == null)
				continue;

			String texfile = FileUtil.getAbsoluteFilePath(textureBasePath,
					tri.appearance.getImageFileName());
			// PImage tex = applet.loadImage(texfile);
			PImage tex = pictures.get(texfile);

			double xMin = Double.MAX_VALUE;
			double yMin = Double.MAX_VALUE;

			for (int i = 0; i < 3; i++) {

				double x = tri.texPosition[i].x * tex.width;
				double y = tex.height - tri.texPosition[i].y * tex.height;
				tri.texPosition[i].x = (float) x;
				tri.texPosition[i].y = (float) y;
				xMin = Math.min(x, xMin);
				yMin = Math.min(y, yMin);
			}

			// Remove offset of texture coordinate if all coordinates are greater than texture
			xMin = Math.floor(xMin / tex.width);
			yMin = Math.floor(yMin / tex.height);

			for (int i = 0; i < 3; i++) {
				tri.texPosition[i].x -= xMin * tex.width;
				tri.texPosition[i].y -= yMin * tex.height;
			}

			tri.appearance.setImageReference(tex);
		}
		texturesInitialized = true;
	}

	/**
	 * Translate (move) the model by the given translation vector
	 * 
	 * @param translation
	 *            translation vector
	 */
	public void translate(Vector3f translation) {
		minX += translation.x;
		maxX += translation.x;
		minY += translation.y;
		maxY += translation.y;
		minZ += translation.z;
		maxZ += translation.z;
		for (Polygon tri : polygons) {

			tri.translate(translation.x, translation.y, translation.z);
		}
		for (Line line : lines) {

			line.translate(translation.x, translation.y, translation.z);
		}
	}

}
