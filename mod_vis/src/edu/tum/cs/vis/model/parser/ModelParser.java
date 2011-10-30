package edu.tum.cs.vis.model.parser;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import javax.vecmath.Point3f;

import processing.core.PApplet;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Base class for all ModelParsers. Used to parse models from file an draw them onto the Processing applet.
 * 
 * @author Stefan Profanter
 *
 */
public abstract class ModelParser {

	/**
	 * List of all triangles parsed from file
	 */
	protected LinkedList<Triangle> triangles = new LinkedList<Triangle>();
	/**
	 * List of all lines parsed from file
	 */
	protected LinkedList<Line> lines = new LinkedList<Line>();

	/**
	 * Minimum position values of model position
	 */
	protected Float minX = null;
	protected Float maxX = null;
	protected Float minY = null;
	protected Float maxY = null;
	protected Float minZ = null;
	protected Float maxZ = null;
	
	/**
	 * Current position (volumetric center) of model.
	 * If model is centered, it is (0|0|0)
	 */
	private Point3f currentPosition = null;
	
	/**
	 * Contains for each valid file extension the appropriate parser.
	 * Used to automatically determine the correct parser for a file.
	 */
	private static final HashMap<String,Class<? extends ModelParser>> extensionAssignment = new HashMap<String,Class<? extends ModelParser>>();
    static {
    	/*
    	 * Add here other parsers with file extension
    	 */
    	extensionAssignment.put("dae", ColladaParser.class);
    	extensionAssignment.put("kmz", ColladaParser.class);
    }
    
    /**
     * Gets the filename extension of a filename. Usually the last 3 characters
     * @param filename The filename
     * @return the extension without dot
     */
    protected static String getExtension(String filename)
    {
    	return filename.substring(filename.lastIndexOf('.') + 1);
    }
    
    /**
     * Checks if the given filename has a valid extension for the extended class.
     * @param filename Filename to check
     * @return true if extension is valid
     */
    protected boolean isValidExtension(String filename)
    {
    	if (extensionAssignment.get(getExtension(filename)) == this.getClass())
    		return true;
    	return false;
    }
    
    /**
     * Checks if the file extension is valid for this class
     * @param filename file to check
     */
    protected boolean checkExtension(String filename) {
    	if (!isValidExtension(filename))
    	{
    		System.out.println("Unknown file extension for class: " + this.getClass().getName() + " " + filename + "\n Must be one of: " + getValidExtensions());
    		return false;
    	} else {
    		return true;
    	}
    }
    
    
    /**
     * Returns a komma separated string with all valid file extensions of this class
     * @return List of file extensions: eg "kmz, dae,"
     */
    protected String getValidExtensions() {
    	Iterator<String> it = extensionAssignment.keySet().iterator();
    	String extensions = "";
    	while(it.hasNext()) {
    		String key = it.next();
    		Object val = extensionAssignment.get(key);
    		if (val.getClass() == this.getClass())
    			extensions += key + ", "; 
    	}
    	return extensions;
    }

	/**
	 * Draw the triangles list to the applet
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	protected void drawTriangles(PApplet applet) {

		// Shapes are not effected by translate
		for (Triangle tri : triangles) {

			tri.draw(applet);
		}
	}

	/**
	 * Draw the lines list to the applet
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	protected void drawLines(PApplet applet) {

		// Shapes are not effected by translate
		for (Line line : lines) {

			line.draw(applet);
		}
	}

	/**
	 * Draws the bounding box around the model with the current style
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void drawBoundingBox(PApplet applet) {
		// Save current translation
		applet.pushMatrix();
		applet.translate(currentPosition.x, currentPosition.y,
				currentPosition.z);
		applet.box(getModelWidth(), getModelHeight(), getModelDepth());

		// Restore last translation
		applet.popMatrix();
	}

	/**
	 * Moves the model so that the center of the bounding box is the point and
	 * further calls to setModelPosition result in the correct position (0,0,0)
	 */
	protected void centerModel() {
		// Initialize min and max variables
		getModelWidth();
		getModelHeight();
		getModelDepth();
		currentPosition = new Point3f(minX + (getModelWidth() / 2), minY
				+ (getModelHeight() / 2), minZ + (getModelDepth() / 2));
		setModelPosition(new Point3f(0, 0, 0));
	}

	/**
	 * Scales all coordinates by the given factor
	 * 
	 * @param factor
	 *            The scale factor
	 */
	protected void scaleModel(float factor) {
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
	 * Set absolute position of the model. Only used for centering model.
	 * If you want to move the model when drawing, use transformation matrix.
	 * @param pos new absolute position of model
	 */
	protected void setModelPosition(Point3f pos) {
		if (currentPosition == null)
			centerModel();
		for (Triangle tri : triangles) {

			tri.translate(pos.x - currentPosition.x, pos.y - currentPosition.y,
					pos.z - currentPosition.z);
		}
		for (Line line : lines) {

			line.translate(pos.x - currentPosition.x,
					pos.y - currentPosition.y, pos.z - currentPosition.z);
		}
		currentPosition = pos;
	}

	/**
	 * Draw method to draw the model on the applet.
	 * @param applet The applet to draw on.
	 */
	public abstract void draw(PApplet applet);

	/**
	 * Main function to parse the model from the given filename
	 * @param filename Physical file of model to parse
	 */
	public abstract boolean loadModel(String filename);
	
	/**
	 * Finds the appropriate parser for the given filename by comaring the file extension.
	 * @param filename Filename to find parser for
	 * @return the Parser Class for this file
	 */
	public static Class<? extends ModelParser> findParser(String filename)
	{
		return extensionAssignment.get(getExtension(filename));
	}
	
	/**
	 * Returns the height of the model by searching the biggest distance on the
	 * y-axis between the vectors.
	 * 
	 * @return float as height of the model
	 */
	public float getModelHeight() {
		if (minY != null && maxY != null)
			return Math.abs(maxY - minY);
		minY = Float.MAX_VALUE;
		maxY = Float.MIN_VALUE;

		for (Triangle tri : triangles) {
			for (int v = 0; v < 3; v++) {
				minY = Math.min(tri.position[v].y, minY);
				maxY = Math.max(tri.position[v].y, maxY);
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
	public float getModelWidth() {
		if (minX != null && maxX != null)
			return Math.abs(maxX - minX);
		minX = Float.MAX_VALUE;
		maxX = Float.MIN_VALUE;

		for (Triangle tri : triangles) {
			for (int v = 0; v < 3; v++) {
				minX = Math.min(tri.position[v].x, minX);
				maxX = Math.max(tri.position[v].x, maxX);
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
	public float getModelDepth() {
		if (minZ != null && maxZ != null)
			return Math.abs(maxZ - minZ);
		minZ = Float.MAX_VALUE;
		maxZ = Float.MIN_VALUE;

		for (Triangle tri : triangles) {
			for (int v = 0; v < 3; v++) {
				minZ = Math.min(tri.position[v].z, minZ);
				maxZ = Math.max(tri.position[v].z, maxZ);
			}
		}
		return maxZ - minZ;
	}

	/**
	 * Unzip a zipped file (eg. kmz) into given directory 
	 * @param zipFile zipped file to unzip
	 * @param outputDirectory destination directory for unzipped content
	 * @return true if successfully unzipped
	 */
	public static boolean Unzip(String zipFile, String outputDirectory) {
		if (!outputDirectory.endsWith("/") && !outputDirectory.endsWith("\\"))
			outputDirectory += "/";

		BufferedOutputStream dest = null;
		BufferedInputStream is = null;
		int BUFFER = 2048;
		ZipEntry entry;
		ZipFile zipfile;
		try {
			zipfile = new ZipFile(zipFile);
			Enumeration<? extends ZipEntry> e = zipfile.entries();
			while (e.hasMoreElements()) {
				entry = (ZipEntry) e.nextElement();
				if (entry.isDirectory()) {
					(new File(outputDirectory + entry.getName())).mkdir();
					continue;
				}

				String filename = outputDirectory + entry.getName();
				String filePath = filename.substring(0,
						filename.lastIndexOf(File.separator));

				// Create directory if not existing
				if (!(new File(filePath)).exists()) {
					(new File(filePath)).mkdirs();
				}
				is = new BufferedInputStream(zipfile.getInputStream(entry));
				int count;
				byte data[] = new byte[BUFFER];
				FileOutputStream fos = new FileOutputStream(filename);
				dest = new BufferedOutputStream(fos, BUFFER);
				while ((count = is.read(data, 0, BUFFER)) != -1) {
					dest.write(data, 0, count);
				}
				dest.flush();
				dest.close();
				is.close();
			}
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			System.err.println("Couldn't unzip file: " + zipFile);
			e1.printStackTrace();
			return false;
		}
		return true;
	}

	/**
	 * creates a temporary directory used for unzipping
	 * @return File with path to created dir
	 * @throws IOException if not successfully created
	 */
	public static File createTempDirectory() throws IOException {
		final File temp;

		temp = File.createTempFile("temp", Long.toString(System.nanoTime()));

		if (!(temp.delete())) {
			throw new IOException("Could not delete temp file: "
					+ temp.getAbsolutePath());
		}

		if (!(temp.mkdir())) {
			throw new IOException("Could not create temp directory: "
					+ temp.getAbsolutePath());
		}

		return (temp);
	}

}
