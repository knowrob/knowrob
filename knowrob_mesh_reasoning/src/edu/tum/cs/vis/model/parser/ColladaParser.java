/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.parser;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import processing.core.PApplet;

import com.dddviewr.collada.Collada;
import com.dddviewr.collada.Input;
import com.dddviewr.collada.Source;
import com.dddviewr.collada.Unit;
import com.dddviewr.collada.effects.Blinn;
import com.dddviewr.collada.effects.Effect;
import com.dddviewr.collada.effects.EffectAttribute;
import com.dddviewr.collada.effects.Lambert;
import com.dddviewr.collada.geometry.Geometry;
import com.dddviewr.collada.geometry.Lines;
import com.dddviewr.collada.geometry.Mesh;
import com.dddviewr.collada.geometry.Primitives;
import com.dddviewr.collada.geometry.Triangles;
import com.dddviewr.collada.materials.Material;
import com.dddviewr.collada.nodes.Node;
import com.dddviewr.collada.visualscene.BaseXform;
import com.dddviewr.collada.visualscene.InstanceGeometry;
import com.dddviewr.collada.visualscene.InstanceMaterial;
import com.dddviewr.collada.visualscene.InstanceNode;
import com.dddviewr.collada.visualscene.Matrix;
import com.dddviewr.collada.visualscene.VisualScene;

import edu.tum.cs.util.FileUtil;
import edu.tum.cs.util.ResourceRetriever;
import edu.tum.cs.vis.model.util.Appearance;
import edu.tum.cs.vis.model.util.DrawObject;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Polygon;

/**
 * Connection between dae4j library and processing applet. Used to parse COLLADA (.dae) files and
 * draw them on the processing applet. Also .kmz files are supported. These are exctracted to tmp
 * dir and the dae file init will be loaded.
 * 
 * @author Stefan Profanter
 * 
 */
public class ColladaParser extends ModelParser {

	/**
	 * Returns the instantiated Apperance by reading the material information from mat.
	 * 
	 * @param mat
	 *            Collada Material which holds information about the appearance
	 * @param collada
	 *            Collada structure
	 * @return Instance of Apperance set according to mat.
	 */
	private static Appearance getAppearance(Material mat, Collada collada) {
		Appearance appearance = new Appearance();
		Effect effect = collada.findEffect(mat.getInstanceEffect().getUrl().substring(1));
		if (effect.getEffectMaterial() instanceof Lambert) {
			Lambert lambert = (Lambert) effect.getEffectMaterial();
			EffectAttribute diffuse = lambert.getDiffuse();
			String texturePath = null;
			Color materialColor = new Color(0.5f, 0.5f, 0.5f, 1.0f);
			if (diffuse.getTexture() != null) {
				texturePath = collada.findImage(
						effect.findNewParam(
								effect.findNewParam(diffuse.getTexture().getTexture())
										.getSampler2D().getSource()).getSurface().getInitFrom())
						.getInitFrom();
			} else {
				float[] colors = diffuse.getData();
				// System.out.println("Diffuse");
				materialColor = new Color(colors[0], colors[1], colors[2], colors[3]);
			}

			if (texturePath != null) {
				appearance.setImageFileName(texturePath);
			} else {
				// Fill will be set to null if it is a line, Line will be set to null if it is a
				// Triangle
				appearance.setColorFill(materialColor);
				appearance.setColorLine(materialColor);
			}
		} else if (effect.getEffectMaterial() instanceof Blinn) {
			// <emission> + <ambient>*al + <diffuse> + <specular>^<shininess>
			Blinn blinn = (Blinn) effect.getEffectMaterial();

			String texturePath = null;
			if (blinn.getDiffuse().getTexture() != null) {
				texturePath = collada.findImage(
						effect.findNewParam(
								effect.findNewParam(blinn.getDiffuse().getTexture().getTexture())
										.getSampler2D().getSource()).getSurface().getInitFrom())
						.getInitFrom();
				appearance.setImageFileName(texturePath);
			} else {

				float[] emission = blinn.getEmission().getData();
				float[] ambient = blinn.getAmbient().getData();
				float[] diffuse = blinn.getDiffuse().getData();
				float[] specular = blinn.getSpecular().getData();
				float[] shininess = blinn.getShininess().getData();
				float[] result = new float[4];
				for (int i = 0; i < 4; i++) {
					result[i] = 0;
					if (emission != null)
						result[i] += emission[i];
					if (ambient != null)
						result[i] += ambient[i];
					if (diffuse != null)
						result[i] += diffuse[i];
					if (specular != null && shininess != null)
						result[i] += (float) Math.pow(specular[i], shininess[0]);
				}
				// Fill will be set to null if it is a line, Line will be set to null if it is a
				// Triangle
				appearance.setColorFill(new Color(result[0], result[1], result[2], Math.min(
						result[3], 1f)));
				appearance.setColorLine(appearance.getColorFill());
			}

		}
		return appearance;
	}

	/**
	 * Tries to determine the location of the .dae file when a .kmz file was extracted. The path to
	 * the .dae file is stored in the doc.kml file withing the zipped kmz.
	 * 
	 * @param tmpPath
	 *            Path to the extracted .kmz file
	 * @return Absolute path to the .dae file or null if not found.
	 */
	private static String getDaeLocation(String tmpPath) {
		File kmlFile = new File(tmpPath + "doc.kml");
		DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder dBuilder = null;
		try {
			dBuilder = dbFactory.newDocumentBuilder();
		} catch (ParserConfigurationException e) {
			System.err.println("Couldn't get .dae location from doc.kml: " + e.getMessage());
			return null;
		}
		Document doc;
		try {
			doc = dBuilder.parse(kmlFile);
		} catch (SAXException e) {
			System.err.println("Couldn't get .dae location from doc.kml: " + e.getMessage());
			return null;
		} catch (IOException e) {
			System.err.println("Couldn't get .dae location from doc.kml: " + e.getMessage());
			return null;
		}
		doc.getDocumentElement().normalize();

		if (!doc.getDocumentElement().getNodeName().equalsIgnoreCase("kml")) {
			System.out.println("Invalid doc.kml file. Root node must be 'kml'");
			return null;
		}

		LinkedList<String> nodeNames = new LinkedList<String>();
		nodeNames.add("Placemark");
		nodeNames.add("Model");
		nodeNames.add("Link");
		nodeNames.add("href");

		NodeList nList = doc.getElementsByTagName(nodeNames.poll());

		String loc = getNodeValue(nList, nodeNames);
		if (loc == null) {
			System.out.println("Couldn't get .dae location from doc.kml");
			return null;
		}
		return tmpPath + loc;
	}

	/**
	 * Searches in the given nList nodes tree for the node represented by nodeNames and returns the
	 * inner value of this node.
	 * 
	 * @param nList
	 *            the list of all nodes (returned by getElementsByTagName)
	 * @param nodeNames
	 *            A linked list with the structure to search. The first name is the name of the
	 *            root, the next name is the name of the child to search in the root and so on.
	 * @return The inner value of the node or null if not found
	 */
	private static String getNodeValue(NodeList nList, LinkedList<String> nodeNames) {
		if (nodeNames.isEmpty() || nList == null) {
			return null;
		}
		if (nodeNames.size() == 1) {
			for (int temp = 0; temp < nList.getLength(); temp++) {
				org.w3c.dom.Node nNode = nList.item(temp);
				if (nNode.getNodeType() == org.w3c.dom.Node.ELEMENT_NODE) {
					Element element = (Element) nNode;
					NodeList nlList = element.getElementsByTagName(nodeNames.poll());
					if (nlList == null)
						return null;
					nlList = nlList.item(0).getChildNodes();

					org.w3c.dom.Node nValue = nlList.item(0);
					return nValue.getNodeValue();

				}
			}
		} else {
			for (int temp = 0; temp < nList.getLength(); temp++) {

				org.w3c.dom.Node nNode = nList.item(temp);

				if (nNode.getNodeType() == org.w3c.dom.Node.ELEMENT_NODE) {

					Element element = (Element) nNode;

					LinkedList<String> tmpNodeNames = new LinkedList<String>();
					tmpNodeNames.addAll(nodeNames);
					String val = getNodeValue(element.getElementsByTagName(tmpNodeNames.poll()),
							tmpNodeNames);
					if (val != null)
						return val;
				}
			}
		}
		return null;
	}

	/**
	 * Parse a geometry object from the collada structure
	 * 
	 * @param g
	 *            Geometry object to parse
	 * @param instanceMaterial
	 *            List of all known materials which may be referred by the geometry object
	 * @param transformations
	 *            List of all parent transformations which should be applied to this and it's
	 *            subsequent geometries
	 * @param collada
	 *            the collada Structure
	 */
	private static void parseGeometry(Geometry g, Group currGroup,
			HashMap<String, String> instanceMaterial, ArrayList<BaseXform> transformations,
			Collada collada) {
		Mesh m = g.getMesh();
		for (Primitives p : m.getPrimitives()) {
			if (p instanceof Triangles) {
				Triangles t = (Triangles) p;
				parseGeometryTriangle(t, m, currGroup, instanceMaterial, transformations, collada);
			} else if (p instanceof Lines) {
				Lines l = (Lines) p;
				parseGeometryLine(l, m, currGroup, instanceMaterial, transformations, collada);
			}
		}
	}

	/**
	 * Parse a line from a geometry object from the collada structure.
	 * 
	 * @param l
	 *            the line object to parse
	 * @param m
	 *            parent mesh for the line
	 * @param instanceMaterial
	 *            List of all known materials which may be referred by the geometry object
	 * @param transformations
	 *            List of all parent transformations which should be applied to this and it's
	 *            subsequent geometries
	 * @param collada
	 *            the collada Structure
	 */
	private static void parseGeometryLine(Lines l, Mesh m, Group currGroup,
			HashMap<String, String> instanceMaterial, ArrayList<BaseXform> transformations,
			Collada collada) {
		Material mat = null;
		if (instanceMaterial.containsKey(l.getMaterial())) {
			mat = collada.findMaterial(instanceMaterial.get(l.getMaterial()));

		} else {
			l.dump(System.err, 5);
			throw new RuntimeException("No material given for Line (see above) ");
		}

		Source vertexSource = null;
		for (Input in : m.getVertices().getInputs()) {
			if (in.getSemantic().compareTo("POSITION") == 0) {
				vertexSource = m.getSource(in.getSource().substring(1));
			}
		}
		// commons for the set of lines
		float[][] vertPoints = SourceToFloat(vertexSource);

		int stride = l.getInputs().size();
		int count = l.getCount();

		int[][] indexes = new int[count * 3][stride];

		for (int i = 0, k = 0; i < count * 3; i++) {
			for (int j = 0; j < stride; j++, k++)
				indexes[i][j] = l.getData()[k];
		}

		int vertexOffset = 0;
		for (Input i : l.getInputs()) {
			if (i.getSemantic().compareTo("VERTEX") == 0)
				vertexOffset = i.getOffset();

		}

		Appearance appearance = getAppearance(mat, collada);
		appearance.setColorFill(null);

		// one single line from the line-set
		for (int i = 0; i < indexes.length && i + 2 < indexes.length; i++) {
			Line line = new Line();
			line.setAppearance(appearance);

			for (int v = 0; v < 2; v++) {
				// set the vertices for Point
				line.getPosition()[v] = new Point3f(vertPoints[indexes[i][vertexOffset]][0],
						vertPoints[indexes[i][vertexOffset]][1],
						vertPoints[indexes[i][vertexOffset]][2]);

				i++;
			}
			line.updateNormalVector();
			i -= 2;

			useTransformation(line, transformations);
			// add it to the Collection
			currGroup.getMesh().getLines().add(line);
		}
	}

	/**
	 * Parse a triangle from a geometry object from the collada structure.
	 * 
	 * @param t
	 *            the line object to parse
	 * @param m
	 *            parent mesh for the line
	 * @param instanceMaterial
	 *            List of all known materials which may be referred by the geometry object
	 * @param transformations
	 *            List of all parent transformations which should be applied to this and it's
	 *            subsequent geometries
	 * @param collada
	 *            the collada Structure
	 */
	private static void parseGeometryTriangle(Triangles t, Mesh m, Group currGroup,
			HashMap<String, String> instanceMaterial, ArrayList<BaseXform> transformations,
			Collada collada) {
		Material mat = null;
		if (instanceMaterial.containsKey(t.getMaterial())) {
			mat = collada.findMaterial(instanceMaterial.get(t.getMaterial()));

		} else {

			t.dump(System.err, 5);
			throw new RuntimeException("No material given for Triangle (see above) ");
		}

		Source vertexSource = null;
		for (Input in : m.getVertices().getInputs()) {
			if (in.getSemantic().compareTo("POSITION") == 0) {
				vertexSource = m.getSource(in.getSource().substring(1));
			}
		}

		// commons for the set of polygons
		float[][] vertPoints = SourceToFloat(vertexSource);

		int stride = t.getInputs().size();
		int count = t.getCount();

		int[][] indexes = new int[count * 3][stride];

		for (int i = 0, k = 0; i < count * 3; i++) {
			for (int j = 0; j < stride; j++, k++)
				indexes[i][j] = t.getData()[k];
		}

		int vertexOffset = 0, textureOffset = 0;
		Source textureSource = null;
		for (Input i : t.getInputs()) {
			if (i.getSemantic().equals("VERTEX"))
				vertexOffset = i.getOffset();
			else if (i.getSemantic().equals("TEXCOORD")) {
				textureOffset = i.getOffset();
				textureSource = m.getSource(i.getSource().substring(1));
			}

		}

		float[][] texturePoints = null; // dummy init
		if (textureSource != null) {
			// Triangle has texture
			texturePoints = SourceToFloat(textureSource);
		}

		Appearance appearance = getAppearance(mat, collada);
		appearance.setColorLine(null);

		// one single triangle from the triangle-set
		for (int i = 0; i < indexes.length; i++) {
			Polygon tri = new Polygon(3);
			tri.setAppearance(appearance);

			if (tri.getAppearance().getImageFileName() != null)
				tri.setTexPosition(new Point2f[3]);

			for (int v = 0; v < 3; v++) {
				// set the vertices for Point
				tri.getPosition()[v] = new Point3f(vertPoints[indexes[i][vertexOffset]][0],
						vertPoints[indexes[i][vertexOffset]][1],
						vertPoints[indexes[i][vertexOffset]][2]);

				// set texture position
				if (tri.getAppearance().getImageFileName() != null) {
					tri.getTexPosition()[v] = new Point2f(
							texturePoints[indexes[i][textureOffset]][0],
							texturePoints[indexes[i][textureOffset]][1]);
				}

				i++;
			}
			i--;

			useTransformation(tri, transformations);
			tri.updateNormalVector();

			// add it to the Collection
			currGroup.getMesh().getPolygons().add(tri);
		}

	}

	/**
	 * Convert a collada source item to the corresponding float array
	 * 
	 * @param the
	 *            collada source
	 * @return float array with float[count][stride]
	 */
	private static float[][] SourceToFloat(Source s) {
		float ret[][] = new float[s.getAccessor().getCount()][s.getAccessor().getStride()];
		for (int i = 0, k = 0; i < s.getAccessor().getCount(); i++) {
			for (int j = 0; j < s.getAccessor().getStride(); j++, k++)
				ret[i][j] = s.getFloatArray().get(k);
		}
		return ret;
	}

	/**
	 * Applies the list of transformations in the correct order to the DrawObject (Line / Triangle)
	 * 
	 * @param obj
	 *            Object to apply the transformation onto
	 * @param transformations
	 *            list of transformations to apply
	 */
	private static void useTransformation(DrawObject obj, ArrayList<BaseXform> transformations) {
		for (int idx = transformations.size() - 1; idx >= 0; idx--) {
			BaseXform transform = transformations.get(idx);
			if (transform instanceof Matrix) {
				float matrix[][] = new float[4][4];
				int v = 0;
				for (int ma_y = 0; ma_y < 4; ma_y++) {
					for (int ma_x = 0; ma_x < 4; ma_x++) {
						matrix[ma_y][ma_x] = ((Matrix) transform).getData()[v];
						v++;
					}
				}
				obj.transform(matrix);
			} else {
				System.out.println("Transformation not implemented: ");
				transform.dump(System.out, 4);
			}
		}
	}

	/**
	 * Base path of the texture files. In the dae file are only relative paths. So we need a base.
	 * If kmz is parsed, this will be set automatically
	 */
	private String	textureBasePath	= "";

	@Override
	public void draw(PApplet applet, Color overrideColor) {
		if (group == null)
			return;
		applet.noStroke();

		if (overrideColor != null)
			applet.fill(overrideColor.getRed(), overrideColor.getGreen(), overrideColor.getBlue(),
					overrideColor.getAlpha());
		else
			applet.fill(127); // set a default color value

		group.draw(applet.g, overrideColor);
	}

	@Override
	protected boolean loadModel(String file) {
		Collada collada = null;
		textureBasePath = null;
		String daeFile = null;

		if (!checkExtension(file)) {
			return false;
		}

		String filename = ResourceRetriever.retrieve(file).getAbsolutePath();

		if ((new File(filename)).exists() == false) {
			System.err.println("ERROR: Can't load model. File not found: " + filename + "\n");
			return false;
		}

		String extension = getExtension(filename);
		if (extension.equalsIgnoreCase("kmz")) {
			String tmpPath = ResourceRetriever.createTempDirectory();
			if (!tmpPath.endsWith("/") && !tmpPath.endsWith("\\"))
				tmpPath += "/";
			FileUtil.Unzip(filename, tmpPath);

			daeFile = getDaeLocation(tmpPath);

			textureBasePath = daeFile.substring(0, daeFile.lastIndexOf(File.separator));

			if (!textureBasePath.endsWith("/") && !textureBasePath.endsWith("\\"))
				textureBasePath += "/";

		} else if (extension.equalsIgnoreCase("dae")) {
			textureBasePath = "";
			daeFile = filename;
		}

		try {
			collada = Collada.readFile(daeFile);
		} catch (FileNotFoundException e) {
			System.err.println("File doesn't exists: " + daeFile);
			e.printStackTrace();
			return false;
		} catch (SAXException e) {
			System.err.println("XML-Exception in : " + daeFile);
			e.printStackTrace();
			return false;
		} catch (IOException e) {
			System.err.println("IO-Exception in : " + daeFile);
			e.printStackTrace();
			return false;
		}

		group = new Group();

		// collada.dump(System.out, 0);

		// This is where everything begins
		VisualScene scene = collada.getLibraryVisualScenes().getScene(
				collada.getScene().getInstanceVisualScene().getUrl());

		for (Node n : scene.getNodes()) {
			parseNode(n, group, new ArrayList<BaseXform>(), collada);
		}

		group.initialize(textureBasePath);

		Unit unit = collada.getUnit();
		if (unit != null && unit.getMeter() != 1.0) {
			group.scale(unit.getMeter());
		}

		group.mirrorX();

		return true;
	}

	/**
	 * Parses a node from the collada structure and stores the polygons and lines information.
	 * 
	 * @param node
	 *            Node to parse (may have other sub-Nodes => called recursively)
	 * @param transformations
	 *            List of all parent transformations which should be applied to this and it's
	 *            subsequent nodes
	 * @param collada
	 *            the collada Structure
	 */
	private void parseNode(Node node, Group currGroup, ArrayList<BaseXform> transformations,
			Collada collada) {

		// Add transformations like a stack
		if (node.getXforms() != null)
			transformations.addAll(node.getXforms());

		// Parse all child nodes
		if (node.getChildNodes() != null) {
			for (Node child : node.getChildNodes()) {
				Group g = new Group();
				g.setName(child.getName());
				currGroup.addChild(g);
				parseNode(child, g, transformations, collada);
			}
		}

		// Parse referencing nodes
		InstanceNode instNode = node.getInstanceNode();
		if (instNode != null) {
			parseNode(collada.findNode(instNode.getUrl()), currGroup, transformations, collada);
		}

		// Parse referencing geometries and Material
		List<InstanceGeometry> instGeoList = node.getInstanceGeometry();

		for (InstanceGeometry instGeo : instGeoList) {
			HashMap<String, String> instanceMaterial = new HashMap<String, String>();
			for (InstanceMaterial mat : instGeo.getInstanceMaterials()) {
				instanceMaterial.put(mat.getSymbol(), mat.getTarget());
			}
			parseGeometry(collada.findGeometry(instGeo.getUrl()), currGroup, instanceMaterial,
					transformations, collada);
		}

		// remove added transformations
		if (node.getXforms() != null)
			transformations.removeAll(node.getXforms());
	}
}
