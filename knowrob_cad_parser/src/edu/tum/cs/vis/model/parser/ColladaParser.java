/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
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
import java.util.Map;
import java.util.regex.Matcher;

import javax.vecmath.Point2f;
import javax.vecmath.Vector3f;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

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

import edu.tum.cs.ias.knowrob.utils.FileUtil;
import edu.tum.cs.ias.knowrob.utils.ResourceRetriever;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.util.Appearance;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Line;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

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
				if (texturePath.startsWith("./images/"))
					texturePath = "." + texturePath;
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
				if (texturePath.startsWith("./images/"))
					texturePath = "." + texturePath;
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

		String ret = tmpPath + loc;
		if (File.separator.equals("\\")) {
			ret = ret.replaceAll("/", Matcher.quoteReplacement(File.separator));
		} else {
			ret = ret.replaceAll("\\\\", Matcher.quoteReplacement(File.separator));
		}
		return ret;
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
	 * Get all vertices of mesh object <tt>m</tt> in an array.
	 * 
	 * @param m
	 *            the mesh object
	 * @param inputs
	 *            list of all vertices
	 * @param type
	 *            type of vertices
	 * @return list of all vertices which match given constraints.
	 */
	private static ArrayList<Vertex> getVerticesOfMesh(Mesh m, List<Input> inputs, String type) {
		Source vertexSource = null;
		for (Input in : inputs) {
			if (in.getSemantic().compareTo(type) == 0) {
				vertexSource = m.getSource(in.getSource().substring(1));
			}
		}

		if (vertexSource == null) {
			return null;
		}

		float[][] vertPoints = SourceToFloat(vertexSource);

		ArrayList<Vertex> vertices = new ArrayList<Vertex>();
		for (int i = 0; i < vertPoints.length; i++) {
			vertices.add(new Vertex(vertPoints[i][0], vertPoints[i][1], vertPoints[i][2]));
		}

		return vertices;
	}

	/**
	 * Parse a geometry object from the collada structure
	 * 
	 * @param g
	 *            Geometry object to parse
	 * @param currGroup
	 *            group of which the current geometry is part of. Used to add all found vertices to
	 *            the group's vertices list.
	 * @param instanceMaterial
	 *            List of all known materials which may be referred by the geometry object
	 * @param transformations
	 *            List of all parent transformations which should be applied to this and it's
	 *            subsequent geometries
	 * @param collada
	 *            the collada Structure
	 */
	private static void parseGeometry(Geometry g, Group currGroup,
			Map<String, String> instanceMaterial, List<BaseXform> transformations, Collada collada) {
		Mesh m = g.getMesh();
		ArrayList<Vertex> vertices = getVerticesOfMesh(m, m.getVertices().getInputs(), "POSITION");
		ArrayList<Vertex> normals = getVerticesOfMesh(m, m.getVertices().getInputs(), "NORMAL");

		for (int i = 0; i < vertices.size(); i++) {
			Vertex v = vertices.get(i);
			useTransformation(v, transformations, true);
			if (normals != null) {
				Vertex n = normals.get(i);
				useTransformation(n, transformations, false);
				v.setNormalVector(new Vector3f(n.x, n.y, n.z));
			}
		}

		currGroup.getModel().getVertices().addAll(vertices);

		for (Primitives p : m.getPrimitives()) {
			if (p instanceof Triangles) {
				Triangles t = (Triangles) p;
				parseGeometryTriangle(t, currGroup, instanceMaterial, collada, vertices, m);
			} else if (p instanceof Lines) {
				Lines l = (Lines) p;
				parseGeometryLine(l, currGroup, instanceMaterial, collada, vertices);
			}
		}
	}

	/**
	 * Parse a line from a geometry object from the collada structure.
	 * 
	 * @param l
	 *            the line object to parse
	 * @param currGroup
	 *            group of which the current geometry is part of. Used to add all found vertices to
	 *            the group's vertices list.
	 * @param instanceMaterial
	 *            List of all known materials which may be referred by the geometry object
	 * @param collada
	 *            the collada Structure
	 * @param vertices
	 *            List of vertices of the line.
	 */
	private static void parseGeometryLine(Lines l, Group currGroup,
			Map<String, String> instanceMaterial, Collada collada, List<Vertex> vertices) {
		Material mat = null;
		if (instanceMaterial.containsKey(l.getMaterial())) {
			mat = collada.findMaterial(instanceMaterial.get(l.getMaterial()));

		} else {
			l.dump(System.err, 5);
			throw new RuntimeException("No material given for Line (see above) ");
		}

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
				line.getPosition()[v] = vertices.get(indexes[i][vertexOffset]);

				i++;
			}
			line.updateCentroid();
			i -= 2;

			// useTransformation(line, transformations);
			// add it to the Collection
			currGroup.getMesh().getLines().add(line);
			currGroup.getModel().getLines().add(line);
		}
	}

	/**
	 * Parse a triangle from a geometry object from the collada structure.
	 * 
	 * @param t
	 *            the line object to parse
	 * @param m
	 *            parent mesh for the triangle
	 * @param instanceMaterial
	 *            List of all known materials which may be referred by the geometry object
	 * @param collada
	 *            the collada Structure
	 * @param vertices
	 *            List of vertices of the line.
	 * @param currGroup
	 *            group of which the current geometry is part of. Used to add all found vertices to
	 *            the group's vertices list.
	 */
	private static void parseGeometryTriangle(Triangles t, Group currGroup,
			Map<String, String> instanceMaterial, Collada collada, List<Vertex> vertices, Mesh m) {
		Appearance appearance;
		if (instanceMaterial.containsKey(t.getMaterial())) {
			Material mat = collada.findMaterial(instanceMaterial.get(t.getMaterial()));
			appearance = getAppearance(mat, collada);
			appearance.setColorLine(null);

		} else {
			// Triangle has no material defined.
			appearance = new Appearance();
			appearance.setColorFill(new Color(120, 120, 120));
		}

		int stride = t.getInputs().size();
		int count = t.getCount();

		int[][] indexes = new int[count * 3][stride];

		for (int i = 0, k = 0; i < count * 3; i++) {
			for (int j = 0; j < stride; j++, k++)
				indexes[i][j] = t.getData()[k];
		}

		int vertexOffset = 0, textureOffset = 0, normalOffset = 0;
		Source textureSource = null;
		Source normalSource = null;
		for (Input i : t.getInputs()) {
			if (i.getSemantic().equals("VERTEX"))
				vertexOffset = i.getOffset();
			else if (i.getSemantic().equals("TEXCOORD")) {
				textureOffset = i.getOffset();
				textureSource = m.getSource(i.getSource().substring(1));
			} else if (i.getSemantic().equals("NORMAL")) {
				normalOffset = i.getOffset();
				normalSource = m.getSource(i.getSource().substring(1));
			}

		}

		float[][] texturePoints = null; // dummy init
		if (textureSource != null) {
			// Triangle has texture
			texturePoints = SourceToFloat(textureSource);
		}

		float[][] normalCoordinates = null; // dummy init
		if (normalSource != null) {
			// Triangle has texture
			normalCoordinates = SourceToFloat(normalSource);
		}

		// one single triangle from the triangle-set
		for (int i = 0; i < indexes.length; i++) {
			Triangle tri = new Triangle();
			tri.setAppearance(appearance);

			if (tri.getAppearance().getImageFileName() != null)
				tri.setTexPosition(new Point2f[3]);
			// calculate triangle normal

			for (int v = 0; v < 3; v++) {
				// set the vertices for Point
				Vertex vert = vertices.get(indexes[i][vertexOffset]);

				tri.getPosition()[v] = vert;

				// set texture position
				if (tri.getAppearance().getImageFileName() != null) {
					tri.getTexPosition()[v] = new Point2f(
							texturePoints[indexes[i][textureOffset]][0],
							texturePoints[indexes[i][textureOffset]][1]);
				}

				if (normalCoordinates != null) {
					Vector3f norm = new Vector3f(normalCoordinates[indexes[i][normalOffset]][0],
							normalCoordinates[indexes[i][normalOffset]][1],
							normalCoordinates[indexes[i][normalOffset]][2]);
					vert.setNormalVector(norm);
				}

				i++;
			}
			i--;

			if (!tri.calculateNormalVector()) {
				continue;
			}
			tri.updateCentroid();

			// add it to the Collection
			currGroup.addTriangle(tri);
		}

	}

	/**
	 * Convert a collada source item to the corresponding float array
	 * 
	 * @param s
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
	 * Applies the list of transformations in the correct order to the vertex
	 * 
	 * @param vertex
	 *            vertex to apply the transformation onto
	 * @param transformations
	 *            list of transformations to apply
	 * @param withTranslate
	 *            if false, only rotation part of transformation matrix is applied. If true, the
	 *            vertex is also translated according to the transformation matrix.
	 */
	private static void useTransformation(Vertex vertex, List<BaseXform> transformations,
			boolean withTranslate) {
		for (int idx = transformations.size() - 1; idx >= 0; idx--) {
			BaseXform transform = transformations.get(idx);
			if (transform instanceof Matrix) {
				int cnt = 4;
				float matrix[][] = new float[cnt][cnt];
				int v = 0;
				for (int ma_y = 0; ma_y < cnt; ma_y++) {
					for (int ma_x = 0; ma_x < cnt; ma_x++) {
						if (!withTranslate && (ma_y > 2 || ma_x > 2) && ma_y != ma_x)
							// only use rotation matrix (eg for rotating and not translating normal
							// vector)
							matrix[ma_y][ma_x] = 0;
						else
							matrix[ma_y][ma_x] = ((Matrix) transform).getData()[v];

						v++;
					}
				}
				vertex.transform(matrix);
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
	protected boolean loadModel(String filename) {
		Collada collada = null;
		textureBasePath = null;
		String daeFile = null;

		// Extract kmz if needed and determin location of .dae file
		String extension = getExtension(filename);
		if (extension.equalsIgnoreCase("kmz")) {
			String tmpPath = ResourceRetriever.createTempDirectory();
			if (!tmpPath.endsWith("/") && !tmpPath.endsWith("\\"))
				tmpPath += File.separator;
			FileUtil.Unzip(filename, tmpPath);

			daeFile = getDaeLocation(tmpPath);

			textureBasePath = daeFile.substring(0, daeFile.lastIndexOf(File.separator));

			if (!textureBasePath.endsWith("/") && !textureBasePath.endsWith("\\"))
				textureBasePath += File.separator;

		} else if (extension.equalsIgnoreCase("dae")) {
			String fullPath = new File(filename).getPath();
			textureBasePath = fullPath.substring(0, fullPath.length()
					- new File(filename).getName().length());
			daeFile = filename;
		}

		// Parse dae with collada xml parser
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

		model = new Model();
		model.setGroup(new Group(model));

		// collada.dump(System.out, 0);

		// library_visual_scenes is where everything begins
		VisualScene scene = collada.getLibraryVisualScenes().getScene(
				collada.getScene().getInstanceVisualScene().getUrl());

		for (Node n : scene.getNodes()) {
			parseNode(n, model.getGroup(), new ArrayList<BaseXform>(), collada);
		}

		model.setTextureBasePath(textureBasePath);

		Unit unit = collada.getUnit();
		if (unit != null && unit.getMeter() != 1.0) {
			model.scale(unit.getMeter());
		}

		model.mirrorX();

		return true;
	}

	/**
	 * Parses a node from the collada structure and stores the triangles and lines information.
	 * 
	 * @param node
	 *            Node to parse (may have other sub-Nodes => called recursively)
	 * @param currGroup
	 *            Group of which the node is part of.
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
				Group g = new Group(currGroup.getModel());
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
