/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.parser;

import java.awt.Color;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.nio.BufferOverflowException;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * Parser for .ply (Polygon File Format) files. Supports binary and ASCII files.
 * 
 * @author Stefan Profanter
 * 
 */
public class PlyParser extends ModelParser {

	/**
	 * File format of ply file
	 * 
	 * @author Stefan Profanter
	 * 
	 */
	private enum FileFormat {
		/**
		 * ASCII file format
		 */
		ASCII,
		/**
		 * Binary: little endian
		 */
		LITTLE_ENDIAN,
		/**
		 * Binary: big endian
		 */
		BIG_ENDIAN
	};

	/**
	 * Byte buffer used for binary parsing.
	 */
	private ByteBuffer	byteBuffer;

	/**
	 * File format of current file
	 */
	private FileFormat	fileFormat;

	/**
	 * Property of ply file (placed in header).
	 * 
	 * @author Stefan Profanter
	 * 
	 */
	private class Property {
		/**
		 * Creates a new property with given name.
		 * 
		 * @param name
		 *            Name of property
		 * 
		 */
		public Property(String name) {
			this.name = name;
		}

		/**
		 * Type of property or <tt>list</tt>. type for single property or type of list count.
		 */
		String	type1	= null; //
		/**
		 * if property is a list of values, this is the type of values
		 */
		String	type2	= null;
		/**
		 * name of property
		 */
		String	name;
	}

	/**
	 * Represents a line with numbers. Used for ASCII and binary.
	 * 
	 * @author Stefan Profanter
	 * 
	 */
	private class ValueLine {
		/**
		 * Values for the line
		 */
		ArrayList<Number>	values	= new ArrayList<Number>();
	}

	/**
	 * An element of ply file. May be for example a vertex or face.
	 * 
	 * @author Stefan Profanter
	 * 
	 */
	private class Element {
		/**
		 * Constructor for element.
		 * 
		 * @param n
		 *            Name of element
		 * @param cnt
		 *            count of sub elements.
		 */
		public Element(String n, int cnt) {
			this.name = n;
			this.cnt = cnt;
		}

		/**
		 * Name of element
		 */
		String					name;
		/**
		 * count of sub elements
		 */
		int						cnt;
		/**
		 * List of properties for element
		 */
		ArrayList<Property>		properties	= new ArrayList<Property>();
		/**
		 * List of value lines for element
		 */
		ArrayList<ValueLine>	lines		= new ArrayList<ValueLine>();
	};

	/**
	 * List of all elements of ply file
	 */
	private ArrayList<Element>	elements		= new ArrayList<Element>();

	/**
	 * Index of current element which is getting parsed.
	 */
	private int					currentElement	= 0;

	/**
	 * Dump ply file. Used for debugging purposes.
	 */
	@SuppressWarnings("unused")
	private void dump() {
		for (Element e : elements) {
			System.out.println(e.name);
			for (Property p : e.properties) {
				System.out.println("prop: " + p.name);
				System.out.println("prop:     " + p.type1);
				System.out.println("prop:     " + p.type2);
			}
			for (ValueLine l : e.lines) {
				for (Number n : l.values)
					System.out.print(n.toString() + " ");
				System.out.println("");
			}
		}
	}

	@Override
	protected boolean loadModel(String filename) {

		RandomAccessFile raf = null;

		try {
			raf = new RandomAccessFile(filename, "r");
			boolean cont = false;
			String line = "";
			line = raf.readLine();
			if (line == null || line.compareTo("ply") != 0) {
				System.err.println("ERRROR: magic bytes 'ply' not found in file.");
				return false;
			}
			do {
				line = raf.readLine();
				if (line != null)
					cont = processLine(line.split(" "));
			} while (cont && line.compareTo("end_header") != 0);

			if (cont) {
				if (fileFormat == FileFormat.ASCII) {
					do {
						line = raf.readLine();
						if (line != null)
							cont = processDataLine(line.split(" "));
					} while (cont && line != null);
				} else {
					int remain = (int) (raf.length() - raf.getFilePointer());
					byteBuffer = ByteBuffer.allocate(remain);
					if (fileFormat == FileFormat.BIG_ENDIAN)
						byteBuffer.order(ByteOrder.BIG_ENDIAN);
					else
						byteBuffer.order(ByteOrder.LITTLE_ENDIAN);

					for (int i = 0; i < remain; i++) {
						byte b = raf.readByte();
						byteBuffer.put(b);
					}
					byteBuffer.position(0);
					try {
						processDataBuffer();

					} catch (BufferUnderflowException e) {
						System.err.println("Couldn't parse model. " + e.toString() + " "
								+ e.getMessage());
						return false;
					} catch (BufferOverflowException e) {
						System.err.println("Couldn't parse model. " + e.toString() + " "
								+ e.getMessage());
						return false;
					}

				}
			}
		} catch (FileNotFoundException e) {
			System.out.println("ERROR: File not found: " + filename);
			return false;
		} catch (IOException e) {
			System.out.println("ERROR reading file: " + e.getMessage());
		} finally {
			try {
				if (raf != null)
					raf.close();
			} catch (IOException e) {
				System.out.println("ERROR closing file: " + e.getMessage());
			}
		}

		// dump();

		try {

			byteBuffer = null;

			Element vertexElement = null;
			Element faceElement = null;
			for (Element e : elements) {
				if (e.name.compareTo("vertex") == 0) {
					vertexElement = e;
				}
				if (e.name.compareTo("face") == 0) {
					faceElement = e;
				}
				if (vertexElement != null && faceElement != null)
					break;
			}

			if (vertexElement == null) {
				System.err.println("ERROR: .ply file doesn't contain vertex definition.");
				return false;
			}

			if (faceElement == null) {
				System.err.println("ERROR: .ply file doesn't contain face definition.");
				return false;
			}

			model = new Model();
			Group g = new Group(model);
			model.setGroup(g);
			Mesh m = new Mesh();
			g.setMesh(m);

			model.setTextureBasePath(null);

			int coordIdx[] = new int[] { -1, -1, -1 };
			int colorIdx[] = new int[] { -1, -1, -1 };
			int idxOk = 0;
			int colorOk = 0;
			for (int i = 0; i < vertexElement.properties.size(); i++) {
				Property p = vertexElement.properties.get(i);
				if (p.name.equals("x")) {
					coordIdx[0] = i;
					idxOk++;
				} else if (p.name.equals("y")) {
					coordIdx[1] = i;
					idxOk++;
				} else if (p.name.equals("z")) {
					coordIdx[2] = i;
					idxOk++;
				} else if (p.name.equals("red")) {
					colorIdx[0] = i;
					colorOk++;
				} else if (p.name.equals("green")) {
					colorIdx[1] = i;
					colorOk++;
				} else if (p.name.equals("nlue")) {
					colorIdx[2] = i;
					colorOk++;
				}
			}

			if (idxOk != 3) {
				System.err.println("ERROR: x,y,z not properly defined for vertices in header.");
				model = null;
				return false;
			}

			for (ValueLine l : vertexElement.lines) {
				if (l.values.size() < 3) {
					System.err.println("ERROR: invalid vertex coordinate count: " + l.values.size()
							+ " for vertex index " + model.getVertices().size());
					model = null;
					return false;
				}
				Vertex v = new Vertex(l.values.get(coordIdx[0]).floatValue(), l.values.get(
						coordIdx[1]).floatValue(), l.values.get(coordIdx[2]).floatValue());
				if (colorOk == 3) {
					v.color = new Color(l.values.get(colorIdx[0]).intValue(), l.values.get(
							colorIdx[1]).intValue(), l.values.get(colorIdx[2]).intValue());
				}
				model.getVertices().add(v);
			}

			for (ValueLine l : faceElement.lines) {
				if (l.values.size() < 3) {
					System.err.println("WARN: Skipping face with less than 3 points");
				} else if (l.values.size() == 3) {
					Triangle t = new Triangle();
					Vertex[] vert = new Vertex[3];
					for (int i = 0; i < 3; i++) {
						vert[i] = model.getVertices().get(l.values.get(i).intValue());
					}
					t.setPosition(vert);
					if (!t.calculateNormalVector()) {
						continue;
					}
					g.addTriangle(t);
				} else {

					Vertex[] polyVert = new Vertex[l.values.size()];
					for (int i = 0; i < l.values.size(); i++) {
						polyVert[i] = model.getVertices().get(l.values.get(i).intValue());
					}
					int triangleIndices[] = polygonTriangulation(polyVert);
					for (int i = 0; i < triangleIndices.length / 3; i++) {
						Triangle t = new Triangle();
						Vertex[] vert = new Vertex[3];
						for (int j = 0; j < 3; j++) {
							vert[j] = polyVert[triangleIndices[i * 3 + j]];
						}
						t.setPosition(vert);
						if (!t.calculateNormalVector()) {
							continue;
						}
						g.addTriangle(t);
					}
				}
			}
		} catch (IndexOutOfBoundsException e) {
			System.err.println("Couldn't parse model. " + e.toString() + " " + e.getMessage());
			return false;
		} catch (BufferUnderflowException e) {
			System.err.println("Couldn't parse model. " + e.toString() + " " + e.getMessage());
			return false;
		} catch (BufferOverflowException e) {
			System.err.println("Couldn't parse model. " + e.toString() + " " + e.getMessage());
			return false;
		}

		elements = null;

		return true;
	}

	/**
	 * Process header line.
	 * 
	 * @param parts
	 *            Parts of line separated by spaces
	 * @return true if successfully parsed.
	 * 
	 */
	private boolean processLine(String parts[]) {
		if (parts.length < 2)
			return true;
		if (parts[0].compareTo("format") == 0) {
			if (parts[1].compareTo("ascii") == 0)
				fileFormat = FileFormat.ASCII;
			else if (parts[1].compareTo("binary_little_endian") == 0)
				fileFormat = FileFormat.LITTLE_ENDIAN;
			else if (parts[1].compareTo("binary_big_endian") == 0)
				fileFormat = FileFormat.BIG_ENDIAN;
			else {
				System.err.println("ERROR: " + parts[0] + " header unknown: " + parts[1]);
				return false;
			}
			return true;
		} else if (parts[0].compareTo("element") == 0) {
			elements.add(new Element(parts[1], Integer.valueOf(parts[2])));
			return true;
		} else if (parts[0].compareTo("property") == 0) {
			Property p = new Property(parts[parts.length - 1]);
			if (parts[1].compareTo("list") == 0) {
				p.type1 = parts[2];
				p.type2 = parts[3];
			} else {
				p.type1 = parts[1];
			}
			elements.get(elements.size() - 1).properties.add(p);
			return true;
		}
		return true;
	}

	/**
	 * Process ASCII line of data.
	 * 
	 * @param parts
	 *            Parts of line separated by spaces
	 * @return true if successfully parsed.
	 */
	private boolean processDataLine(String parts[]) {
		int offset = 0;
		int cnt = 1;

		if (elements.get(currentElement).properties.get(0).type2 != null) {
			// it is list
			cnt = Integer.valueOf(parts[0]);
			offset = 1;
		} else {
			cnt = elements.get(currentElement).properties.size();
		}

		ValueLine l = new ValueLine();
		for (int i = offset; i < cnt + offset; i++) {
			l.values.add(Float.valueOf(parts[i]));
		}
		elements.get(currentElement).lines.add(l);
		if (elements.get(currentElement).lines.size() == elements.get(currentElement).cnt)
			currentElement++;
		return true;
	}

	/**
	 * Get next bytes of byteBuffer where <tt>type</tt> indicates which number type is expected
	 * (char, short, long, ...) and therefore how much bytes to read.
	 * 
	 * @param type
	 *            Type of expected value.
	 * @return parsed value
	 * 
	 */
	private Number parseType(String type) {
		if (type.equals("char")) {
			return new Integer(byteBuffer.get());
		} else if (type.equals("uchar")) {
			return new Integer(byteBuffer.get());
		} else if (type.equals("short")) {
			short c = byteBuffer.getShort();
			return new Integer(c);
		} else if (type.equals("ushort")) {
			short c = byteBuffer.getShort();
			int v = new Integer(c);
			v += 32768; // TODO not yet tested
			return new Integer(v);
		} else if (type.equals("int")) {
			return new Integer(byteBuffer.getInt());
		} else if (type.equals("uint")) {
			int v = byteBuffer.getInt();
			v += 2147483648l; // TODO not yet tested
			return new Integer(v);
		} else if (type.equals("float")) {
			return new Float(byteBuffer.getFloat());
		} else if (type.equals("int")) {
			return new Float(byteBuffer.getDouble());
		} else {
			System.err.println("ERROR: invalid data type: " + type);
			return null;
		}
	}

	/**
	 * Parse current binary buffer by converting bytes into numbers and storing them in internal
	 * structure.
	 * 
	 * @return true if successfully parsed.
	 */
	private boolean processDataBuffer() {

		for (int el = 0; el < elements.size(); el++) {
			for (int j = 0; j < elements.get(el).cnt; j++) {
				int valCnt;
				String listType = null;
				if (elements.get(el).properties.get(0).type2 != null) {
					listType = elements.get(el).properties.get(0).type2;
					valCnt = (Integer) parseType(elements.get(el).properties.get(0).type1);
				} else {
					listType = null;
					valCnt = elements.get(el).properties.size();
				}

				ValueLine l = new ValueLine();
				for (int i = 0; i < valCnt; i++) {
					String type = listType;
					if (type == null) {
						type = elements.get(el).properties.get(i).type1;
					}
					l.values.add(parseType(type));
				}
				elements.get(el).lines.add(l);
			}
		}
		return true;
	}
}
