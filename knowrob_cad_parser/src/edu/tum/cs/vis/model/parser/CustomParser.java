/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.parser;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.InputStreamReader;

import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * 
 * Parser for simple file format, where first all vertices are listed and after <tt>#</tt> 3 values
 * per line for each triangle indicating the vertex index.
 * 
 * Example:
 * 
 * <pre>
 * 0.5	0.4	0.5
 * 0.4	0.3	1.3
 * ....
 * #
 * 0	3	4
 * 0	1	2
 * 1	2	4
 * ....
 * </pre>
 * 
 * @author Stefan Profanter
 * 
 */
public class CustomParser extends ModelParser {

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.parser.ModelParser#loadModel(java.lang.String)
	 */
	@Override
	protected boolean loadModel(String filename) {
		try {
			// Open the file that is the first
			// command line parameter
			FileInputStream fstream = new FileInputStream(filename);
			// Get the object of DataInputStream
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;

			Group g = new Group(model);
			model.setGroup(g);
			Mesh m = new Mesh();
			g.setMesh(m);

			boolean isTriangles = false;
			// Read File Line By Line
			while ((strLine = br.readLine()) != null) {
				if (strLine.compareTo("#") == 0)
					isTriangles = true;
				else {
					String[] parts = strLine.split("\t");

					if (!isTriangles) {
						if (parts.length != 5)
							throw new RuntimeException("Vertex: invalid number of data: "
									+ parts.length);

						float x = Float.valueOf(parts[0]);
						float y = Float.valueOf(parts[1]);
						float z = Float.valueOf(parts[2]);

						model.getVertices().add(new Vertex(x, y, z));
					} else {
						if (parts.length != 3)
							throw new RuntimeException("Triangle: invalid number of data: "
									+ parts.length);
						int p1 = Integer.valueOf(parts[0]);
						int p2 = Integer.valueOf(parts[1]);
						int p3 = Integer.valueOf(parts[2]);

						Triangle t = new Triangle();
						t.getPosition()[0] = model.getVertices().get(p1);
						t.getPosition()[1] = model.getVertices().get(p2);
						t.getPosition()[2] = model.getVertices().get(p3);

						t.updateCentroid();
						m.getTriangles().add(t);
					}
				}

			}
			// Close the input stream
			in.close();

		} catch (Exception e) {// Catch exception if any
			System.err.println("Error: " + e.getMessage());
			return false;
		}

		return true;

	}
}
