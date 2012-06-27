/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.test;

import edu.tum.cs.vis.model.MeshReasoning;

/**
 * Main test class for testing MeshReasoningTest
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshReasoningTest {

	/**
	 * Main Method loading the mesh, drawing it and starting the analyzer
	 * 
	 * @param args
	 *            command line arguments
	 */
	public static void main(String[] args) {

		// String path = "/home/stefan/simple_dome.kmz";
		// String path = "models/hospital_bed.kmz";
		// String path = "models/Expedit_2X4.kmz";
		// String path = "models/quader.dae";
		// String path = "models/open_box.kmz";

		String path = "models/cup2.kmz";
		// String path = "models/sphere.dae";
		// String path = "models/two_spheres.dae";
		// String path = "models/test.dae";

		// String path = "models/cylinders.dae";
		// String path = "models/CylinderCone.dae";
		// String path = "models/flatcylinder.dae";
		// String path = "models/cylinderpart.dae";
		// String path = "models/cone_part.dae";
		// String path = "models/pyramid.dae";
		// String path = "models/Skateboard.dae";
		// String path = "models/Table.dae";
		// String path = "models/Dome.dae";
		// String path = "models/cylinder.dae";
		// String path = "models/cup_red.kmz";
		// String path = "models/boxcylinder.dae";
		// String path = "models/boxcone.dae";
		// String path = "models/cone.dae";
		// String path = "models/box.dae";
		// String path = "models/plane2.dae";
		// String path = "models/sheetmetal.dae";

		// String path = "models/test2.dae";
		// String path = "models/hammer.dae";
		// String path = "models/spoon.dae";
		// String path = "models/spoon2.dae";
		// String path = "models/spoon_hires.dae";
		// String path = "models/ketchup bottle.dae";

		MeshReasoning mr = MeshReasoning.initMeshReasoning(true);

		mr.analyzeByPath(path);
	}
}
