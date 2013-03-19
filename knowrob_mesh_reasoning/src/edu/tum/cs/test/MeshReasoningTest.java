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
	 *            command line arguments not used
	 */
	public static void main(String[] args) {

		// String path = "models/batch/kitchen/knife1.dae";

		// String path = "models/box.dae";
		// String path = "models/cup2.kmz";
		// String path = "/home/stefan/work/models/primitive/cylindery.dae";
		// String path = "/home/stefan/work/models/primitive/CylinderPart.dae";
		// String path = "/home/stefan/work/models/primitive/cone.dae";
		// String path = "/home/stefan/work/models/primitive/ConeCut.dae";
		// String path = "/home/stefan/work/models/primitive/BoxCone.dae";
		// String path = "/home/stefan/work/models/primitive/ConePart.dae";
		// String path = "/home/stefan/work/models/primitive/TestObject.dae";
		// String path = "/home/stefan/work/models/primitive/CupOuter.dae";
		// String path = "/home/stefan/work/models/primitive/ConeSphere.dae";
		// String path = "/home/stefan/work/models/primitive/InvCone.dae";

		/* Complex handle test */
		// String path = "/home/stefan/work/models/hand-tools/putty.kmz";
		// String path = "/home/stefan/work/models/hand-tools/hammer2.dae";
		// String path = "/home/stefan/work/models/hand-tools/wrench.kmz";
		// String path = "/home/stefan/work/models/silverware/spoon2.kmz";
		// String path = "/home/stefan/work/models/silverware/knife1.kmz";
		// String path = "/home/stefan/work/models/industrial-parts/valves/Ball Valve.kmz";
		// String path = "/home/stefan/work/models/silverware/Butter Knife.dae";
		// String path = "/home/stefan/work/models/food-drinks/coke.kmz";
		// String path = "/home/stefan/work/models/drinking-vessels/cup2.dae";
		// String path = "/home/stefan/work/models/drinking-vessels/my_mug_v2.kmz";
		// String path = "/home/stefan/work/models/food-drinks/mondamin-pancake-mix.dae";
		// String path = "/home/stefan/work/models/cooking-vessels/pot-ww.dae";
		// String path = "/home/stefan/work/models/cooking-vessels/pancake_maker.dae";
		// String path = "/home/stefan/work/models/food-drinks/ketchup bottle.dae";
		String path = "/home/stefan/work/models/primitive/Handle.dae";
		MeshReasoning mr = MeshReasoning.initMeshReasoning(true);

		mr.analyseByPath(path);
	}
}
