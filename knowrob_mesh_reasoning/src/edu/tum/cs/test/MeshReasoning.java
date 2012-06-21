/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.test;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.util.ArrayList;

import javax.swing.JFrame;

import org.apache.log4j.Logger;
import org.apache.log4j.xml.DOMConfigurator;

import edu.tum.cs.util.PrintUtil;
import edu.tum.cs.vis.model.ItemModel;
import edu.tum.cs.vis.model.Model;
import edu.tum.cs.vis.model.uima.analyzer.ContainerAnalyzer;
import edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer;
import edu.tum.cs.vis.model.uima.analyzer.NeighborAnalyzer;
import edu.tum.cs.vis.model.uima.analyzer.PrimitiveAnalyzer;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.algorithm.CurvatureCalculation;
import edu.tum.cs.vis.model.view.MeshReasoningView;
import edu.tum.cs.vis.model.view.MeshReasoningViewControl;

/**
 * Main test class for testing MeshReasoning
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshReasoning {

	/**
	 * Log4j logger
	 */
	private static Logger	logger	= Logger.getRootLogger();

	/**
	 * Main Method loading the mesh, drawing it and starting the analyzer
	 * 
	 * @param args
	 *            command line arguments
	 */
	public static void main(String[] args) {
		DOMConfigurator.configureAndWatch("log4j.xml", 60 * 1000);

		logger.info("MeshReasoning started. Parsing model ...");
		long start = System.currentTimeMillis();

		// ItemModel itemModel = new ItemModel("/home/stefan/simple_dome.kmz");
		// ItemModel itemModel = new ItemModel("models/hospital_bed.kmz");
		// ItemModel itemModel = new ItemModel("models/Expedit_2X4.kmz");
		// ItemModel itemModel = new ItemModel("models/quader.dae");
		// ItemModel itemModel = new ItemModel("models/open_box.kmz");

		// ItemModel itemModel = new ItemModel("models/cup2.kmz");
		// ItemModel itemModel = new ItemModel("models/sphere.dae");
		// ItemModel itemModel = new ItemModel("models/two_spheres.dae");
		// ItemModel itemModel = new ItemModel("models/test.dae");

		// ItemModel itemModel = new ItemModel("models/cylinders.dae");
		// ItemModel itemModel = new ItemModel("models/CylinderCone.dae");
		// ItemModel itemModel = new ItemModel("models/flatcylinder.dae");
		// ItemModel itemModel = new ItemModel("models/cylinderpart.dae");
		// ItemModel itemModel = new ItemModel("models/cone_part.dae");
		// ItemModel itemModel = new ItemModel("models/pyramid.dae");
		// ItemModel itemModel = new ItemModel("models/Skateboard.dae");
		// ItemModel itemModel = new ItemModel("models/Table.dae");
		// ItemModel itemModel = new ItemModel("models/Dome.dae");
		// ItemModel itemModel = new ItemModel("models/cylinder.dae");
		// ItemModel itemModel = new ItemModel("models/cup_red.kmz");
		// ItemModel itemModel = new ItemModel("models/boxcylinder.dae");
		// ItemModel itemModel = new ItemModel("models/boxcone.dae");
		// ItemModel itemModel = new ItemModel("models/cone.dae");
		// ItemModel itemModel = new ItemModel("models/box.dae");
		// ItemModel itemModel = new ItemModel("models/plane2.dae");
		// ItemModel itemModel = new ItemModel("models/sheetmetal.dae");

		// ItemModel itemModel = new ItemModel("models/test2.dae");
		// ItemModel itemModel = new ItemModel("models/hammer.dae");
		// ItemModel itemModel = new ItemModel("models/spoon.dae");
		ItemModel itemModel = new ItemModel("models/spoon21.dae");
		// ItemModel itemModel = new ItemModel("models/spoon_hires.dae");
		// ItemModel itemModel = new ItemModel("models/ketchup bottle.dae");

		if (itemModel.getParser() == null) {
			throw new RuntimeException("Couldn't parse model. Maybe path of model file is wrong.");
		}

		Model model = itemModel.getParser().getModel();
		logger.debug("Model parsed. Took: "
				+ PrintUtil.prettyMillis(System.currentTimeMillis() - start) + " (Vertices: "
				+ model.getVertices().size() + ", Lines: " + model.getLines().size()
				+ ", Triangles: " + model.getTriangles().size() + ")");

		logger.debug("Calculating curvature ...");

		model.normalize();

		MeshCas cas = new MeshCas();
		CurvatureCalculation.calculateCurvatures(cas.getCurvatures(), model);

		/*Group g = new Group();
		g.setMesh(new Mesh());
		
		Triangle t = new Triangle();
		Appearance app = new Appearance();
		app.setColourFill(new Color(0,255,0));
		t.setAppearance(app);
		t.getPosition()[0] = new Point3f(1,4,0);
		t.getPosition()[1] = new Point3f(5,4,0);
		t.getPosition()[2] = new Point3f(2,1,1);
		
		t.updateNormalVector();
		
		g.getMesh().triangles.add(t);*/

		cas.setModel(model);

		// cas.setGroup(g);

		JFrame frame = new JFrame();
		frame.setMinimumSize(new Dimension(800, 600));
		frame.setSize(1324, 768);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setTitle("Mesh reasoning view");
		frame.setLocationRelativeTo(null);

		ArrayList<MeshAnalyzer> analyzer = new ArrayList<MeshAnalyzer>();

		MeshReasoningView mrv = new MeshReasoningView();
		MeshReasoningViewControl control = new MeshReasoningViewControl(cas, analyzer, mrv);
		mrv.setControl(control);
		mrv.init();

		mrv.getCasList().add(cas);

		frame.setLayout(new BorderLayout());
		frame.getContentPane().add(mrv, BorderLayout.CENTER);
		frame.getContentPane().add(control, BorderLayout.LINE_END);
		frame.setVisible(true);

		NeighborAnalyzer na = new NeighborAnalyzer();
		analyzer.add(na);
		PrimitiveAnalyzer pa = new PrimitiveAnalyzer();
		analyzer.add(pa);
		ContainerAnalyzer ca = new ContainerAnalyzer();
		analyzer.add(ca);

		Thread.yield();

		na.process(cas);
		pa.process(cas);
		ca.process(cas);

	}
}
