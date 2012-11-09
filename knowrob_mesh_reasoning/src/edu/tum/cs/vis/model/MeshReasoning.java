/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.HashSet;

import javax.swing.JFrame;

import org.apache.log4j.Logger;
import org.apache.log4j.xml.DOMConfigurator;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.util.PrintUtil;
import edu.tum.cs.util.ResourceRetriever;
import edu.tum.cs.vis.model.uima.analyser.ContainerAnalyser;
import edu.tum.cs.vis.model.uima.analyser.MeshAnalyser;
import edu.tum.cs.vis.model.uima.analyser.NeighborAnalyser;
import edu.tum.cs.vis.model.uima.analyser.PrimitiveAnalyser;
import edu.tum.cs.vis.model.uima.annotation.ContainerAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.SphereAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.algorithm.CurvatureCalculation;
import edu.tum.cs.vis.model.view.MeshReasoningView;
import edu.tum.cs.vis.model.view.MeshReasoningViewControl;

/**
 * Main mesh reasoning class for parsing and analyzing CAD models. Provide methods for starting mesh
 * reasoning with and without GUI.
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
	 * Main initialization method for creating mesh reasoning object. Constructs mesh reasoning
	 * object and initializes log4j logger.
	 * 
	 * @param withView
	 *            Also create GUI to visualize mesh reasoning
	 * @return new mesh reasoning object
	 */
	public static MeshReasoning initMeshReasoning(boolean withView) {

		DOMConfigurator.configureAndWatch("log4j.xml", 60 * 1000);
		return new MeshReasoning(withView);
	}

	/**
	 * View for this mesh reasoning object
	 */
	private MeshReasoningView			mrv	= null;

	/**
	 * Mesh reasoning container
	 */
	private MeshCas						cas	= null;

	/**
	 * Main frame container for mesh reasoning view
	 */
	public JFrame						frame;

	/**
	 * View control for mesh reasoning view
	 */
	private MeshReasoningViewControl	control;

	/**
	 * Constructor for mesh reasoning object. Initializes object and creates mesh reasoning view if
	 * indicated.
	 * 
	 * @param withView
	 *            set to true if mesh reasoning view should be created and shown
	 * 
	 */
	public MeshReasoning(boolean withView) {
		cas = new MeshCas();

		if (withView) {
			frame = new JFrame();
			frame.setMinimumSize(new Dimension(800, 600));
			frame.setSize(1324, 768);
			frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			frame.setTitle("Mesh reasoning view");
			frame.setLocationRelativeTo(null);

			ArrayList<MeshAnalyser> analyser = new ArrayList<MeshAnalyser>();

			mrv = new MeshReasoningView();
			control = new MeshReasoningViewControl(cas, analyser, mrv);
			mrv.setControl(control);
			mrv.init();

			mrv.getCasList().add(cas);

			frame.setLayout(new BorderLayout());
			frame.getContentPane().add(mrv, BorderLayout.CENTER);
			frame.getContentPane().add(control, BorderLayout.LINE_END);
			frame.setVisible(true);
		}

	}

	/**
	 * Start mesh reasoning on specified file path
	 * 
	 * @param path
	 *            path to CAD model. Can be physical file path or http://, ftp:// or even package://
	 *            which indicates a ros package
	 * 
	 * @see ResourceRetriever
	 */
	public void analyseByPath(String path) {

		logger.info("MeshReasoning started. Parsing model ...");
		logger.debug("Path: " + path);
		long start = System.currentTimeMillis();

		// Load and parse model
		ItemModel itemModel = new ItemModel(path);

		if (itemModel.getParser() == null) {
			logger.error("Couldn't parse model. Maybe path of model file is wrong.");
			return;
		}

		Model model = itemModel.getParser().getModel();
		model.removeDoubleSidedTriangles(); // in ply files there may be double sided triangles
		logger.debug("Model parsed. Took: "
				+ PrintUtil.prettyMillis(System.currentTimeMillis() - start) + " (Vertices: "
				+ model.getVertices().size() + ", Lines: " + model.getLines().size()
				+ ", Triangles: " + model.getTriangles().size() + ")");

		// normalize model for further reasoning
		model.normalize();

		// list of current running analyzers used in mesh reasoning view
		ArrayList<MeshAnalyser> analyser;
		if (mrv != null) {
			analyser = mrv.getControl().getAnalyser();
		} else {
			cas = new MeshCas();
			analyser = new ArrayList<MeshAnalyser>();
		}
		cas.setModel(model);

		// Create analyzers and start them

		NeighborAnalyser na = new NeighborAnalyser();
		analyser.add(na);
		Thread.yield();
		na.process(cas);

		logger.debug("Calculating curvature ...");
		long curvatureStartTime = System.currentTimeMillis();
		CurvatureCalculation.calculateCurvatures(cas.getCurvatures(), model);
		long curvatureDuration = System.currentTimeMillis() - curvatureStartTime;
		logger.debug("Ended. Took: " + PrintUtil.prettyMillis(curvatureDuration));

		PrimitiveAnalyser pa = new PrimitiveAnalyser();
		analyser.add(pa);
		ContainerAnalyser ca = new ContainerAnalyser();
		analyser.add(ca);

		Thread.yield();
		pa.process(cas);
		ca.process(cas);

	}

	/**
	 * Clear all highlighted annotations in mesh reasoning view
	 */
	public void clearHightlight() {
		if (mrv != null)
			mrv.clearSelectedAnnotations();
	}

	/**
	 * Get all cone annotations
	 * 
	 * @return Set of cone annotations in model
	 */
	public HashSet<ConeAnnotation> findAnnotationsCone() {
		return cas.findAnnotations(ConeAnnotation.class);
	}

	/**
	 * Get all container annotations
	 * 
	 * @return Set of container annotations in model
	 */
	public HashSet<ContainerAnnotation> findAnnotationsContainer() {
		return cas.findAnnotations(ContainerAnnotation.class);
	}

	/**
	 * Get all plane annotations
	 * 
	 * @return Set of plane annotations in model
	 */
	public HashSet<PlaneAnnotation> findAnnotationsPlane() {
		return cas.findAnnotations(PlaneAnnotation.class);
	}

	/**
	 * Get all sphere annotations
	 * 
	 * @return Set of sphere annotations in model
	 */
	public HashSet<SphereAnnotation> findAnnotationsSphere() {
		return cas.findAnnotations(SphereAnnotation.class);
	}

	/**
	 * Get list of all found annotation types in mesh object
	 * 
	 * @return List of annotation names such as Sphere, Cone, Plane, ...
	 */
	public ArrayList<String> getAnnotationTypes() {
		HashSet<String> types = new HashSet<String>();
		for (Annotation a : cas.getAnnotations()) {
			if (!(a instanceof MeshAnnotation))
				continue;
			@SuppressWarnings("rawtypes")
			MeshAnnotation ma = (MeshAnnotation) a;
			String cl = ma.getClass().getSimpleName();
			if (cl.length() < 1)
				continue;
			int pos = cl.indexOf("Annotation");
			if (pos > 0)
				cl = cl.substring(0, pos);
			types.add(cl);
		}
		ArrayList<String> ret = new ArrayList<String>();
		ret.addAll(types);
		return ret;
	}

	/**
	 * Highlight specified annotation in mesh reasoning view
	 * 
	 * @param a
	 *            Annotation to highlight
	 */
	public void highlightAnnotation(@SuppressWarnings("rawtypes") MeshAnnotation a) {
		if (mrv == null)
			return;
		mrv.addSelectedAnnotation(a);
	}

	/**
	 * Default image file name for saving current mesh reasoning view as a png image.
	 * 
	 * @param s
	 *            default file name
	 */
	public void setDefaultImageFilename(String s) {
		if (control != null)
			control.setDefaultImageFilename(s);
	}

	/**
	 * Set title of main frame
	 * 
	 * @param title
	 *            new title
	 */
	public void setFrameTitle(String title) {
		frame.setTitle(title);
	}

}
