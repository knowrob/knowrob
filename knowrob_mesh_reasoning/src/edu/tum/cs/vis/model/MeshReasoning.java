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
 * @author Stefan Profanter
 * 
 */
public class MeshReasoning {

	/**
	 * Log4j logger
	 */
	private static Logger	logger	= Logger.getRootLogger();

	public static MeshReasoning initMeshReasoning(boolean withView) {

		DOMConfigurator.configureAndWatch("log4j.xml", 60 * 1000);
		return new MeshReasoning(withView);
	}

	private MeshReasoningView			mrv	= null;

	private MeshCas						cas	= null;

	public JFrame						frame;

	private MeshReasoningViewControl	control;

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

	public void analyseByIdentifier(String prologIdentifier) {
		String path = Properties.getPropertyStringOfClassOrIndividual("knowrob:pathToCadModel",
				prologIdentifier);
		if (path != null)
			analyseByPath(path);
	}

	public void analyseByPath(String path) {

		logger.info("MeshReasoning started. Parsing model ...");
		logger.debug("Path: " + path);
		long start = System.currentTimeMillis();

		ItemModel itemModel = new ItemModel(path);

		if (itemModel.getParser() == null) {
			logger.error("Couldn't parse model. Maybe path of model file is wrong.");
			return;
		}

		Model model = itemModel.getParser().getModel();
		// if (path.endsWith("ply"))
		model.removeDoubleSidedTriangles(); // in ply files there may be double sided triangles
		logger.debug("Model parsed. Took: "
				+ PrintUtil.prettyMillis(System.currentTimeMillis() - start) + " (Vertices: "
				+ model.getVertices().size() + ", Lines: " + model.getLines().size()
				+ ", Triangles: " + model.getTriangles().size() + ")");

		logger.debug("Calculating curvature ...");

		model.normalize();

		ArrayList<MeshAnalyser> analyser;
		if (mrv != null) {
			analyser = mrv.getControl().getAnalyser();
		} else {
			cas = new MeshCas();
			analyser = new ArrayList<MeshAnalyser>();
		}
		cas.setModel(model);
		CurvatureCalculation.calculateCurvatures(cas.getCurvatures(), model, path.endsWith("ply"));

		NeighborAnalyser na = new NeighborAnalyser();
		analyser.add(na);
		PrimitiveAnalyser pa = new PrimitiveAnalyser();
		analyser.add(pa);
		ContainerAnalyser ca = new ContainerAnalyser();
		analyser.add(ca);

		Thread.yield();

		na.process(cas);
		pa.process(cas);
		ca.process(cas);

	}

	public void clearHightlight() {
		mrv.clearSelectedAnnotations();
	}

	public HashSet<ConeAnnotation> findAnnotationsCone() {
		return cas.findAnnotations(ConeAnnotation.class);
	}

	public HashSet<ContainerAnnotation> findAnnotationsContainer() {
		return cas.findAnnotations(ContainerAnnotation.class);
	}

	public HashSet<PlaneAnnotation> findAnnotationsPlane() {
		return cas.findAnnotations(PlaneAnnotation.class);
	}

	public HashSet<SphereAnnotation> findAnnotationsSphere() {
		return cas.findAnnotations(SphereAnnotation.class);
	}

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

	public void highlightAnnotation(MeshAnnotation a) {
		if (mrv == null)
			return;
		mrv.addSelectedAnnotation(a);
	}

	public void setDefaultImageFilename(String s) {
		if (control != null)
			control.setDefaultImageFilename(s);
	}

	public void setFrameTitle(String title) {
		frame.setTitle(title);
	}

}
