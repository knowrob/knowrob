/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import javax.swing.JFrame;

import org.apache.log4j.Logger;
import org.apache.log4j.xml.DOMConfigurator;

import sun.awt.SunToolkit;
import edu.tum.cs.tools.ImageGenerator.ImageGeneratorAction;
import edu.tum.cs.tools.ImageGenerator.ImageGeneratorSettings;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.util.PrintUtil;
import edu.tum.cs.vis.model.uima.analyser.ComplexHandleAnalyser;
import edu.tum.cs.vis.model.uima.analyser.ContainerAnalyser;
import edu.tum.cs.vis.model.uima.analyser.MeshAnalyser;
import edu.tum.cs.vis.model.uima.analyser.NeighborAnalyser;
import edu.tum.cs.vis.model.uima.analyser.PrimitiveAnalyser;
import edu.tum.cs.vis.model.uima.annotation.ComplexHandleAnnotation;
import edu.tum.cs.vis.model.uima.annotation.ContainerAnnotation;
import edu.tum.cs.vis.model.uima.annotation.DrawableAnnotation;
import edu.tum.cs.vis.model.uima.annotation.HandleAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.SphereAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.ContainerAnnotationVolumeComarator;
import edu.tum.cs.vis.model.util.HandleComparator;
import edu.tum.cs.vis.model.util.PrimitiveAnnotationAreaComparator;
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
	static Logger	logger	= Logger.getRootLogger();

	/**
	 * Main initialization method for creating mesh reasoning object. Constructs mesh reasoning
	 * object and initializes log4j logger.
	 * 
	 * @param withView
	 *            Also create GUI to visualize mesh reasoning
	 * @return new mesh reasoning object
	 */
	public static MeshReasoning initMeshReasoning(boolean withView) {
		return MeshReasoning.initMeshReasoning(withView, null);
	}

	public static MeshReasoning initMeshReasoning(boolean withView,
			ImageGeneratorSettings imageGenerator) {
		DOMConfigurator.configureAndWatch("log4j.xml", 60 * 1000);
		return new MeshReasoning(withView, imageGenerator);
	}

	private ImageGeneratorSettings		imageGeneratorSettings	= null;

	/**
	 * View for this mesh reasoning object
	 */
	private MeshReasoningView			mrv						= null;

	/**
	 * Mesh reasoning container
	 */
	private MeshCas						cas						= null;

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
		this(withView, null);
	}

	public MeshReasoning(boolean withView, ImageGeneratorSettings imageGenerator) {
		imageGeneratorSettings = imageGenerator;
		if (imageGeneratorSettings != null)
			initImageGenerator();
		// initImageGeneratorSpoon();
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
			mrv.setImageGeneratorSettings(imageGeneratorSettings);
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
	 */
	public void analyseByPath(String path) {

		if (imageGeneratorSettings != null) {
			// update file name
			imageGeneratorSettings.setCurrentModel(path.substring(imageGeneratorSettings
					.getInputBasePath().getAbsolutePath().length() + 1));
		}
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
		logger.debug("Model parsed. Took: "
				+ PrintUtil.prettyMillis(System.currentTimeMillis() - start) + " (Vertices: "
				+ model.getVertices().size() + ", Lines: " + model.getLines().size()
				+ ", Triangles: " + model.getTriangles().size() + ")");

		start = System.currentTimeMillis();

		// list of current running analyzers used in mesh reasoning view
		ArrayList<MeshAnalyser> analyser;
		if (mrv != null) {
			analyser = mrv.getControl().getAnalyser();
		} else {
			cas = new MeshCas();
			analyser = new ArrayList<MeshAnalyser>(5);
		}
		cas.setModel(model);

		// in ply (and also collada) files there may be double sided triangles
		logger.debug("Checking for double sided triangles ...");
		logger.debug("Removed " + model.removeDoubleSidedTriangles() + " triangles. Took: "
				+ PrintUtil.prettyMillis(System.currentTimeMillis() - start));

		NeighborAnalyser na = new NeighborAnalyser();
		analyser.add(na);
		Thread.yield();
		na.process(cas);

		logger.debug("Checking for vertex sharing and calculating vertex normals ...");
		model.updateVertexSharing();

		model.updateVertexNormals();
		logger.debug("Model initialized. Took: "
				+ PrintUtil.prettyMillis(System.currentTimeMillis() - start) + " (Vertices: "
				+ model.getVertices().size() + ", Lines: " + model.getLines().size()
				+ ", Triangles: " + model.getTriangles().size() + ")");

		// File f = model.exportVerticesAsTxt();

		// normalize model for further reasoning
		model.normalize();

		if (imageGeneratorSettings != null) {
			imageGeneratorSettings.waitSetup();

			if (imageGeneratorSettings.isInitViewFromFile()) {
				if (imageGeneratorSettings.initView(mrv.getCam())) {
					SunToolkit.flushPendingEvents();
					try {
						Thread.sleep(2500); // wait until model correctly shown
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			if (imageGeneratorSettings.isSaveView() && !imageGeneratorSettings.isViewInitialized()) {
				// allow user to set a viewpoint
				logger.info("Set the desired view and then press Ctrl+S to continue");
				imageGeneratorSettings.waitViewInitialized();
			}

			if (imageGeneratorSettings.isSavePlainModel()) {
				// wait until model is saved
				imageGeneratorSettings.waitSaved("plain");
			}
		}

		// Create analyzers and start them

		logger.debug("Calculating curvature ...");
		long curvatureStartTime = System.currentTimeMillis();
		CurvatureCalculation.calculateCurvatures(cas.getCurvatures(), model);
		long curvatureDuration = System.currentTimeMillis() - curvatureStartTime;
		logger.debug("Ended. Took: " + PrintUtil.prettyMillis(curvatureDuration));

		if (imageGeneratorSettings != null && imageGeneratorSettings.isSaveCurvatureColor()) {
			// wait until model is saved
			mrv.setDrawCurvatureColor(true);
			imageGeneratorSettings.waitSaved("curvature");
			mrv.setDrawCurvatureColor(false);
		}

		PrimitiveAnalyser pa = new PrimitiveAnalyser();
		analyser.add(pa);
		ContainerAnalyser ca = new ContainerAnalyser();
		analyser.add(ca);
		ComplexHandleAnalyser cha = new ComplexHandleAnalyser();
		analyser.add(cha);

		Thread.yield();
		pa.process(cas, imageGeneratorSettings);
		ca.process(cas, imageGeneratorSettings);
		cha.process(cas, imageGeneratorSettings);

		if (imageGeneratorSettings != null && imageGeneratorSettings.isCloseAfterFinish()) {
			logger.debug("Closing ...");
			System.exit(123);
		}
	}

	/**
	 * Clear all highlighted annotations in mesh reasoning view
	 */
	public void clearHightlight() {
		if (mrv != null)
			mrv.clearSelectedAnnotations();
	}

	@SuppressWarnings("rawtypes")
	public <T extends MeshAnnotation> HashSet<T> findAnnotations(Class<T> clazz) {
		return cas.findAnnotations(clazz);
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

	public HandleAnnotation[] getHandle() {
		return getHandle(-1f, -1f, -1f, -1f);
	}

	public HandleAnnotation[] getHandle(double minRadius, double maxRadius) {
		return getHandle(minRadius, maxRadius, -1f, -1f);
	}

	public HandleAnnotation[] getHandle(double minRadius, double maxRadius, double minLength,
			double maxLength) {

		Set<ConeAnnotation> cones = findAnnotationsCone();
		Set<ComplexHandleAnnotation> complexHandle = cas
				.findAnnotations(ComplexHandleAnnotation.class);
		List<HandleAnnotation> allAnnotations = new ArrayList<HandleAnnotation>(cones.size()
				+ complexHandle.size());
		for (ConeAnnotation c : cones) {
			if (c.isConcave())
				continue;
			allAnnotations.add(c);
		}
		allAnnotations.addAll(complexHandle);
		cones.clear();
		complexHandle.clear();
		HandleAnnotation[] handleList = allAnnotations.toArray(new HandleAnnotation[0]);
		Arrays.sort(handleList, new HandleComparator(cas.getModel(), minRadius, maxRadius,
				minLength, maxLength));
		ArrayList<HandleAnnotation> rets = new ArrayList<HandleAnnotation>();
		for (HandleAnnotation h : handleList) {
			if (HandleComparator.getHandleWeight(h, cas.getModel(),
					HandleComparator.DEFAULT_RADIUS_MIN, HandleComparator.DEFAULT_RADIUS_MAX,
					HandleComparator.DEFAULT_LENGTH_MIN, HandleComparator.DEFAULT_LENGTH_MAX) < HandleComparator.MIN_WEIGHT_FOR_HANDLE)
				break;
			rets.add(h);
		}
		return rets.toArray(new HandleAnnotation[0]);
	}

	/**
	 * Highlight specified annotation in mesh reasoning view
	 * 
	 * @param a
	 *            Annotation to highlight
	 */
	public void highlightAnnotation(DrawableAnnotation a) {
		highlightAnnotation(a, null);
	}

	/**
	 * Highlight specified annotation in mesh reasoning view
	 * 
	 * @param a
	 *            Annotation to highlight
	 */
	public void highlightAnnotation(DrawableAnnotation a, Color color) {
		if (mrv == null)
			return;
		mrv.addSelectedAnnotation(a, color);
	}

	private void initImageGenerator() {
		imageGeneratorSettings.setDrawAxis(false);
		imageGeneratorSettings.setWhiteBackground(true);
		// If there is a view settings file, load those settings
		imageGeneratorSettings.setInitViewFromFile(true);
		// If no view settings exist yet, allow to align model and then continue by pressing Ctrl+S
		imageGeneratorSettings.setSaveView(true);
		imageGeneratorSettings.setSavePlainModel(true);
		imageGeneratorSettings.setSaveCurvatureColor(true);
		imageGeneratorSettings.setCloseAfterFinish(true);
		imageGeneratorSettings.addAnalyserToSave(PrimitiveAnalyser.class, "segmented");

		/**
		 * Analyser actions begin
		 */
		final MeshReasoning mr = this;

		imageGeneratorSettings.clearAnalyserActions();
		imageGeneratorSettings.addAnalyserAction(PrimitiveAnalyser.class,
				new ImageGeneratorAction() {

					@Override
					public void trigger(ImageGeneratorSettings localSettings) {
						mr.clearHightlight();
						@SuppressWarnings("rawtypes")
						PrimitiveAnnotation[] cones = mr.findAnnotationsCone().toArray(
								new PrimitiveAnnotation[0]);
						Arrays.sort(cones, new PrimitiveAnnotationAreaComparator());
						int max = 3;
						for (int i = 0; i < max && i < cones.length; i++) {
							mr.highlightAnnotation(cones[i]);
							try {
								Thread.sleep(200);
							} catch (InterruptedException e) {} // wait til selected
							localSettings.waitSaved("cones" + (i + 1));
							mr.clearHightlight();
						}
					}
				});
		imageGeneratorSettings.addAnalyserAction(PrimitiveAnalyser.class,
				new ImageGeneratorAction() {

					@Override
					public void trigger(ImageGeneratorSettings localSettings) {
						mr.clearHightlight();
						@SuppressWarnings("rawtypes")
						PrimitiveAnnotation[] planes = mr.findAnnotationsPlane().toArray(
								new PrimitiveAnnotation[0]);
						Arrays.sort(planes, new PrimitiveAnnotationAreaComparator());
						int max = 3;
						for (int i = 0; i < max && i < planes.length; i++) {
							mr.highlightAnnotation(planes[i]);
							try {
								Thread.sleep(200);
							} catch (InterruptedException e) {} // wait til selected
							localSettings.waitSaved("planes" + (i + 1));
							mr.clearHightlight();
						}
					}
				});
		imageGeneratorSettings.addAnalyserAction(PrimitiveAnalyser.class,
				new ImageGeneratorAction() {

					@Override
					public void trigger(ImageGeneratorSettings localSettings) {
						mr.clearHightlight();
						@SuppressWarnings("rawtypes")
						PrimitiveAnnotation[] spheres = mr.findAnnotationsSphere().toArray(
								new PrimitiveAnnotation[0]);
						Arrays.sort(spheres, new PrimitiveAnnotationAreaComparator());
						int max = 3;
						for (int i = 0; i < max && i < spheres.length; i++) {
							mr.highlightAnnotation(spheres[i]);
							try {
								Thread.sleep(200);
							} catch (InterruptedException e) {} // wait til selected
							localSettings.waitSaved("spheres" + (i + 1));
							mr.clearHightlight();
						}
					}
				});

		imageGeneratorSettings.addAnalyserAction(ContainerAnalyser.class,
				new ImageGeneratorAction() {

					@Override
					public void trigger(ImageGeneratorSettings localSettings) {
						mr.clearHightlight();
						ContainerAnnotation[] container = mr.findAnnotations(
								ContainerAnnotation.class).toArray(new ContainerAnnotation[0]);
						Arrays.sort(container, new ContainerAnnotationVolumeComarator());
						int max = 3;
						for (int i = 0; i < max && i < container.length; i++) {
							mr.highlightAnnotation(container[i]);
							try {
								Thread.sleep(200);
							} catch (InterruptedException e) {} // wait til selected
							localSettings.waitSaved("container" + (i + 1));
							mr.clearHightlight();
						}
					}
				});

		imageGeneratorSettings.addAnalyserAction(ComplexHandleAnalyser.class,
				new ImageGeneratorAction() {

					@Override
					public void trigger(ImageGeneratorSettings localSettings) {
						mr.clearHightlight();
						HandleAnnotation[] handles = mr.getHandle(
								HandleComparator.DEFAULT_RADIUS_MIN,
								HandleComparator.DEFAULT_RADIUS_MAX,
								HandleComparator.DEFAULT_LENGTH_MIN,
								HandleComparator.DEFAULT_LENGTH_MAX);
						int max = 3;
						for (int i = 0; i < max && i < handles.length; i++) {
							mr.highlightAnnotation((DrawableAnnotation) handles[i]);
							try {
								Thread.sleep(200);
							} catch (InterruptedException e) {} // wait til selected
							localSettings.waitSaved("handle" + (i + 1));
							mr.clearHightlight();
						}
					}
				});

		/**
		 * Analyser actions end
		 */
	}

	private void initImageGeneratorSpoon() {
		imageGeneratorSettings.setDrawAxis(false);
		imageGeneratorSettings.setWhiteBackground(true);
		// If there is a view settings file, load those settings
		imageGeneratorSettings.setInitViewFromFile(true);
		// If no view settings exist yet, allow to align model and then continue by pressing Ctrl+S
		imageGeneratorSettings.setSaveView(true);
		imageGeneratorSettings.setSavePlainModel(true);
		imageGeneratorSettings.setSaveCurvatureColor(true);
		imageGeneratorSettings.setCloseAfterFinish(true);
		// even draw fitted cone/sphere/plane if more than one annotation selected
		imageGeneratorSettings.setAlwaysDrawSelectedPrimitives(true);

		/**
		 * Analyser actions begin
		 */
		final MeshReasoning mr = this;

		imageGeneratorSettings.clearAnalyserActions();
		imageGeneratorSettings.addAnalyserAction(PrimitiveAnalyser.class,
				new ImageGeneratorAction() {

					@Override
					public void trigger(ImageGeneratorSettings localSettings) {
						mr.clearHightlight();
						@SuppressWarnings("rawtypes")
						PrimitiveAnnotation[] cones = mr.findAnnotationsCone().toArray(
								new PrimitiveAnnotation[0]);
						Arrays.sort(cones, new PrimitiveAnnotationAreaComparator());

						@SuppressWarnings("rawtypes")
						PrimitiveAnnotation[] spheres = mr.findAnnotationsSphere().toArray(
								new PrimitiveAnnotation[0]);
						Arrays.sort(spheres, new PrimitiveAnnotationAreaComparator());

						if (cones.length == 0) {
							logger.warn("Skipping image because no cone found");
							return;
						}
						if (spheres.length == 0) {
							logger.warn("Skipping image because no sphere found");
							return;
						}
						for (int i = 0; i < cones.length; i++) {
							if (!((ConeAnnotation) cones[i]).isConcave()) {
								mr.highlightAnnotation(cones[i]);
								break;
							}
						}
						for (int i = 0; i < spheres.length; i++) {
							if (((SphereAnnotation) spheres[i]).isConcave()) {
								mr.highlightAnnotation(spheres[i]);
								break;
							}
						}
						try {
							Thread.sleep(200);
						} catch (InterruptedException e) {} // wait til selected
						localSettings.waitSaved("ConeSphere");
						mr.clearHightlight();

					}
				});

		/**
		 * Analyser actions end
		 */
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
