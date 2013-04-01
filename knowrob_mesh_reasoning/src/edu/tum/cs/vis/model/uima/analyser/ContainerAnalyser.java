/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyser;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import edu.tum.cs.ias.knowrob.utils.ThreadPool;
import edu.tum.cs.vis.model.uima.annotation.ContainerAnnotation;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * Mesh analyzer for detecting containers based on previously found primitive annotations. Uses
 * already found primitive annotations to detect containers and then create new container
 * annotations on mesh reasoning object.
 * 
 * @author Stefan Profanter
 * 
 */
public class ContainerAnalyser extends MeshAnalyser {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger	= Logger.getLogger(ContainerAnalyser.class);

	/**
	 * Check if given cone annotation represents a container by checking if cone annotation has
	 * either a bottom cap or top cap
	 * 
	 * @param cas
	 *            main mesh cas
	 * @param an
	 *            concave cone annotation to check if it is a container annotation
	 * @param annotations
	 *            list of all primitive annotations of object
	 */
	static void checkConcaveAnnotationCone(MeshCas cas, ConeAnnotation an,
			@SuppressWarnings("rawtypes") HashSet<PrimitiveAnnotation> annotations) {

		// expected areas of top and bottom cap
		float areaOfCapSmall = (float) (Math.pow(an.getRadiusSmall(), 2) * Math.PI);
		float areaOfCapLarge = (float) (Math.pow(an.getRadiusLarge(), 2) * Math.PI);
		float heightHalf = an.getDirection().length();

		// get two points to create a ray aligned to cone direction
		Point3f topCapCenter = new Point3f(an.getCentroid());
		topCapCenter.add(an.getDirection());

		Point3f centroid = new Point3f(an.getCentroid());

		boolean hasTopCap = false;
		boolean hasBottomCap = false;

		// previous propAbs value
		float prevPropAbs = Float.MAX_VALUE;

		PlaneAnnotation capAnnotation = null;

		for (MeshAnnotation<PlaneAnnotation> a : annotations) {
			if (a instanceof PlaneAnnotation) {
				PlaneAnnotation pa = (PlaneAnnotation) a;

				Point3f intersect = new Point3f();
				boolean isIntersect = false;
				// Get triangle which intersects ray between centroid and topCapCenter
				for (Triangle t : pa.getMesh().getTriangles()) {
					if (t.intersectsRay(topCapCenter, centroid, intersect)) {
						isIntersect = true;
						break;
					}
				}
				if (!isIntersect) {
					continue;
				}

				// intersection point found. Check distance:
				Vector3f midToIntersect = new Vector3f(intersect);
				midToIntersect.sub(an.getCentroid());
				float distToIntersect = midToIntersect.length();

				float proportion = distToIntersect / heightHalf;

				float propAbs = Math.abs(1f - proportion);

				if (propAbs > 0.3 || propAbs > prevPropAbs)
					continue; // plane isn't at the top or bottom of the cylinder or is not as near
								// as previous found

				if (midToIntersect.dot(an.getDirection()) > 0) {
					// is top
					if (pa.getArea() / areaOfCapSmall > 0.7) {
						hasTopCap = true;
						capAnnotation = pa;
						prevPropAbs = propAbs;
					}
				} else {
					// is bottom
					if (pa.getArea() / areaOfCapLarge > 0.7) {
						hasBottomCap = true;
						capAnnotation = pa;
						prevPropAbs = propAbs;
					}
				}

				if (hasBottomCap && hasTopCap)
					return; // isn't container. Is a closed box.
			}
		}
		if ((hasBottomCap || hasTopCap) && capAnnotation != null) {
			// it is container, so create annotation
			ContainerAnnotation ca = new ContainerAnnotation(cas.getModel());
			synchronized (ca.getMesh().getTriangles()) {
				ca.getMesh().getTriangles().addAll(an.getMesh().getTriangles());
				ca.getMesh().getTriangles().addAll(capAnnotation.getMesh().getTriangles());
			}

			ca.setDirection((Vector3f) an.getDirection().clone());

			if (hasTopCap) {
				ca.getDirection().scale(-1);
			}

			ca.setVolume(an.getVolume());

			cas.addAnnotation(ca);
		}
	}

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current progress
	 */
	final AtomicInteger	itemsElaborated		= new AtomicInteger(0);

	/**
	 * maximum items to elaborate. Used for indicating current progress
	 */
	private int			maxItemsToElaborate	= 0;

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#getLogger()
	 */
	@Override
	public Logger getLogger() {
		return logger;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#getName()
	 */
	@Override
	public String getName() {
		return "Container";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(final MeshCas cas) {
		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();

		@SuppressWarnings("rawtypes")
		final HashSet<PrimitiveAnnotation> annotations = cas
				.findAnnotations(PrimitiveAnnotation.class);

		for (@SuppressWarnings("rawtypes")
		final MeshAnnotation a : annotations) {
			if (a instanceof ConeAnnotation && ((ConeAnnotation) a).isConcave()) {
				maxItemsToElaborate++;
				threads.add(new Callable<Void>() {

					@Override
					public Void call() throws Exception {
						checkConcaveAnnotationCone(cas, (ConeAnnotation) a, annotations);

						itemsElaborated.incrementAndGet();
						return null;
					}
				});
			}
			// TODO Add container finding for half-spheres or rounded boxes or simple boxes
		}

		ThreadPool.executeInPool(threads);
		// increment, because if no containers, 1 indicates that
		// it has finished
		maxItemsToElaborate = 1;
		itemsElaborated.incrementAndGet();

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (maxItemsToElaborate > 0) {
			setProgress(itemsElaborated.get() / (float) (maxItemsToElaborate) * 100.0f);
		}

	}

}
