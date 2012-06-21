/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyzer;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.PlaneAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.ThreadPool;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public class ContainerAnalyzer extends MeshAnalyzer {

	/**
	 * Log4J Logger
	 */
	private static Logger	logger				= Logger.getLogger(PrimitiveAnalyzer.class);

	/**
	 * Number of triangles already elaborated/processed. Used for indicating current process
	 */
	final AtomicInteger		itemsElaborated		= new AtomicInteger(0);

	private int				maxItemsToElaborate	= 0;

	void checkConcavAnnotationCone(ConeAnnotation an, HashSet<MeshAnnotation> annotations) {

		float areaOfCapSmall = (float) (Math.pow(an.getRadiusSmall(), 2) * Math.PI);
		float areaOfCapLarge = (float) (Math.pow(an.getRadiusLarge(), 2) * Math.PI);
		Vector3f midPointSmall = (Vector3f) an.getCentroid().clone();
		midPointSmall.sub(an.getDirection());
		Vector3f midPointLarge = (Vector3f) an.getCentroid().clone();
		midPointLarge.add(an.getDirection());
		float heightHalf = an.getDirection().length();

		boolean hasTopCap = false;
		boolean hasBottomCap = false;

		for (MeshAnnotation a : annotations) {
			if (a instanceof PlaneAnnotation) {
				PlaneAnnotation pa = (PlaneAnnotation) a;
				Triangle t1 = new Triangle();
				t1.getPosition()[0] = new Vertex(pa.getCorner()[0]);
				t1.getPosition()[1] = new Vertex(pa.getCorner()[1]);
				t1.getPosition()[2] = new Vertex(pa.getCorner()[2]);
				Triangle t2 = new Triangle();
				t2.getPosition()[0] = new Vertex(pa.getCorner()[0]);
				t2.getPosition()[1] = new Vertex(pa.getCorner()[1]);
				t2.getPosition()[2] = new Vertex(pa.getCorner()[2]);

				Point3f intersect = new Point3f();
				if (!t1.intersectsRay(new Point3f(midPointSmall), new Point3f(midPointLarge),
						intersect)) {
					if (!t2.intersectsRay(new Point3f(midPointSmall), new Point3f(midPointLarge),
							intersect)) {
						continue;
					}
				}

				// intersection point found. Check distance:
				Vector3f midToIntersect = new Vector3f(intersect);
				midToIntersect.sub(an.getCentroid());
				float distToIntersect = midToIntersect.length();

				float proportion = distToIntersect / heightHalf;

				if (proportion > 1.3 || proportion < 0.7)
					continue; // plane isn't at the top or bottom of the cylinder

				if (midToIntersect.dot(an.getDirection()) > 0) {
					// use top
					if (!hasTopCap) {
						hasTopCap = (pa.getArea() / areaOfCapSmall > 0.7);
					}
				} else {
					if (!hasBottomCap) {
						hasBottomCap = (pa.getArea() / areaOfCapLarge > 0.7);
					}
				}

				if (hasBottomCap && hasTopCap)
					break;
			}
		}
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#getLogger()
	 */
	@Override
	public Logger getLogger() {
		return logger;
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#getName()
	 */
	@Override
	public String getName() {
		return "Container";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(MeshCas cas) {
		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();

		final HashSet<MeshAnnotation> annotations = cas.findAnnotations(PrimitiveAnnotation.class);

		for (final MeshAnnotation a : annotations) {
			if (a instanceof ConeAnnotation && ((ConeAnnotation) a).isConcav()) {
				maxItemsToElaborate++;
				threads.add(new Callable<Void>() {

					@Override
					public Void call() throws Exception {
						checkConcavAnnotationCone((ConeAnnotation) a, annotations);

						itemsElaborated.incrementAndGet();
						return null;
					}
				});
			} // TODO Add container finding for half-spheres or rounded boxes or simple boxes
		}

		ThreadPool.executeInPool(threads);

	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyzer.MeshAnalyzer#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (maxItemsToElaborate > 0) {
			setProgress(itemsElaborated.get() / (float) (maxItemsToElaborate) * 100.0f);
		}

	}

}
