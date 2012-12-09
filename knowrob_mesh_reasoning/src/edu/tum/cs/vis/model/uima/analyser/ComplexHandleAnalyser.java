/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyser;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.concurrent.atomic.AtomicInteger;

import javax.vecmath.Vector3f;

import org.apache.log4j.Logger;

import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.ComplexHandleAnnotation;
import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Triangle;
import edu.tum.cs.vis.model.util.Vertex;

/**
 * @author Stefan Profanter
 * 
 */
public class ComplexHandleAnalyser extends MeshAnalyser {
	/**
	 * Log4J Logger
	 */
	private static Logger	logger	= Logger.getLogger(ComplexHandleAnalyser.class);

	private static boolean isConvexRelation(Triangle t1, Triangle t2) {

		Vertex notShared = null;
		// Get the point of t2 which isn't on the shared edge
		for (Vertex v : t2.getPosition()) {
			notShared = v;
			for (Vertex v1 : t1.getPosition()) {
				if (v1.sameCoordinates(v)) {
					notShared = null;
					break;
				}
			}
			if (notShared != null)
				break;
		}

		if (notShared == null)
			return false;

		Vector3f vec = new Vector3f(notShared);

		// Now determine if the triangles are convex or concave
		vec.sub(t1.getCentroid());
		vec.normalize();

		if (t1.getNormalVector().dot(vec) > 0)
			return false; // triangles are concave

		// angle is between 0 and PI
		double dot = t1.getNormalVector().dot(t2.getNormalVector());
		// Make sure dot is between -1 and 1, then calculate acos
		double angle = Math.acos(Math.max(-1, Math.min(1, dot)));

		return angle >= 0.0 / 180.0 * Math.PI;
		// return true;
	}

	/**
	 * Number of currently processed triangles. Used for progress status.
	 */
	final AtomicInteger	trianglesElaborated	= new AtomicInteger(0);

	/**
	 * When calling <code>process</code> all triangles of the group and its children are collected
	 * in this list to process them afterwards.
	 */
	ArrayList<Triangle>	allTriangles;

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
		return "ComplexHandle";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@Override
	public void processStart(final MeshCas cas) {
		trianglesElaborated.set(0);
		allTriangles = cas.getModel().getTriangles();

		final HashMap<Triangle, ComplexHandleAnnotation> alreadyAdded = new HashMap<Triangle, ComplexHandleAnnotation>(
				allTriangles.size());

		for (int i = 0; i < allTriangles.size(); i++) {

			Triangle t = allTriangles.get(i);
			ComplexHandleAnnotation an;
			an = alreadyAdded.get(t);
			if (an == null) {
				an = new ComplexHandleAnnotation(cas.getCurvatures(), cas.getModel());
				alreadyAdded.put(t, an);
				synchronized (an.getMesh().getTriangles()) {
					// Drawing would throw exception with concurrent modification
					an.getMesh().getTriangles().add(t);
				}
				cas.addAnnotation(an);
			}

			regionGrowTriangle(cas, alreadyAdded, t, an);
			trianglesElaborated.addAndGet(1);
		}

		// Delete annotations which have only one triangle
		synchronized (cas.getAnnotations()) {
			Iterator<Annotation> it = cas.getAnnotations().listIterator();

			while (it.hasNext()) {
				Annotation a = it.next();
				if (!(a instanceof ComplexHandleAnnotation))
					continue;
				ComplexHandleAnnotation cha = (ComplexHandleAnnotation) a;
				if (cha.getMesh().getTriangles().size() <= 1)
					it.remove();
				else
					cha.fit();
			}

		}
		trianglesElaborated.addAndGet(1);

		updateProgress();

	}

	void regionGrowTriangle(MeshCas cas, HashMap<Triangle, ComplexHandleAnnotation> alreadyAdded,
			Triangle t, ComplexHandleAnnotation an) {

		ComplexHandleAnnotation currentAnnotation = an;

		for (Triangle n : t.getNeighbors()) {
			// Avoid parallel elaboration of same triangle
			ComplexHandleAnnotation neighborAn = null;
			if (isConvexRelation(t, n)) {
				neighborAn = alreadyAdded.get(n);

				if (neighborAn == currentAnnotation)
					continue; // already in same annotation

				if (neighborAn == null) {
					// add to this annotation
					synchronized (currentAnnotation.getMesh().getTriangles()) {
						currentAnnotation.getMesh().getTriangles().add(n);
					}
					alreadyAdded.put(n, currentAnnotation);
					continue;
				}

				// otherwise merge annotations
				neighborAn.merge(currentAnnotation);
				synchronized (currentAnnotation.getMesh().getTriangles()) {
					for (Triangle remap : currentAnnotation.getMesh().getTriangles())
						alreadyAdded.put(remap, neighborAn);
				}
				cas.removeAnnotation(currentAnnotation);
				currentAnnotation = neighborAn;
			}
		}
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#updateProgress()
	 */
	@Override
	public void updateProgress() {
		if (allTriangles != null)
			setProgress((float) trianglesElaborated.get() / (float) (allTriangles.size() + 1)
					* 100.0f);

	}

}
