/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter. All rights reserved. This program and the accompanying
 * materials are made available under the terms of the GNU Public License v3.0 which accompanies
 * this distribution, and is available at http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors: Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.vis.model.uima.analyser;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.log4j.Logger;

import com.google.common.collect.HashMultimap;

import edu.tum.cs.ias.knowrob.utils.ThreadPool;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.vis.model.uima.annotation.ComplexHandleAnnotation;
import edu.tum.cs.vis.model.uima.annotation.PrimitiveAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.ConeAnnotation;
import edu.tum.cs.vis.model.uima.annotation.primitive.SphereAnnotation;
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

	/**
	 * Tries to find complex handle by region growing. Neighboring annotations are combined if their
	 * angle is smaller than 180 degree to form a convex handle.
	 * 
	 * @param cas
	 *            MeshCas
	 * @param pa
	 *            Annotation from where to start region growing.
	 * @param alreadyAdded
	 *            Map of already added annotations and their complex handle
	 * @return the new complex handle
	 */
	@SuppressWarnings({ "rawtypes", "unchecked" })
	private static ComplexHandleAnnotation regionGrowAnnotation(MeshCas cas,
			PrimitiveAnnotation pa,
			HashMap<PrimitiveAnnotation, ComplexHandleAnnotation> alreadyAdded) {
		Set<PrimitiveAnnotation> neighbors = pa.getNeighborAnnotations(cas,
				PrimitiveAnnotation.class);
		ComplexHandleAnnotation newHandle = null;
		if (alreadyAdded.containsKey(pa)) {
			newHandle = alreadyAdded.get(pa);
		}

		for (PrimitiveAnnotation n : neighbors) {
			if (n instanceof SphereAnnotation && ((SphereAnnotation) n).isConcave())
				continue;
			if (n instanceof ConeAnnotation && ((ConeAnnotation) n).isConcave())
				continue;

			HashMultimap<Triangle, Triangle> edgeTriangles = HashMultimap.create(n.getMesh()
					.getTriangles().size() / 3, 2);
			Set<Vertex> edgeVertices = new HashSet<Vertex>();
			pa.getNeighborEdge(cas, n, edgeVertices, edgeTriangles);

			// check for a triangle pair where the angle between triangle normals is bigger or equal
			// to 180 degree.
			for (Triangle t : edgeTriangles.keySet()) {
				Set<Triangle> partnerSet = edgeTriangles.get(t);

				for (Triangle partner : partnerSet) {

					float dot = t.getNormalVector().dot(partner.getNormalVector());
					// allow a small error
					if (dot > ComplexHandleAnnotation.DOT_NORMAL_TOLERANCE) {
						// angle is smaller than 180 degree
						continue;
					}
					// we have found two triangles, let's merge the two annotations into a complex
					// handle

					if (newHandle == null) {
						newHandle = new ComplexHandleAnnotation(cas.getModel());
						alreadyAdded.put(pa, newHandle);
						newHandle.addAnnotation(pa);
					}

					// first check if neighbor annotation is already a part of complex handle
					if (alreadyAdded.containsKey(n)) {
						ComplexHandleAnnotation neighborHandle = alreadyAdded.get(n);
						if (neighborHandle == newHandle)
							continue;
						if (!newHandle.allowMerge(neighborHandle))
							continue;
						newHandle.merge(neighborHandle);
						alreadyAdded.remove(n);

						// update reference list
						for (PrimitiveAnnotation an : neighborHandle.getPrimitiveAnnotations()) {
							alreadyAdded.put(an, newHandle);
						}
					} else {
						newHandle.addAnnotation(n);
					}
					alreadyAdded.put(n, newHandle);
					break;
				}
			}
		}

		return newHandle;
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
		return "ComplexHandle";
	}

	/* (non-Javadoc)
	 * @see edu.tum.cs.vis.model.uima.analyser.MeshAnalyser#processStart(edu.tum.cs.vis.model.uima.cas.MeshCas)
	 */
	@SuppressWarnings("rawtypes")
	@Override
	public void processStart(final MeshCas cas) {

		HashMap<PrimitiveAnnotation, ComplexHandleAnnotation> alreadyAdded = new HashMap<PrimitiveAnnotation, ComplexHandleAnnotation>();
		maxItemsToElaborate = cas.getAnnotations().size() + 1;
		itemsElaborated.set(0);
		for (Annotation a : cas.getAnnotations()) {
			itemsElaborated.incrementAndGet();
			if (!(a instanceof PrimitiveAnnotation))
				continue;
			if (a instanceof SphereAnnotation && ((SphereAnnotation) a).isConcave())
				continue;
			if (a instanceof ConeAnnotation && ((ConeAnnotation) a).isConcave())
				continue;
			// We only need planes and convex sphere or cone
			PrimitiveAnnotation pa = (PrimitiveAnnotation) a;
			regionGrowAnnotation(cas, pa, alreadyAdded);

		}

		// Make sure that no annotation is inserted multiple times
		final Set<ComplexHandleAnnotation> toAdd = new HashSet<ComplexHandleAnnotation>();
		toAdd.addAll(alreadyAdded.values());

		final List<ComplexHandleAnnotation> failedFittings = new LinkedList<ComplexHandleAnnotation>();

		List<Callable<Void>> threads = new LinkedList<Callable<Void>>();
		threads.add(new Callable<Void>() {

			@Override
			public Void call() throws Exception {

				for (ComplexHandleAnnotation cha : toAdd) {
					if (!(cha.fit())) {
						synchronized (failedFittings) {
							failedFittings.add(cha);
						}
					}
				}

				return null;
			}
		});
		ThreadPool.executeInPool(threads);
		if (failedFittings.size() > 0) {
			logger.debug("Ignoring complex handles where fitting failed (" + failedFittings.size()
					+ ")");
			toAdd.removeAll(failedFittings);
		}

		synchronized (cas.getAnnotations()) {
			cas.getAnnotations().addAll(toAdd);
		}

		itemsElaborated.incrementAndGet();
		updateProgress();

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
