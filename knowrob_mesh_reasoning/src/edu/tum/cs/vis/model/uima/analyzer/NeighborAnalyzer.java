package edu.tum.cs.vis.model.uima.analyzer;

import edu.tum.cs.vis.model.uima.cas.MeshCas;
import edu.tum.cs.vis.model.util.Group;
import edu.tum.cs.vis.model.util.Mesh;
import edu.tum.cs.vis.model.util.Polygon;

/**
 * Analyzer for a mesh which sets direct neighbors of a triangle.
 * 
 * The neighbor information is used in other Analyzer for better performance.
 * 
 * @author Stefan Profanter
 * 
 */
public class NeighborAnalyzer extends MeshAnalyzer {

	/**
	 * Process a mesh which contains polygons and find the neighbors for each triangle.
	 * 
	 * @param m
	 *            Mesh to process
	 */
	private static void processMesh(Mesh m) {
		// Iterate over the polygons and find direct neighbors
		for (Polygon tr : m.getPolygons()) {
			for (Polygon n : m.getPolygons()) {
				if (n == tr)
					continue;
				if (n.isNeighbor(tr))
					n.addNeighbor(tr);
			}
		}
	}

	@Override
	public void process(MeshCas cas) {
		processGroup(cas.getGroup());
	}

	/**
	 * Process a group which contains a mesh.
	 * 
	 * @param g
	 *            group to process
	 */
	private void processGroup(Group g) {
		processMesh(g.getMesh());
		for (Group gr : g.getChildren()) {
			processGroup(gr);
		}
	}

}
