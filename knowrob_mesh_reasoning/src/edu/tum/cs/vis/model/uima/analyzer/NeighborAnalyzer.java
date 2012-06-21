package edu.tum.cs.vis.model.uima.analyzer;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicInteger;

import org.apache.log4j.Logger;

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

	private static Logger				logger				= Logger.getLogger(NeighborAnalyzer.class);

	private final List<Callable<Void>>	threads				= new LinkedList<Callable<Void>>();

	private final AtomicInteger			polygonsElaborated	= new AtomicInteger(0);

	private ArrayList<Polygon>			allPolygons;

	@Override
	public Logger getLogger() {
		return logger;
	}

	@Override
	public String getName() {
		return "Neighbor";
	}

	public void polygonsElaborated(int cnt) {
		polygonsElaborated.addAndGet(cnt);
	}

	/**
	 * Process a group which contains a mesh.
	 * 
	 * @param g
	 *            group to process
	 */
	private void processGroup(final Group g) {

		processMesh(g.getMesh());

		for (Group gr : g.getChildren()) {
			processGroup(gr);
		}
	}

	/**
	 * Process a mesh which contains polygons and find the neighbors for each triangle.
	 * 
	 * @param m
	 *            Mesh to process
	 */
	void processMesh(final Mesh m) {
		if (m.getPolygons().size() == 0)
			return;

		allPolygons.addAll(m.getPolygons());

	}

	@Override
	public void processStart(MeshCas cas) {
		allPolygons = new ArrayList<Polygon>();
		processGroup(cas.getGroup());

		int startIdx = 0;
		int interval = 100;

		do {
			threads.add(new NeighborAnalyzerThread(startIdx, Math.min(startIdx + interval,
					allPolygons.size()), allPolygons, this));
			startIdx += interval;
		} while (startIdx < allPolygons.size());

		int threadNum = Runtime.getRuntime().availableProcessors() * 25;

		logger.debug("All Threads initialized. Starting Pool with " + threadNum + " threads ...");
		ExecutorService pool = Executors.newFixedThreadPool(threadNum);

		try {
			pool.invokeAll(threads);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	@Override
	public void updateProgress() {
		if (allPolygons != null)
			setProgress((float) polygonsElaborated.get() / (float) allPolygons.size() * 100.0f);
	}
}

class NeighborAnalyzerThread implements Callable<Void> {

	final int						start;
	final int						end;
	final ArrayList<Polygon>		polygons;

	private static Logger			logger	= Logger.getLogger(NeighborAnalyzerThread.class);
	private final NeighborAnalyzer	analyzer;

	public NeighborAnalyzerThread(int start, int end, ArrayList<Polygon> polygons,
			NeighborAnalyzer analyzer) {
		this.start = start;
		this.end = end;
		this.polygons = polygons;
		this.analyzer = analyzer;
	}

	@Override
	public Void call() throws Exception {
		for (int i = start; i < end; i++) {
			Polygon tr = polygons.get(i);
			for (int j = i + 1; j < polygons.size(); j++) {
				Polygon n = polygons.get(j);
				n.addNeighbor(tr);
			}
		}
		analyzer.polygonsElaborated(end - start);
		return null;
	}

}
