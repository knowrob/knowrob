package edu.tum.cs.test;

import javax.swing.JFrame;

import edu.tum.cs.vis.model.ItemModel;
import edu.tum.cs.vis.model.MeshReasoningView;
import edu.tum.cs.vis.model.uima.analyzer.FlatSurfaceAnalyzer;
import edu.tum.cs.vis.model.uima.analyzer.NeighborAnalyzer;
import edu.tum.cs.vis.model.uima.cas.MeshCas;

/**
 * Main test class for testing MeshReasoning
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshReasoning {

	/**
	 * Main Method loading the mesh, drawing it and starting the analyzer
	 * 
	 * @param args
	 *            command line arguments
	 */
	public static void main(String[] args) {
		// TODO Auto-generated method stub

		ItemModel model = new ItemModel("/home/stefan/simple_dome.kmz");
		// ItemModel model = new ItemModel("/home/stefan/Downloads/triangle.kmz");
		if (!model.parseModel()) {
			throw new RuntimeException("Couldn't parse model. Maybe path to model is wrong.");
		}

		MeshCas cas = new MeshCas();

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

		cas.setGroup(model.getParser().getGroup());
		// cas.setGroup(g);

		JFrame frame = new JFrame();
		frame.setSize(1024, 768);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		MeshReasoningView mrv = new MeshReasoningView();
		mrv.init();

		mrv.getCasList().add(cas);

		frame.add(mrv);
		frame.setVisible(true);

		NeighborAnalyzer na = new NeighborAnalyzer();
		na.process(cas);

		FlatSurfaceAnalyzer fsa = new FlatSurfaceAnalyzer();
		fsa.process(cas);

	}

}
