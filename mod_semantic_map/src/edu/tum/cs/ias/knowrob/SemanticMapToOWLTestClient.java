package edu.tum.cs.ias.knowrob;

import ros.*;
import ros.pkg.mod_semantic_map.srv.*;
import ros.pkg.mod_semantic_map.msg.*;


/**
 * 
 * Interface between the CoP cognitive perception system (http://www.ros.org/wiki/cop)
 * and the RoboEarth database.
 * 
 * Provides ROS services to upload CoP object models to the RoboEarth database as well
 * as to retrieve object models from the database and add them to CoP.
 * 
 * @author Moritz Tenorth, tenorth@cs.tum.edu
 *
 */

public class SemanticMapToOWLTestClient {

	
	////////////////////////////////////////////////////////////////////////////////
	// ROS stuff
	
	static Boolean rosInitialized = false;
	static Ros ros;
	static NodeHandle n;
	
	////////////////////////////////////////////////////////////////////////////////

		

	public static void main(String args[]) {
		  
		Ros ros = Ros.getInstance();
		if(!Ros.getInstance().isInitialized()) {
			ros.init("semantic_map_to_owl_test_client");
		}
		NodeHandle n = ros.createNodeHandle();

		
		// create map

		GenerateSemanticMapOWL.Request req = new GenerateSemanticMapOWL.Request();
		req.map= new SemMap();
		req.map.header.frame_id="/map";
		req.map.header.stamp=ros.now();
		
		req.map.objects = new SemMapObject[4];
		
		// create cupboard
		req.map.objects[0] = new SemMapObject();

		req.map.objects[0].id=1;
		req.map.objects[0].partOf=0;
		req.map.objects[0].type="cupboard";

		req.map.objects[0].depth  = 55.6f;
		req.map.objects[0].width  = 60.7f;
		req.map.objects[0].height = 70.6f;

		req.map.objects[0].pose = new float[] { 1.0f,0.0f,0.0f,2.3f,
											0.0f,1.0f,0.0f,1.2f,
											0.0f,0.0f,1.0f,0.4f,
											0.0f,0.0f,0.0f,1.0f};		
		// create door
		req.map.objects[1] = new SemMapObject();

		req.map.objects[1].id=2;
		req.map.objects[1].partOf=1;
		req.map.objects[1].type="door";

		req.map.objects[1].depth  = 0.6f;
		req.map.objects[1].width  = 60.7f;
		req.map.objects[1].height = 70.6f;

		req.map.objects[1].pose = new float[] { 1.0f,0.0f,0.0f,2.4f,
											0.0f,1.0f,0.0f,1.3f,
											0.0f,0.0f,1.0f,0.4f,
											0.0f,0.0f,0.0f,1.0f};		
		// create hinge
		req.map.objects[2] = new SemMapObject();

		req.map.objects[2].id=3;
		req.map.objects[2].partOf=2;
		req.map.objects[2].type="hinge";

		req.map.objects[2].depth  = 5.6f;
		req.map.objects[2].width  = 0.7f;
		req.map.objects[2].height = 0.6f;

		req.map.objects[2].pose = new float[] { 1.0f,0.0f,0.0f,2.5f,
											0.0f,1.0f,0.0f,1.4f,
											0.0f,0.0f,1.0f,0.4f,
											0.0f,0.0f,0.0f,1.0f};
		
		// create handle
		req.map.objects[3] = new SemMapObject();

		req.map.objects[3].id=4;
		req.map.objects[3].partOf=2;
		req.map.objects[3].type="handle";

		req.map.objects[3].depth  = 5.6f;
		req.map.objects[3].width  = 6.7f;
		req.map.objects[3].height = 7.6f;

		req.map.objects[3].pose = new float[] { 1.0f,0.0f,0.0f,2.6f,
											0.0f,1.0f,0.0f,1.5f,
											0.0f,0.0f,1.0f,0.4f,
											0.0f,0.0f,0.0f,1.0f};

				ServiceClient<GenerateSemanticMapOWL.Request, GenerateSemanticMapOWL.Response, GenerateSemanticMapOWL> cl = 
							n.serviceClient("/generate_owl_map", new GenerateSemanticMapOWL());
		
		try {
			GenerateSemanticMapOWL.Response res = cl.call(req);
			System.out.println("Map:\n\n " +res.owlmap);
			
		} catch (RosException e) {
			e.printStackTrace();
		}

	}
}
