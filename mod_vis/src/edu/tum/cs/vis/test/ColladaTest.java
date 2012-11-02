package edu.tum.cs.vis.test;


import java.util.LinkedList;

import javax.vecmath.Vector3f;


import processing.core.*;

import peasy.*;

import edu.tum.cs.vis.model.ItemModel;
import edu.tum.cs.vis.model.util.Triangle;

/**
 * 
 * This is a test class for testing collada parser and visualization functionality.
 *   
 *  
 * @author Stefan Profanter
 *
 */
public class ColladaTest extends PApplet {

	private static final long serialVersionUID = 5157225399456524729L;
	
	PeasyCam cam;
    LinkedList <Triangle> triangles = new LinkedList <Triangle>();
	ItemModel model1;
	ItemModel model2;
	ItemModel model3;

    
	public void setup()
    {
      size(1000, 1000, P3D);
      
      //model1 = new ItemModel("/home/stefan/Truck2.kmz");
      //model1 = new ItemModel("/home/stefan/ros/knowrob/knowrob_cad_models/models/collada/Expedit/Expedit_2X4.kmz");
      model2 = new ItemModel("/home/stefan/ros/knowrob/knowrob_cad_models/models/collada/hospital_bed.kmz");
      //model3 = new ItemModel("/home/stefan/ros/knowrob/knowrob_cad_models/models/collada/Expedit/Door.kmz");
      //parser = ColladaParser.loadModel();
      
    
      frameRate(500);
      cam = new PeasyCam(this, 0,0,0, 10);
   // values for zooming (min distance to which one can zoom in)
		cam.setMinimumDistance(0.01);
		cam.setMaximumDistance(200);
		
		cam.setRightDragHandler(cam.getPanDragHandler());
		cam.setLeftDragHandler(cam.getRotateDragHandler());

		cam.setDistance(100);
		
		// initialize camera view parameters
		cam.rotateX((float)Math.PI/2f);
		//cam.rotateY(3.141592654);

		/*System.out.println("Model Width: " + model1.getParser().getGroup().getTotalWidth());
		System.out.println("Model Height: " + model1.getParser().getGroup().getTotalHeight());
		System.out.println("Model Depth: " + model1.getParser().getGroup().getTotalDepth());*/
		

    }
	
	
	float a; 

	public Vector3f rotateVectorX(Vector3f v, float angle)
	{
		return new Vector3f(v.x,v.y*(float)Math.cos(angle)-v.z*(float)Math.sin(angle),v.y*(float)Math.sin(angle)+v.z*(float)Math.cos(angle));
	}
	public Vector3f rotateVectorY(Vector3f v, float angle)
	{
		return new Vector3f(v.x*(float)Math.cos(angle)-v.z*(float)Math.sin(angle),v.y,v.x*(float)Math.sin(angle)+v.z*(float)Math.cos(angle));
	}
	public Vector3f rotateVectorZ(Vector3f v, float angle)
	{
		return new Vector3f(v.x*(float)Math.cos(angle)-v.y*(float)Math.sin(angle),v.x*(float)Math.sin(angle)+v.y*(float)Math.cos(angle),v.z);
	}
	
    public void draw(){
      background(16);
      
      noStroke();

      fill(127);
		
		Vector3f camPos = new Vector3f(cam.getPosition());
		Vector3f camLook = new Vector3f(cam.getLookAt());
		Vector3f camRot = new Vector3f(cam.getRotations());
		

		
		
		float zCoord = camPos.z - camLook.z;
		if (zCoord < 0)
			camRot.y += Math.PI;
		
		Vector3f camDir = new Vector3f(camPos);
		camDir.sub(camLook);
		
		camDir = rotateVectorX(camDir, (float)Math.PI/4f);
		camDir = rotateVectorY(camDir,(float)Math.PI/4f);
		//camDir = rotateVectorZ(camDir, camRot.z);
		camDir.add(camLook);
		camDir.normalize();
		camDir.scale(30);

		//Rotate left and up by 45 deg
		//Vector3f keyLight = new Vector3f(camDir.x*(float)Math.cos(Math.PI/4f)-camDir.z*(float)Math.sin(Math.PI/4f),camDir.y,camDir.x*(float)Math.sin(Math.PI/4f)+camDir.z*(float)Math.cos(Math.PI/4f));
		//keyLight = new Vector3f(keyLight.x,keyLight.y*(float)Math.cos(Math.PI/4f)-keyLight.z*(float)Math.sin(Math.PI/4f),keyLight.y*(float)Math.sin(Math.PI/4f)+keyLight.z*(float)Math.cos(Math.PI/4f));
		//keyLight.add(camLook);

		//Vector2f rot90 = new Vector2f();

		lights();
		
		System.out.println("x:" + camRot.x + " y:" + camRot.y + " z:" + camRot.z);
		//spotLight(255f, 20f, 0f, 0, 0, 0, 0, 0, 1, (float)Math.PI/4f, 5);
		//pointLight(255f, 20f, 0f, keyLight.x,keyLight.y, keyLight.z);
		pointLight(80f, 80f, 100f, camPos.x,camPos.y, camPos.z);
		//spotLight(255f, 20f, 0f, 10, -10, 0, -1, 0, 1, (float)Math.PI/4f, 0);
		//directionalLight(255f, 20f, 0f, 1, 0, 0);
		//ambientLight(100, 100, 80);
		

	     // strokeWeight(10);
	     // stroke(255, 255, 255);
	     // line(camLook.x, camLook.y, camLook.z, camDir.x, camDir.y, camDir.z);
      
     scale(5);
      //model1.getParser().draw(this,0);
     
     //x plane
   /*  pushMatrix();
     translate(-5, 0, 0);
     rotateY((float)Math.PI/2f);
     rect(-5, -5, 10, 10);
     popMatrix();
     
     //z plane
     pushMatrix();
     translate(0, 0, 5);
     rect(-5, -5, 10, 10);
     popMatrix();
     
     //y plane
     pushMatrix();
     translate(0, 5, 0);
     rotateX((float)Math.PI/2f);
     rect(-5, -5, 10, 10);
     popMatrix();
     */
     
      model2.getParser().getModel().draw(this.g,null);
      //model3.getParser().draw(this,0);
      noFill();
      strokeWeight(1);
      stroke(204, 120, 15);
     // model1.getParser().getGroup().drawBoundingBox(this, true);
     
      
      stroke(125, 0, 0);
      strokeWeight(1);
      line(0, 0, 0, width, 0, 0);
      stroke(0, 125, 0);
      line(0, 0, 0, 0, 0, width);
      stroke(0, 0, 125);
      line(0, 0, 0, 0, height, 0);
      


    }
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		PApplet.main(new String[]{"edu.tum.cs.vis.test.ColladaTest"});
	}

}

