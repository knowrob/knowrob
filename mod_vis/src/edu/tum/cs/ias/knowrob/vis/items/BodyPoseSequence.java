package edu.tum.cs.ias.knowrob.vis.items;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import edu.tum.cs.ias.knowrob.util.datastructures.Point;
import edu.tum.cs.ias.knowrob.vis.Canvas;
import edu.tum.cs.ias.knowrob.vis.DrawableAnimated;

import javax.vecmath.Vector3f;

/**
 * TODO This class should be moved to edu.tum.cs.vis.items and should be specialized here. The specizalization should overwrite, in particular, drawSphere to use negative radii.
 * 
 * @author jain, waldhers
 */
public class BodyPoseSequence extends ItemBase implements DrawableAnimated {
	
	protected ArrayList<Point[]> jointPositions = new ArrayList<Point[]>();
	protected Integer trackedBodyPart = null, trackedBodyPartStartTime;
	protected int color = 0xffff00ff;
	
	protected final float armsThickness = 0.035f;
	protected final float spineThickness = 0.04f;
	
	protected boolean wireMode = false;

	public void addPose(float EPISODE_NR, float TIME, String action, 
			  float BECX, float BECY, float BECZ,  float ULWX, float ULWY, float ULWZ, 
			  float OLWX, float OLWY, float OLWZ,  float UBWX, float UBWY, float UBWZ, 
			  float OBWX, float OBWY, float OBWZ,  float UHWX, float UHWY, float UHWZ,
			  float BRKX, float BRKY, float BRKZ,  float OHWX, float OHWY, float OHWZ,
			  float KOX,  float KOY,  float KOZ,   float SEHX, float SEHY, float SEHZ,
			  float OSLX, float OSLY, float OSLZ,  float USLX, float USLY, float USLZ, 
			  float FULX, float FULY, float FULZ,  float FBLX, float FBLY, float FBLZ,
			  float OSRX, float OSRY, float OSRZ,  float USRX, float USRY, float USRZ,
			  float FURX, float FURY, float FURZ,  float FBRX, float FBRY, float FBRZ,
			  float SBLX, float SBLY, float SBLZ,  float OALX, float OALY, float OALZ,
			  float UALX, float UALY, float UALZ,  float HALX, float HALY, float HALZ,
			  float FILX, float FILY, float FILZ,  float SBRX, float SBRY, float SBRZ,  
			  float OARX, float OARY, float OARZ,  float UARX, float UARY, float UARZ, 
			  float HARX, float HARY, float HARZ,  float FIRX, float FIRY, float FIRZ ){
		  
		  Point[] newPosition = new Point[28];
		 
		  newPosition[0]  = new Point(BECX, BECY, BECZ, EPISODE_NR, TIME, "BEC", action);	  
		  newPosition[1]  = new Point(ULWX, ULWY, ULWZ, EPISODE_NR, TIME, "ULW", action);
		  newPosition[2]  = new Point(OLWX, OLWY, OLWZ, EPISODE_NR, TIME, "OLW", action);
		  newPosition[3]  = new Point(UBWX, UBWY, UBWZ, EPISODE_NR, TIME, "UBW", action);
		  newPosition[4]  = new Point(OBWX, OBWY, OBWZ, EPISODE_NR, TIME, "OBW", action);
		  newPosition[5]  = new Point(UHWX, UHWY, UHWZ, EPISODE_NR, TIME, "UHW", action);
		  newPosition[6]  = new Point(BRKX, BRKY, BRKZ, EPISODE_NR, TIME, "BRK", action);
		  newPosition[7]  = new Point(OHWX, OHWY, OHWZ, EPISODE_NR, TIME, "OHW", action);
		  newPosition[8]  = new Point(KOX,  KOY,  KOZ,  EPISODE_NR, TIME, "KO", action);
		  newPosition[9]  = new Point(SEHX, SEHY, SEHZ, EPISODE_NR, TIME, "SEH", action);
		  newPosition[10] = new Point(OSLX, OSLY, OSLZ, EPISODE_NR, TIME, "OSL", action);
		  newPosition[11] = new Point(USLX, USLY, USLZ, EPISODE_NR, TIME, "USL", action);
		  newPosition[12] = new Point(FULX, FULY, FULZ, EPISODE_NR, TIME, "FUL", action);
		  newPosition[13] = new Point(FBLX, FBLY, FBLZ, EPISODE_NR, TIME, "FBL", action);
		  newPosition[14] = new Point(OSRX, OSRY, OSRZ, EPISODE_NR, TIME, "OSR", action);
		  newPosition[15] = new Point(USRX, USRY, USRZ, EPISODE_NR, TIME, "USR", action);
		  newPosition[16] = new Point(FURX, FURY, FURZ, EPISODE_NR, TIME, "FUR", action);
		  newPosition[17] = new Point(FBRX, FBRY, FBRZ, EPISODE_NR, TIME, "FBR", action);
		  newPosition[18] = new Point(SBLX, SBLY, SBLZ, EPISODE_NR, TIME, "SBL", action);
		  newPosition[19] = new Point(OALX, OALY, OALZ, EPISODE_NR, TIME, "OAL", action);
		  newPosition[20] = new Point(UALX, UALY, UALZ, EPISODE_NR, TIME, "UAL", action);
		  newPosition[21] = new Point(HALX, HALY, HALZ, EPISODE_NR, TIME, "HAL", action);
		  newPosition[22] = new Point(FILX, FILY, FILZ, EPISODE_NR, TIME, "FIL", action);
		  newPosition[23] = new Point(SBRX, SBRY, SBRZ, EPISODE_NR, TIME, "SBR", action);
		  newPosition[24] = new Point(OARX, OARY, OARZ, EPISODE_NR, TIME, "OAR", action);
		  newPosition[25] = new Point(UARX, UARY, UARZ, EPISODE_NR, TIME, "UAR", action);
		  newPosition[26] = new Point(HARX, HARY, HARZ, EPISODE_NR, TIME, "HAR", action);		  
		  newPosition[27] = new Point(FIRX, FIRY, FIRZ, EPISODE_NR, TIME, "FIR", action);
		  
//		  // invert z
//		  for(int i = 0; i < newPosition.length; i++)
//			  newPosition[i].z *= -1;
		  
		  jointPositions.add(newPosition);
		  defaultColor = color;
	}
	
	public ArrayList<Point[]> getSequence() {
		return jointPositions;
	}

	public void addPose(float episode_nr, float time, String action, float[] coords) {
		addPose(episode_nr, time, action, coords[0], coords[1], coords[2], coords[3], coords[4], coords[5], coords[6], coords[7], coords[8], coords[9], coords[10], coords[11], coords[12], coords[13], coords[14], coords[15], coords[16], coords[17], coords[18], coords[19], coords[20], coords[21], coords[22], coords[23], coords[24], coords[25], coords[26], coords[27], coords[28], coords[29], coords[30], coords[31], coords[32], coords[33], coords[34], coords[35], coords[36], coords[37], coords[38], coords[39], coords[40], coords[41], coords[42], coords[43], coords[44], coords[45], coords[46], coords[47], coords[48], coords[49], coords[50], coords[51], coords[52], coords[53], coords[54], coords[55], coords[56], coords[57], coords[58], coords[59], coords[60], coords[61], coords[62], coords[63], coords[64], coords[65], coords[66], coords[67], coords[68], coords[69], coords[70], coords[71], coords[72], coords[73], coords[74], coords[75], coords[76], coords[77], coords[78], coords[79], coords[80], coords[81], coords[82], coords[83]);
	}
	
	public void setColor(int color) {
		this.color = color;
	}
	
	public void draw(Canvas c, int step) {		
		drawPose(c, step);
		
		if(trackedBodyPart != null)
			drawBodyPartTrajectory(c, trackedBodyPart, trackedBodyPartStartTime, step);
	}	
	
	public void drawPose(Canvas c, int step) {
		if(wireMode)
			drawPoseWire(c, step);
		else
			drawPoseFat(c, step);
	}
	
	public void drawPoseWire(Canvas c, int step) {
		if(step > getMaxStep())
			return;
		
		Point[] pose = this.jointPositions.get(step);

  	    //c.strokeWeight(5f);
  	    c.stroke(this.color);
		
		c.pushMatrix();
		c.scale(100);

		// draw trajectory
		for(int i = 0; i < 9; i++) 
			c.line(pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z);		

		// draw head
		c.pushMatrix();
		c.translate(pose[8].x, pose[8].y, pose[8].z);
		c.sphere(0.09f);
		c.popMatrix();

		// draw body segments
		c.line(pose[0].x, pose[0].y, pose[0].z, pose[10].x, pose[10].y, pose[10].z);
		c.line(pose[0].x, pose[0].y, pose[0].z, pose[14].x, pose[14].y, pose[14].z);
		c.line(pose[6].x, pose[6].y, pose[6].z, pose[18].x, pose[18].y, pose[18].z);
		c.line(pose[6].x, pose[6].y, pose[6].z, pose[23].x, pose[23].y, pose[23].z);

		for(int i = 10; i < 13; i++) 
			c.line(pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z);		
		for(int i = 14; i < 17; i++) 
			c.line(pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z);		
		for(int i = 18; i < 22; i++) 
			c.line(pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z);		
		for(int i = 23; i < 27; i++) 
			c.line(pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z);		
		
		c.popMatrix();
	}
	
	public void drawPoseFat(Canvas c, int step) {
		if(step > getMaxStep())
			return;
			
		Point[] pose = this.jointPositions.get(step);

  	  //  c.strokeWeight(5f);
  	    c.color(color);
  	    c.fill(color);
		
		c.pushMatrix();
		c.scale(100);
		c.sphereDetail(20);

		// draw trajectory
		for(int i = 0; i < 8; i++){ 
			c.color(0xffff0000);
			cyl(c,pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z, spineThickness);
			new Sphere(pose[i].x, pose[i].y, pose[i].z, -spineThickness, this.color).draw(c);
		}
		
		// draw head
		new Sphere(pose[8].x, pose[8].y, pose[8].z, -spineThickness*2, color).draw(c);	
		// draw cone for gaze direction
		float lambda = 1.1f;
		cone(c,pose[8].x, pose[8].y, pose[8].z, pose[8].x + lambda*(pose[9].x - pose[8].x), pose[8].y + lambda*(pose[9].y - pose[8].y), pose[8].z+ + lambda*(pose[9].z - pose[8].z), spineThickness, spineThickness * 0.05f);		

		// draw body segments
		cyl(c,pose[0].x, pose[0].y, pose[0].z, pose[10].x, pose[10].y, pose[10].z, armsThickness);
		cyl(c,pose[0].x, pose[0].y, pose[0].z, pose[14].x, pose[14].y, pose[14].z, armsThickness);
		cyl(c,pose[6].x, pose[6].y, pose[6].z, pose[18].x, pose[18].y, pose[18].z, armsThickness);
		cyl(c,pose[6].x, pose[6].y, pose[6].z, pose[23].x, pose[23].y, pose[23].z, armsThickness);
		
		for(int i = 10; i < 13; i++) {
			cyl(c,pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z, armsThickness);
			new Sphere(pose[i].x, pose[i].y, pose[i].z, -armsThickness, this.color).draw(c);
		}
		for(int i = 14; i < 17; i++){
			cyl(c,pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z, armsThickness);
			new Sphere(pose[i].x, pose[i].y, pose[i].z, -armsThickness, this.color).draw(c);
		}
		for(int i = 18; i < 21; i++){ 
			cyl(c,pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z, armsThickness);
			new Sphere(pose[i].x, pose[i].y, pose[i].z, -armsThickness, this.color).draw(c);
		}
		
		cone(c,pose[21].x, pose[21].y, pose[21].z, pose[21].x + lambda*(pose[22].x - pose[21].x), pose[21].y + lambda*(pose[22].y - pose[21].y), pose[21].z+ + lambda*(pose[22].z - pose[21].z), armsThickness, armsThickness * 0.1f);
		
		for(int i = 23; i < 26; i++){ 
			cyl(c,pose[i].x, pose[i].y, pose[i].z, pose[i + 1].x, pose[i + 1].y, pose[i + 1].z, armsThickness);
			drawSphere(c, pose[i].x, pose[i].y, pose[i].z, armsThickness);
		}
		
		cone(c,pose[26].x, pose[26].y, pose[26].z, pose[26].x + lambda*(pose[27].x - pose[26].x), pose[26].y + lambda*(pose[27].y - pose[26].y), pose[26].z+ + lambda*(pose[27].z - pose[26].z), armsThickness, armsThickness * 0.1f);
		
		// elbows
		drawSphere(c, pose[20].x, pose[20].y, pose[20].z, armsThickness);
		drawSphere(c, pose[25].x, pose[25].y, pose[25].z, armsThickness);
		
		c.popMatrix();
		
	}
	
	protected void drawSphere(Canvas c, float x, float y, float z, float r) {
		new Sphere(x, y, z, -r, this.color).draw(c);
	}
	
	protected void cyl(Canvas c, float x1, float y1, float z1, float x2, float y2, float z2, float r){
		ConePrimitive cyl = new ConePrimitive(new Vector3f(x1, y1, z1), new Vector3f(x2, y2, z2), r);
		cyl.draw(c);
	}
	
	
	protected void cone(Canvas c, float x1, float y1, float z1, float x2, float y2, float z2, float r1, float r2){
		ConePrimitive cyl = new ConePrimitive(new Vector3f(x1, y1, z1), new Vector3f(x2, y2, z2), r1, r2);
		cyl.draw(c);
	}
	
	public void trackBodyPart(Integer bodyPart, int startTime) {
		this.trackedBodyPart = bodyPart;
		this.trackedBodyPartStartTime = startTime;
	}
	
	public boolean isTrackingBodyPart() {
		return trackedBodyPart != null;
	}

	public void drawBodyPartTrajectory(Canvas c, int bodyPart, int begin, int end) {		
		c.pushMatrix();
		c.stroke(255,255,0);
		c.scale(100);
		if((bodyPart != -1) && (end > 0)) {
			for(int i = begin; i < end - 1; i++) {
				Point[] pose1 = jointPositions.get(i); 
				Point[] pose2 = jointPositions.get(i+1);
				c.line(pose1[bodyPart].x, pose1[bodyPart].y, pose1[bodyPart].z, pose2[bodyPart].x, pose2[bodyPart].y, pose2[bodyPart].z);
			}
		}
		c.popMatrix();
	}	  
	
	public int getMaxStep() {
		return jointPositions.size()-1;
	}
	
	public boolean isEmpty() {
		return jointPositions.size() == 0;
	}
	
    /**
     * reads joint trajectory data from a matlab ascii file
     * @param matlabAsciiFile
     * @throws NumberFormatException
     * @throws IOException
     */
    public void readData(File matlabAsciiFile) throws NumberFormatException, IOException {
		BufferedReader r = new BufferedReader(new FileReader(matlabAsciiFile));
		String line;
		while((line = r.readLine()) != null) {
			String[] parts = line.trim().split("\\s+");
			float[] coords = new float[parts.length];
			for(int i = 0; i < parts.length; i++)
				coords[i] = Float.parseFloat(parts[i]);
			this.addPose(1, 0, "", coords);
		}
    }

    public void setMode(boolean wireOnly) {
    	wireMode = wireOnly;
    }

	@Override
	public void setColor(int color, int start, int end) {
		System.err.println("not availible");
		
	}
}
