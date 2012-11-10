package edu.tum.cs.ias.knowrob.vis.items;

import java.util.Vector;

import edu.tum.cs.ias.knowrob.vis.Canvas;



public class Trajectories extends ItemBase{
	public float pointSize = 0.0f, sphereSize = 120.0f;
	public int pointColor = 0xffcbcbcb, sphereColor = 0x99ffff00;
	public int lineColor = 0xffffffff;
	public Vector<Point> pointsTraj;
	public Vector<Point> point;
	public int cont =0;
	
	public Trajectories() {
		pointsTraj = new Vector<Point>();
	}
	
	public void addTraj(float x, float y, float z) {
		pointsTraj.add(new Point(x, y, z, pointColor, pointSize));
	}
	
	public void addPoint(float x, float y, float z, float radius){
		point.add(new Point(x,y,z,pointColor, pointSize));
	}
	
	public void draw(Canvas c, int step) {
		c.pushMatrix();
		//c.scale(100);
		
		Point prev=null;
		c.sphereDetail(3);
	
		for(Point p : pointsTraj) {
    		p.draw(c);
    		//System.out.println("p.v= "+p.v);
    		if (cont<=0 && p.v.x!=0 ){
    			prev=p; 
    			//System.out.println("prev: "+prev.v);
    			
    			cont++;
    		} else if (p.v.x !=0) {
			 // draw line connecting previous point with current point
				//System.out.println("prev: " +prev.v +" current: "+p.v);
				c.drawLine(prev.v, p.v, lineColor);
				
				//add the point
				prev = p;
				//System.out.println("p.v= "+p.v);
			} else if (p.v.x==0){
				cont=0;
				//lineColor=0xFFFF0000;
				//System.out.println("0000000");
			  }
			
		}
		c.sphereDetail(30);
		c.popMatrix();
	}
	
}
