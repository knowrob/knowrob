package edu.tum.cs.ias.knowrob.vis.items;

import java.util.ArrayList;

import edu.tum.cs.ias.knowrob.vis.Canvas;

/**
 * More efficient way to draw a lot of ellipses / points.
 * have to be on the same height (z-coordinate).
 * cannot be animated (-> setColor doesn't do anything)
 */
public class MultiEllipse extends ItemBase {
	private ArrayList<float[]> data;
	private ArrayList<Integer> colors;
	private float z;
	
	public MultiEllipse(float z)
	{
		this.z = z;
		colors = new ArrayList<Integer>();
		data = new ArrayList<float[]>();
	}
	
	public void addEllipse(float x, float y, float xVar, float yVar, int color) {
		this.colors.add(color);
		this.data.add(new float[] {x,y,xVar,yVar});
	}
	
	public void addPoint(float x, float y, int color) {
		this.colors.add(color);
		this.data.add(new float[] {x,y,2.5f,2.5f});
	}
	
	@Override
	public void setColor(int color) {
	}

	@Override
	public void setColor(int color, int start, int end) {
	}

	public void draw(Canvas c, int step) {
		c.pushMatrix();
		if(trafoMatrix != null)
			c.applyMatrix(trafoMatrix[0], trafoMatrix[1], trafoMatrix[2], trafoMatrix[3], 
					trafoMatrix[4], trafoMatrix[5], trafoMatrix[6], trafoMatrix[7], 
					trafoMatrix[8], trafoMatrix[9], trafoMatrix[10], trafoMatrix[11], 
					trafoMatrix[12], trafoMatrix[13], trafoMatrix[14], trafoMatrix[15]);
		c.translate(0, 0, -z);
		for(int i=0;i<colors.size();i++) {
			if (drawSettings != null && drawSettings.getOverrideColor() != null)
				c.fill(drawSettings.getOverrideColor().getRed(), drawSettings.getOverrideColor().getGreen(), drawSettings.getOverrideColor().getBlue(), drawSettings.getOverrideColor().getAlpha());
			else
				c.fill(colors.get(i));
			c.ellipse(data.get(i)[0],data.get(i)[1],data.get(i)[2],data.get(i)[3]);
		}
		c.popMatrix();
	}

}
