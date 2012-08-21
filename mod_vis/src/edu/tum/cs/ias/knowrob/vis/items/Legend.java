package edu.tum.cs.ias.knowrob.vis.items;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

import edu.tum.cs.ias.knowrob.vis.Canvas;
import edu.tum.cs.ias.knowrob.vis.Drawable;

public class Legend implements Drawable {
	
	int legPosX = 0, legPosY = 0;
	
	public Legend(java.io.File labelFile, int x, int y) throws NumberFormatException, IOException {
		legPosX = x;
		legPosY = y;
		loadLabels(labelFile);
	}
	
	public Vector<String> stringLeg = new Vector<String>();
	public static int[] colors = new int[] {0xffff0000, 0xff00ff00, 0xff0000ff, 0xffffff00, 0xffff00ff, 0xff00ffff, 0xffff8800, 0xffff0088, 0xff88ff00, 0xff00ff88, 0xff8800ff, 0xff0088ff};
	
	public void draw(Canvas c) {
		int i = 0;
		for(String str : stringLeg){
			c.fill(colors[i]);
			int x = legPosX;
			int y = legPosY + 13*i;
			c.rect(x, y-6, 6, 6);
			c.fill(0xffffffff);
			c.text(str, x + 10, y);
			i += 1;
		}
	}
	
	protected void loadLabels(java.io.File ascFile) throws NumberFormatException, IOException{
		BufferedReader r = new BufferedReader(new FileReader(ascFile));
		String line;
		while((line = r.readLine()) != null) {			
			this.stringLeg.add(line);
		}
	}
}
