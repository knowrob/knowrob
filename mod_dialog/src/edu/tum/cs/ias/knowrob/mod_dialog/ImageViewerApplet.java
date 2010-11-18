package edu.tum.cs.ias.knowrob.mod_dialog;

import processing.core.PApplet;
import processing.core.PImage;

public class ImageViewerApplet extends PApplet {

	private static final long serialVersionUID = 4502618030349542714L;

	PImage img;
	public void setup() {

		size(50, 40, P2D);
	}

	public void draw() {
		
		background(20, 20, 20);

		if(this.img!=null) {
			image(this.img, 0,0,this.img.width,this.img.height);
		}
	}

	
	public void setImage(String image) {
		this.img=loadImage(image);
		this.height = img.height;
		this.width = img.width;
		
		draw();
	}
	
}

