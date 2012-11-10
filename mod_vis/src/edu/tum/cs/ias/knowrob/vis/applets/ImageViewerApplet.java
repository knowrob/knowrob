package edu.tum.cs.ias.knowrob.vis.applets;

import processing.core.PApplet;
import processing.core.PImage;
import java.awt.Dimension;

public class ImageViewerApplet extends PApplet {

	private static final long serialVersionUID = 4502618030349542714L;


    PImage img;
    String image = null;

    public ImageViewerApplet(String image) {
        this.image = image;
    }

    public ImageViewerApplet() {
        
    }
	
	public void setup() {

        if(this.image != null) {
            System.out.println("Loading " + image);
            this.img = loadImage(image);
            size(img.width, img.height, P2D);
        }
        else
		size(279, 400, P2D);
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
		setSize(width, height);
        setPreferredSize(new Dimension(width,height));
        setMaximumSize(new Dimension(width,height));
        setMinimumSize(new Dimension(width,height));
		draw();
	}
	
}

