package edu.tum.cs.ias.knowrob.vis.themes;

import processing.core.PFont;
import controlP5.Button;
import controlP5.ControlFont;
import controlP5.ControlP5;
import controlP5.Group;
import controlP5.Knob;
import controlP5.ListBox;
import controlP5.MultiList;
import controlP5.MultiListButton;
import controlP5.Tab;
import controlP5.Textfield;
import controlP5.Textlabel;



/**
 * Define styles for different GUI elements to keep them at a central
 * location and allow easy re-definition.
 * 
 * @author tenorth
 *
 */
public class GreyTheme {

	
	/**
	 * Font for drawing text
	 */
	private static PFont dejavuFont = null;

	/** 
	 * Font for ControlP5
	 */
	public static ControlFont cfont = null;
	
	
	public static int colorBackground = color(80);
	public static int colorForeground = color(180);
	public static int colorActive = color(200);
	public static int colorGroupBackground = color(50);
	
	
	public static ControlP5 applyStyle(ControlP5 controlP5) {

		controlP5.setColorBackground(color(80))
		.setColorForeground(color(180))
		.setColorActive(color(200));

		dejavuFont = controlP5.papplet.createFont("DejaVu Sans",12);
		cfont = new ControlFont(dejavuFont);

		return controlP5;
	}

	public static Tab applyStyle(Tab tab) {
		
		tab.activateEvent(true)
			.setColorLabel(color(255))
			.setHeight(20);
		return tab;
	}


	public static Textfield applyStyle(Textfield t) {
		 t.setAutoClear(false);
		 return t;
	}
		
	public static Button applyStyle(Button b) {
		 return b;
	}

	public static Group applyStyle(Group g) {
		g.setBackgroundColor(color(50));
		g.setBarHeight(15);
		g.getCaptionLabel().getStyle().marginTop=3;
		return g;
	}

	public static Knob applyStyle(Knob k) {
		k.setViewStyle(Knob.ARC)
		 .setShowAngleRange(false)
		 .setAngleRange((float) (2*Math.PI));
		return k;
	}

	public static ListBox applyStyle(ListBox lb, int itemHeight) {
		
        lb.setItemHeight(itemHeight)
          .setBarHeight(itemHeight);

  		lb.getCaptionLabel().getStyle().marginTop = itemHeight/4;
  		lb.getValueLabel().getStyle().marginTop = itemHeight/4;
  		lb.getCaptionLabel().getStyle().marginBottom = itemHeight/4;
  		lb.getValueLabel().getStyle().marginBottom = itemHeight/4;
		return lb;
	}
	
	public static Textlabel applyStyle(Textlabel l) {
		return l;
	}
	
	

	/**
	 * Apply the default styling to a MultiListButton
	 * 
	 * @param b The button to be styled
	 * @return the same button instance with updated properties
	 */
	public static MultiListButton applyStyle(MultiListButton b) {
		b.getCaptionLabel().toUpperCase(false).setFont(cfont);
		b.setHeight(20);
		return b;
	}

	
	/**
	 * Apply the default styling to a MultiList
	 * 
	 * @param b The list to be styled
	 * @return the same list instance with updated properties
	 */
	public static MultiList applyStyle(MultiList lb) {

		lb.setHeight(20);
		lb.getCaptionLabel().setFont(cfont);
		
		return lb;
	}

	
	
	// utility methods
	public static final int color(int gray) {
		if (gray > 255) gray = 255; else if (gray < 0) gray = 0;
		return 0xff000000 | (gray << 16) | (gray << 8) | gray;
	}
	
	public static final int color(int r, int g, int b) {
		
		if (r > 255) r = 255; else if (r < 0) r = 0;
		if (g > 255) g = 255; else if (g < 0) g = 0;
		if (b > 255) b = 255; else if (b < 0) b = 0;

		return 0xff000000 | (r << 16) | (g << 8) | b;
	}

}
