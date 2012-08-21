package edu.tum.cs.ias.knowrob.util;

/**
 * implements some Color features.
 * can be used static (Colors.getColor()) different colors each time (at least the first 18 times).
 * can also be used instantiated. (new Colors()).nextColor();
 * @author Jakob
 */
public class Colors {
	public static Colors defaultInstance;
	static { defaultInstance = new Colors(); }
	
	public static int[][] defaultColors = new int[][] {
		{255,0,0,255},
		{255,255,0,255},
		{0,0,255,255},
		{255,0,255,255},
		{150,150,0,255},
		{0,150,150,255},
		{0,255,0,255},
		{150,0,0,255},
		{0,150,0,255},
		{150,0,150,255},
		{0,255,255,255},
		{0,150,255,255},
		{150,0,255,255},
		{0,255,150,255},
		{0,0,150,255},
		{150,255,0,255},
		{255,0,150,255},
		{255,150,0,255},
	};
	
	private int currentDefaultColor;
	
	/**
	 * creates a color out of it's red, green, blue and alpha components
	 */
	public static int convertColor(int red, int green, int blue, int alpha) {
		return (((((alpha << 8) + red) << 8) + green) << 8) + blue;
	}
	public static int getRed(int color) {
		return (color >> 16) % 256;
	}
	public static int getGreen(int color) {
		return (color >> 8) % 256;
	}
	public static int getBlue(int color) {
		return color% 256;
	}
	public static int getAlpha(int color) {
		return (color >> 24) % 256;
	}
	/**
	 * returns the next unused color (18 different colors currently implemented)
	 * of this instance
	 * @return
	 */
	public int nextColor() {
		int color = convertColor(defaultColors[currentDefaultColor][0],defaultColors[currentDefaultColor][1],defaultColors[currentDefaultColor][2],defaultColors[currentDefaultColor][3]);
		currentDefaultColor = (currentDefaultColor+1) % defaultColors.length;
		return color;
	}
	
	/**
	 * returns the next unused color of the default instance
	 */
	public static int getColor() {
		return defaultInstance.nextColor();
	}
}
