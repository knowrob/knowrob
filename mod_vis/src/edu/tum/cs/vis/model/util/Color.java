package edu.tum.cs.vis.model.util;

/**
 * Represents a rgba color
 * 
 * @author Stefan Profanter
 * @version 1.0
 */
public class Color {

    public Color(float red,float green, float blue,float transp )
    {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.transparency = transp;
    }

	/**
	 * 
	 * @param col
	 *            Array with colors. Size: 4
	 */
	public Color(float col[]) {
		this.red = col[0];
		this.green = col[1];
		this.blue = col[2];
		this.transparency = col[3];
	}
    /**
     * from 0 to 1.0
     */
    public float red;
    public float green;
    public float blue;
    public float transparency;
}
