package edu.tum.cs.vis.model.util;

import java.awt.Color;

public class DrawSettings {
	public DrawSettings() {
		this.drawType = DrawType.FILL;
		overrideColor = null;
	}

	public Color	overrideColor;
	public DrawType	drawType;
	private int		lineWidth	= 1;

	public void incLineWidth() {
		lineWidth++;
	}

	public void decLineWidth() {
		if (lineWidth == 1)
			return;
		lineWidth--;
	}

	public int getLineWidth() {
		return lineWidth;
	}
}
