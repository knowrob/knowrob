package edu.tum.cs.vis.model.util;

import java.awt.Color;

public class DrawSettings {
	public DrawSettings() {
		this.drawType = DrawType.FILL;
		overrideColor = null;
	}

	private Color	overrideColor;
	public DrawType	drawType;
	private int		lineWidth	= 1;
	public boolean	forceDraw	= false;

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

	/**
	 * @param overrideColor
	 *            the overrideColor to set
	 */
	public void setOverrideColor(Color overrideColor) {
		this.overrideColor = overrideColor;
	}

	/**
	 * @param lineWidth
	 *            the lineWidth to set
	 */
	public void setLineWidth(int lineWidth) {
		this.lineWidth = lineWidth;
	}

	@Override
	public Object clone() {
		DrawSettings ds = new DrawSettings();
		ds.overrideColor = overrideColor == null ? null : new Color(overrideColor.getRed(),
				overrideColor.getGreen(), overrideColor.getBlue(), overrideColor.getAlpha());
		ds.drawType = drawType;
		ds.lineWidth = lineWidth;
		ds.forceDraw = forceDraw;
		return ds;

	}

	public DrawSettings getTemporaryOverride(Color overrideColor) {
		DrawSettings ds = (DrawSettings) this.clone();
		ds.overrideColor = overrideColor;
		return ds;
	}

	/**
	 * @return the overrideColor
	 */
	public Color getOverrideColor() {
		return overrideColor;
	}

}
