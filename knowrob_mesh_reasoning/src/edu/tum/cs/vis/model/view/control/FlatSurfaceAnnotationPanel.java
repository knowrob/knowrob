package edu.tum.cs.vis.model.view.control;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import javax.swing.JLabel;
import javax.swing.JSlider;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import edu.tum.cs.vis.model.uima.annotation.FlatSurfaceAnnotation;

public class FlatSurfaceAnnotationPanel extends AnnotationPanel<FlatSurfaceAnnotation> implements
		ChangeListener {

	private final JSlider		minArea;
	private final JSlider		maxArea;

	private boolean				sliderChanged	= false;

	private final static int	FACTOR			= 100;

	public FlatSurfaceAnnotationPanel() {
		setLayout(new GridBagLayout());
		GridBagConstraints c = new GridBagConstraints();
		c.fill = GridBagConstraints.HORIZONTAL;
		JLabel sliderLabel = new JLabel("Min. area", JLabel.CENTER);
		minArea = new JSlider(JSlider.HORIZONTAL, 0, 1, 0);

		minArea.addChangeListener(this);

		// Turn on labels at major tick marks.

		c.gridx = 0;
		c.gridy = 0;
		add(sliderLabel, c);
		c.gridx = 1;
		c.gridy = 0;
		add(minArea, c);

		JLabel sliderLabelMax = new JLabel("Min. area", JLabel.CENTER);
		maxArea = new JSlider(JSlider.HORIZONTAL, 0, 1, 1);

		maxArea.addChangeListener(this);

		// Turn on labels at major tick marks.

		c.gridx = 0;
		c.gridy = 1;
		add(sliderLabelMax, c);
		c.gridx = 1;
		c.gridy = 1;
		add(maxArea, c);

		updateValues();
	}

	@Override
	public void stateChanged(ChangeEvent arg0) {
		sliderChanged = true;
		synchronized (annotations) {
			for (FlatSurfaceAnnotation a : annotations) {
				a.setDrawAnnotation(a.getArea().getSquareMM() * FACTOR <= maxArea.getValue()
						&& a.getArea().getSquareMM() * FACTOR >= minArea.getValue());
			}
		}

	}

	@Override
	public void updateValues() {

		if (annotations == null)
			return;

		float min = Float.MAX_VALUE;
		float max = Float.MIN_VALUE;

		synchronized (annotations) {
			for (FlatSurfaceAnnotation a : annotations) {
				min = Math.min(a.getArea().getSquareMM(), min);
				max = Math.max(a.getArea().getSquareMM(), max);
			}
		}

		minArea.setMinimum((int) min * FACTOR);
		minArea.setMaximum((int) (max + 1) * FACTOR);
		maxArea.setMinimum((int) min * FACTOR);
		maxArea.setMaximum((int) (max + 1) * FACTOR);
	}
}
