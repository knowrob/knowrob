package edu.tum.cs.vis.model.view.control;

import java.util.ArrayList;

import javax.swing.JPanel;

public abstract class AnnotationPanel<T> extends JPanel {

	protected ArrayList<T>	annotations;

	/**
	 * @return the annotations
	 */
	public ArrayList<T> getAnnotations() {
		return annotations;
	}

	/**
	 * @param annotations
	 *            the annotations to set
	 */
	public void setAnnotations(ArrayList<T> annotations) {
		this.annotations = annotations;
	}

	public abstract void updateValues();
}
