package edu.tum.cs.vis.model.uima.cas;

import processing.core.PApplet;
import edu.tum.cs.uima.Annotation;
import edu.tum.cs.uima.JCas;
import edu.tum.cs.vis.model.uima.annotation.MeshAnnotation;
import edu.tum.cs.vis.model.util.Group;

/**
 * UIMA CAS for 3D meshes.
 * 
 * @author Stefan Profanter
 * 
 */
public class MeshCas extends JCas {

	/**
	 * Group which represents the mesh and its child groups
	 */
	private Group	group;

	/**
	 * Draw the original mesh and all the meshes of the annotations.
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void draw(PApplet applet) {
		// group.draw(applet, null);
		for (Annotation a : annotations) {
			if (!(a instanceof MeshAnnotation))
				continue;
			MeshAnnotation ma = (MeshAnnotation) a;
			ma.draw(applet);
		}
	}

	/**
	 * @return the group
	 */
	public Group getGroup() {
		return group;
	}

	/**
	 * @param group
	 *            the group to set
	 */
	public void setGroup(Group group) {
		this.group = group;
	}

}
