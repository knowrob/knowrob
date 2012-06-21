package edu.tum.cs.vis.model.uima.cas;

import java.io.Serializable;

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
public class MeshCas extends JCas implements Serializable {

	/**
	 * Auto generated
	 */
	private static final long						serialVersionUID	= 4603166505444872760L;
	/**
	 * Group which represents the mesh and its child groups
	 */
	private Group									group;

	private boolean									drawMesh			= true;

	// Create the listener list
	protected javax.swing.event.EventListenerList	listenerList		= new javax.swing.event.EventListenerList();

	/**
	 * adds a new annotation to the annotations list
	 * 
	 * @param a
	 *            annotation to add
	 */
	public void addAnnotation(Annotation a) {
		synchronized (annotations) {
			annotations.add(a);
		}
	}

	/**
	 * Draw the original mesh and all the meshes of the annotations.
	 * 
	 * @param applet
	 *            Applet to draw on
	 */
	public void draw(PApplet applet) {
		if (drawMesh)
			group.draw(applet, null);
		synchronized (annotations) {
			for (Annotation a : annotations) {
				if (!(a instanceof MeshAnnotation))
					continue;
				MeshAnnotation ma = (MeshAnnotation) a;
				ma.draw(applet);
			}
		}
	}

	/**
	 * @return the group
	 */
	public Group getGroup() {
		return group;
	}

	/**
	 * @return the drawMesh
	 */
	public boolean isDrawMesh() {
		return drawMesh;
	}

	/**
	 * @param drawMesh
	 *            the drawMesh to set
	 */
	public void setDrawMesh(boolean drawMesh) {
		this.drawMesh = drawMesh;
	}

	/**
	 * @param group
	 *            the group to set
	 */
	public void setGroup(Group group) {
		this.group = group;
	}

}