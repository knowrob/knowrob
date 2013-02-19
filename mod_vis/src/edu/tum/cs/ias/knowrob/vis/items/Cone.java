package edu.tum.cs.ias.knowrob.vis.items;

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;

import javax.vecmath.Vector3f;

import edu.tum.cs.ias.knowrob.vis.Canvas;

public class Cone extends Item {

	private Vector3f direction;
	private float radius1;
	private float radius2;
	private float height;
	


	public Cone(Matrix4d pose, Vector3d dim, Vector3d dir, float radius1, float radius2, float height) {
		super(pose, dim);
		
		this.direction = new Vector3f((float) dir.x, (float) dir.y, (float) dir.z);
		this.radius1 = radius1;
		this.radius2 = radius2;
		this.height = height;
	}
	
	
	public Cone(float m00, float m01, float m02, float m03, float m10,
			float m11, float m12, float m13, float m20, float m21, float m22,
			float m23, float m30, float m31, float m32, float m33, float dirX, 
			float dirY, float dirZ, float radius1, float radius2, float height) {
		
		super(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31,
				m32, m33, 0, 0, 0);
		
		this.direction = new Vector3f(dirX, dirY, dirZ);
		this.radius1 = radius1;
		this.radius2 = radius2;
		this.height = height;
		
	}
	
	public Cone(Matrix4d pose, Vector3d dim){
		super(pose, dim);
	}
	
	@Override
	public void drawIt(Canvas c) {		
		new ConePrimitive().cone(c, direction, radius1, radius2, height, true, true);
	}

}
