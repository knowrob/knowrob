package edu.tum.cs.ias.knowrob.util.datastructures;

import javax.vecmath.Matrix4f;

public class Vector3f extends javax.vecmath.Vector3f {
	private static final long serialVersionUID = 1L;

	public Vector3f() {
		super();
	}
	
	public Vector3f(float x, float y, float z) {
		super(x,y,z);
	}
	
	public Vector3f(javax.vecmath.Vector3f v) {
		super(v);
	}
	
	public double distance(Vector3f other) {
		float xd = x-other.x;
		float yd = y-other.y;
		float zd = z-other.z;
		return Math.sqrt(xd*xd+yd*yd+zd*zd);
	}
	
	public void rotate(double theta, javax.vecmath.Vector3f axis) {
	   Vector3f p = this;
	   Vector3f q = new Vector3f(0.0f,0.0f,0.0f);
	   double costheta, sintheta;

	   Vector3f r = new Vector3f(axis);
	   r.normalize();
	   
	   costheta = Math.cos(theta);
	   sintheta = Math.sin(theta);

	   q.x += (costheta + (1 - costheta) * r.x * r.x) * p.x;
	   q.x += ((1 - costheta) * r.x * r.y - r.z * sintheta) * p.y;
	   q.x += ((1 - costheta) * r.x * r.z + r.y * sintheta) * p.z;

	   q.y += ((1 - costheta) * r.x * r.y + r.z * sintheta) * p.x;
	   q.y += (costheta + (1 - costheta) * r.y * r.y) * p.y;
	   q.y += ((1 - costheta) * r.y * r.z - r.x * sintheta) * p.z;

	   q.z += ((1 - costheta) * r.x * r.z - r.y * sintheta) * p.x;
	   q.z += ((1 - costheta) * r.y * r.z + r.x * sintheta) * p.y;
	   q.z += (costheta + (1 - costheta) * r.z * r.z) * p.z;

	   this.x = q.x;
	   this.y = q.y;
	   this.z = q.z;
	}
	
	public static Matrix4f getRotationMatrix(double theta, javax.vecmath.Vector3f axis) {
		Matrix4f ret = new Matrix4f();
		float costheta, sintheta;
		
		Vector3f r = new Vector3f(axis);
		r.normalize();

		costheta = (float)Math.cos(theta);
		sintheta = (float)Math.sin(theta);

		ret.m00 = (costheta + (1 - costheta) * r.x * r.x);
		ret.m10 = ((1 - costheta) * r.x * r.y - r.z * sintheta);
		ret.m20 = ((1 - costheta) * r.x * r.z + r.y * sintheta);
		ret.m30 = 0;

		ret.m01 = ((1 - costheta) * r.x * r.y + r.z * sintheta);
		ret.m11 = (costheta + (1 - costheta) * r.y * r.y);
		ret.m21 = ((1 - costheta) * r.y * r.z - r.x * sintheta);
		ret.m31 = 0;

		ret.m02 = ((1 - costheta) * r.x * r.z - r.y * sintheta);
		ret.m12 = ((1 - costheta) * r.y * r.z + r.x * sintheta);
		ret.m22 = (costheta + (1 - costheta) * r.z * r.z);
		ret.m32 = 0;
		
		ret.m03 = 0;
		ret.m13 = 0;
		ret.m23 = 0;
		ret.m33 = 1;

		return ret;
	}
}
