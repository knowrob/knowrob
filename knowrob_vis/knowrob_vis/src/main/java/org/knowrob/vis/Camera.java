package org.knowrob.vis;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import geometry_msgs.Pose;

public class Camera extends AbstractNodeMain {
	Publisher<Pose> cam_pub = null;

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		cam_pub = connectedNode.newPublisher("/camera/pose",
				geometry_msgs.Pose._TYPE);
	}

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("knowrob_vis/camera");
	}

	// Accepts 4x4 Transformation matrix for adjusting camera pose
	public void setCameraTransform(final String[] transforms) {
		try {
			final Pose pose = cam_pub.newMessage();

			pose.getPosition().setX(Float.parseFloat(transforms[3]));
			pose.getPosition().setY(Float.parseFloat(transforms[7]));
			pose.getPosition().setZ(Float.parseFloat(transforms[11]));

			float xx = Float.parseFloat(transforms[0]);
			float xy = Float.parseFloat(transforms[1]);
			float xz = Float.parseFloat(transforms[2]);
			float yx = Float.parseFloat(transforms[4]);
			float yy = Float.parseFloat(transforms[5]);
			float yz = Float.parseFloat(transforms[6]);
			float zx = Float.parseFloat(transforms[8]);
			float zy = Float.parseFloat(transforms[9]);
			float zz = Float.parseFloat(transforms[10]);

			final float t = xx + yy + zz;
			float w, x, y, z;
			if (t >= 0) { // |w| >= .5
				float s = (float) Math.sqrt(t + 1);
				w = 0.5f * s;
				s = 0.5f / s;
				x = (zy - yz) * s;
				y = (xz - zx) * s;
				z = (yx - xy) * s;
			} else if ((xx > yy) && (xx > zz)) {
				float s = (float) Math.sqrt(1.0 + xx - yy - zz); // |s|>=1
				x = s * 0.5f; // |x| >= .5
				s = 0.5f / s;
				y = (yx + xy) * s;
				z = (xz + zx) * s;
				w = (zy - yz) * s;
			} else if (yy > zz) {
				float s = (float) Math.sqrt(1.0 + yy - xx - zz); // |s|>=1
				y = s * 0.5f; // |y| >= .5
				s = 0.5f / s;
				x = (yx + xy) * s;
				z = (zy + yz) * s;
				w = (xz - zx) * s;
			} else {
				float s = (float) Math.sqrt(1.0 + zz - xx - yy); // |s|>=1
				z = s * 0.5f; // |z| >= .5
				s = 0.5f / s;
				x = (xz + zx) * s;
				y = (zy + yz) * s;
				w = (yx - xy) * s;
			}

			pose.getOrientation().setX(x);
			pose.getOrientation().setY(y);
			pose.getOrientation().setZ(z);
			pose.getOrientation().setW(w);

			cam_pub.publish(pose);
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	public void setCameraPose(final String[] positions,
			final String[] orientations) {
		try {
			final Pose pose = cam_pub.newMessage();

			pose.getPosition().setX(Float.parseFloat(positions[0]));
			pose.getPosition().setY(Float.parseFloat(positions[1]));
			pose.getPosition().setZ(Float.parseFloat(positions[2]));

			pose.getOrientation().setX(Float.parseFloat(orientations[1]));
			pose.getOrientation().setY(Float.parseFloat(orientations[2]));
			pose.getOrientation().setZ(Float.parseFloat(orientations[3]));
			pose.getOrientation().setW(Float.parseFloat(orientations[0]));

			cam_pub.publish(pose);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
