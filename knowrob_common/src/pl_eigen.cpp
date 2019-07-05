
#include <Eigen/Geometry>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

void eigen2pl(const Eigen::Quaterniond &q, const PlTerm &out) {
	PlTail l(out);
	l.append(q.x());
	l.append(q.y());
	l.append(q.z());
	l.append(q.w());
	l.close();
}

void eigen2pl(const Eigen::Vector3d &v, const PlTerm &out) {
	PlTail l(out);
	l.append(v.x());
	l.append(v.y());
	l.append(v.z());
	l.close();
}

void eigen2pl(const Eigen::Matrix4d &m, const PlTerm &out) {
	PlTail l(out);
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j) {
			l.append(m(i,j));
		}
	}
	l.close();
}

void pl2eigen(const PlTerm &arg, Eigen::Quaterniond &q) {
	PlTail list(arg); PlTerm value;
	list.next(value); q.x() = value;
	list.next(value); q.y() = value;
	list.next(value); q.z() = value;
	list.next(value); q.w() = value;
}

void pl2eigen(const PlTerm &arg, Eigen::Vector3d &v) {
	PlTail list(arg); PlTerm value;
	list.next(value); v.x() = value;
	list.next(value); v.y() = value;
	list.next(value); v.z() = value;
}

void pl2eigen(const PlTerm &arg, Eigen::Matrix4d &m) {
	PlTail list(arg); PlTerm value;
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j) {
			list.next(value);
			m(i,j) = value;
		}
	}
}

// +Quaternion, -Inverse
PREDICATE(quaternion_inverse, 2) {
	Eigen::Quaterniond q;
	pl2eigen(PL_A1,q);
	eigen2pl(q.inverse(),PL_A2);
	return TRUE;
}

// +Quaternion, +Vector, -Transformed
PREDICATE(quaternion_transform, 3) {
	Eigen::Quaterniond q;
	Eigen::Vector3d v;
	pl2eigen(PL_A1,q);
	pl2eigen(PL_A2,v);
	eigen2pl(q*v,PL_A3);
	return TRUE;
}

// +Quaternion1, +Quaternion2, -Multiplied
PREDICATE(quaternion_multiply, 3) {
	Eigen::Quaterniond q1,q2;
	pl2eigen(PL_A1,q1);
	pl2eigen(PL_A2,q2);
	eigen2pl(q1*q2,PL_A3);
	return TRUE;
}

// +Quaternion1, +Quaternion2, -Diff
PREDICATE(quaternion_diff, 3) {
	Eigen::Quaterniond q1,q2;
	pl2eigen(PL_A1,q1);
	pl2eigen(PL_A2,q2);
	eigen2pl(q1.inverse()*q2,PL_A3);
	return TRUE;
}

// +Quaternion, +Translation, -Matrix
PREDICATE(matrix_create, 3) {
	Eigen::Quaterniond q;
	Eigen::Vector3d v;
	pl2eigen(PL_A1,q);
	pl2eigen(PL_A2,v);
	Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
	// 3x3 block starting at (0,0)
	m.block(0,0,3,3) = q.toRotationMatrix();
	// 3x1 block starting at (0,3)
	m.block(0,3,3,1) = v;
	return TRUE;
}

// +Matrix, -Quaternion
PREDICATE(matrix_quaternion, 2) {
	Eigen::Matrix4d m4;
	pl2eigen(PL_A1,m4);
	Eigen::Matrix3d m3 = m4.block(0,0,3,3);
	Eigen::Quaterniond q(m3);
	eigen2pl(q,PL_A2);
	return TRUE;
}

// +Matrix, -Translation
PREDICATE(matrix_translation, 2) {
	Eigen::Matrix4d m;
	pl2eigen(PL_A1,m);
	Eigen::Vector3d v = m.block<3,1>(0,3,3,1);
	eigen2pl(v,PL_A2);
	return TRUE;
}
