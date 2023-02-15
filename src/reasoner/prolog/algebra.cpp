
#include <Eigen/Geometry>

#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

void eigen2pl(const Eigen::Quaterniond &q, term_t &out) {
	PlTail l(out);
	l.append(q.x());
	l.append(q.y());
	l.append(q.z());
	l.append(q.w());
	l.close();
}

void eigen2pl(const Eigen::Vector3d &v, term_t &out) {
	PlTail l(out);
	l.append(v.x());
	l.append(v.y());
	l.append(v.z());
	l.close();
}

void eigen2pl(const Eigen::Matrix4d &m, term_t &out) {
	PlTail l(out);
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j) {
			l.append(m(i,j));
		}
	}
	l.close();
}

void pl2eigen(term_t &arg, Eigen::Quaterniond &q) {
	PlTail list(arg); PlTerm value;
	list.next(value); q.x() = value;
	list.next(value); q.y() = value;
	list.next(value); q.z() = value;
	list.next(value); q.w() = value;
}

void pl2eigen(term_t &arg, Eigen::Vector3d &v) {
	PlTail list(arg); PlTerm value;
	list.next(value); v.x() = value;
	list.next(value); v.y() = value;
	list.next(value); v.z() = value;
}

void pl2eigen(term_t &arg, Eigen::Matrix4d &m) {
	PlTail list(arg); PlTerm value;
	for(int i=0; i<4; ++i) {
		for(int j=0; j<4; ++j) {
			list.next(value);
			m(i,j) = value;
		}
	}
}

// +Quaternion, -Inverse
foreign_t pl_quaternion_inverse(term_t t_Quaternion, term_t t_Inverse)
{
	Eigen::Quaterniond q;
	pl2eigen(t_Quaternion,q);
	eigen2pl(q.inverse(),t_Inverse);
	return TRUE;
}

// +Quaternion, +Vector, -Transformed
foreign_t pl_quaternion_transform(term_t t_Quaternion, term_t t_Vector, term_t t_Transformed)
{
	Eigen::Quaterniond q;
	Eigen::Vector3d v;
	pl2eigen(t_Quaternion,q);
	pl2eigen(t_Vector,v);
	eigen2pl(q*v,t_Transformed);
	return TRUE;
}

// +Quaternion1, +Quaternion2, -Multiplied
foreign_t pl_quaternion_multiply(term_t t_Quaternion1, term_t t_Quaternion2, term_t t_Multiplied)
{
	Eigen::Quaterniond q1,q2;
	pl2eigen(t_Quaternion1,q1);
	pl2eigen(t_Quaternion2,q2);
	eigen2pl(q1*q2,t_Multiplied);
	return TRUE;
}

// +Quaternion1, +Quaternion2, -Diff
foreign_t pl_quaternion_diff(term_t t_Quaternion1, term_t t_Quaternion2, term_t t_Diff)
{
	Eigen::Quaterniond q1,q2;
	pl2eigen(t_Quaternion1,q1);
	pl2eigen(t_Quaternion2,q2);
	eigen2pl(q1.inverse()*q2,t_Diff);
	return TRUE;
}

// +Quaternion1, +Quaternion2, +Factor, -Interpolated
foreign_t pl_quaternion_slerp(term_t t_Quaternion1, term_t t_Quaternion2, term_t t_Factor, term_t t_Interpolated)
{
	Eigen::Quaterniond q1,q2;
	pl2eigen(t_Quaternion1,q1);
	pl2eigen(t_Quaternion2,q2);
	double factor = 0.0;
	if(!PL_get_float(t_Factor, &factor)) return FALSE;
	Eigen::Quaterniond q = q1.slerp(factor,q2);
	eigen2pl(q,t_Interpolated);
	return TRUE;
}

// +Quaternion, +Translation, -Matrix
foreign_t pl_quaternion_matrix(term_t t_Quaternion, term_t t_Translation, term_t t_Matrix)
{
	Eigen::Quaterniond q;
	Eigen::Vector3d v;
	pl2eigen(t_Quaternion,q);
	pl2eigen(t_Translation,v);
	Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
	// 3x3 block starting at (0,0)
	m.block(0,0,3,3) = q.toRotationMatrix();
	// 3x1 block starting at (0,3)
	m.block(0,3,3,1) = v;
	eigen2pl(m,t_Matrix);
	return TRUE;
}

// +Matrix, -Quaternion
foreign_t pl_matrix_quaternion(term_t t_Matrix, term_t t_Quaternion)
{
	Eigen::Matrix4d m4;
	pl2eigen(t_Matrix,m4);
	Eigen::Matrix3d m3 = m4.block(0,0,3,3);
	Eigen::Quaterniond q(m3);
	eigen2pl(q,t_Quaternion);
	return TRUE;
}

// +Matrix, -Translation
foreign_t pl_matrix_translation(term_t t_Matrix, term_t t_Translation)
{
	Eigen::Matrix4d m;
	pl2eigen(t_Matrix,m);
	Eigen::Vector3d v = m.block<3,1>(0,3,3,1);
	eigen2pl(v,t_Translation);
	return TRUE;
}

PL_extension algebra_predicates[] = {
		{ "quaternion_inverse", 2, (pl_function_t)pl_quaternion_inverse, 0 },
		{ "quaternion_transform", 3, (pl_function_t)pl_quaternion_transform, 0 },
		{ "quaternion_multiply", 3, (pl_function_t)pl_quaternion_multiply, 0 },
		{ "quaternion_diff", 3, (pl_function_t)pl_quaternion_diff, 0 },
		{ "quaternion_slerp", 4, (pl_function_t)pl_quaternion_slerp, 0 },
		{ "quaternion_matrix", 4, (pl_function_t)pl_quaternion_matrix, 0 },
		{ "matrix_quaternion", 4, (pl_function_t)pl_matrix_quaternion, 0 },
		{ "matrix_translation", 2, (pl_function_t)pl_matrix_translation, 0 },
		{nullptr, 0, nullptr, 0 }
};
