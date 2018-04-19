#include "3d.h"
#include <math.h>

//------------------------------------------------------------------------------

inline
float radians( float degrees ) {
	return ( M_PI / 180.0f ) * degrees;
}

inline
float approach( float goal, float current, float dt ) {
	float diff = goal - current;

	if( diff > dt ) {
		return( current + dt );
	} else if( diff < -dt ) {
		return( current - dt );
	} else {
		return goal;
	}
}

inline
int max_f( float a, float b ) {
	return ( a > b ) ? a : b;
}

//------------------------------------------------------------------------------

	/* VECTOR */

inline
void vector_3d_add( Vector_3d u, Vector_3d v, Vector_3d *out ) {
	out->x = u.x + v.x;
	out->y = u.y + v.y;
	out->z = u.z + v.z;
}

inline
void vector_3d_add_scaled( Vector_3d u, Vector_3d v, Vector_3d *out,
	float scale )
{
	out->x = u.x + v.x * scale;
	out->y = u.y + v.y * scale;
	out->z = u.z + v.z * scale;
}

inline
void vector_3d_sub( Vector_3d u, Vector_3d v, Vector_3d *out ) {
	out->x = u.x - v.x;
	out->y = u.y - v.y;
	out->z = u.z - v.z;
}

inline
float vector_3d_sub_length( Vector_3d u, Vector_3d v ) {
	float w = u.x - v.x;
	float h = u.y - v.y;
	float l = u.z - v.z;
	return sqrtf( w * w + h * h + l * l );
}

inline
void vector_3d_scale( Vector_3d *u, float scalar ) {
	u->x *= scalar;
	u->y *= scalar;
	u->z *= scalar;
}

inline
float vector_3d_dot( Vector_3d u, Vector_3d v ) {
	return u.x * v.x + u.y * v.y + u.z * v.z;
}

inline
void vector_3d_cross( Vector_3d u, Vector_3d v, Vector_3d *out ) {
	out->x = u.y * v.z - u.z * v.y;
	out->y = u.z * v.x - u.x * v.z;
	out->z = u.x * v.y - u.y * v.x;
}

inline
Vector_3d vector_3d_hadamard( Vector_3d u, Vector_3d v ) {
	Vector_3d result;
	result.x = u.x * v.x;
	result.y = u.y * v.y;
	result.z = u.z * v.z;
	return result;
}

inline
float vector_3d_length( Vector_3d u ) {
	return sqrtf( u.x * u.x + u.y * u.y + u.z * u.z );
}

inline
void vector_3d_normalize( Vector_3d *u ) {
	float len = sqrtf( u->x * u->x + u->y * u->y + u->z * u->z );
	if( 0.0f != len ) {
		u->x /= len;
		u->y /= len;
		u->z /= len;
	}
}

inline
void vector_3d_normal( Vector_3d u, Vector_3d v, Vector_3d *out ) {
	float x = u.y * v.z - u.z * v.y;
	float y = u.z * v.x - u.x * v.z;
	float z = u.x * v.y - u.y * v.x;
	float len = sqrtf( x * x + y * y + z * z );
	out->x = x / len;
	out->y = y / len;
	out->z = z / len;
}

//------------------------------------------------------------------------------

	/* QUATERNION */

inline
float quaternion_length( Quaternion *q ) {
	return sqrtf( q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w );
}

inline
void quaternion_normalize( Quaternion *q ) {
	float len = quaternion_length( q );
	q->x /= len;
	q->y /= len;
	q->z /= len;
	q->w /= len;
}

inline
void quaternion_conjugate( const Quaternion *q, Quaternion *r ) {
	r->x = -q->x;
	r->y = -q->y;
	r->z = -q->z;
	r->w = q->w;
}

inline
void quaternion_mul_quaternion( const Quaternion *q, const Quaternion *r,
	Quaternion *out )
{
	out->x =  q->x * r->w + q->y * r->z - q->z * r->y + q->w * r->x;
	out->y = -q->x * r->z + q->y * r->w + q->z * r->x + q->w * r->y;
	out->z =  q->x * r->y - q->y * r->x + q->z * r->w + q->w * r->z;
	out->w = -q->x * r->x - q->y * r->y - q->z * r->z + q->w * r->w;
}

inline
void quaternion_mul_vector( const Quaternion *q, Vector_3d v, Vector_3d *out ) {
	Vector_3d a = { q->x, q->y, q->z }, b, c, d;
	vector_3d_cross( a, v, &b );
	vector_3d_scale( &b, 2.0f );
	vector_3d_cross( a, b, &d );
	vector_3d_scale( &b, q->w );
	vector_3d_add( b, d, &c );
	vector_3d_add( v, c, out );
}

inline
void quaternion_from_axis_angle( Quaternion *q, Vector_3d axis, float angle ) {
	float a = angle * 0.5f;
	float s = sinf( a );
	q->x = axis.x * s;
	q->y = axis.y * s;
	q->z = axis.z * s;
	q->w = cosf( a );
}

// x = heading, y = attitude, z = bank
inline
void quaternion_from_euler_v( Quaternion *q, Vector_3d v ) {
	float rad = v.y * 0.5f;
	float c1 = cosf( rad );
	float s1 = sinf( rad );
	rad = v.z * 0.5f;
	float c2 = cosf( rad );
	float s2 = sinf( rad );
	rad = v.x * 0.5f;
	float c3 = cosf( rad );
	float s3 = sinf( rad );

	float c = c1 * c2;
	float s = s1 * s2;
	q->w = c * c3 + s * s3;
	q->x = c * s3 + s * c3;
	q->y = s1 * c2 * c3 - c1 * s2 * s3;
	q->z = c1 * s2 * c3 - s1 * c2 * s3;
}
// x = heading, y = attitude, z = bank
inline
void quaternion_to_euler( const Quaternion *q, Vector_3d *v ) {
	float test = q->x * q->y + q->z * q->w;

	if( test > 0.499f ) { // Singularity at north pole.
		v->x = 2.0f * atan2f( q->x, q->w );
		v->y = M_PI / 2.0f;
		v->z = 0.0f;
	} else if( test < -0.499f ) { // Singularity at south pole.
		v->x = -2.0f * atan2f( q->x, q->w );
		v->y = -M_PI / 2.0f;
		v->z = 0.0f;
	} else {
		float sqx = q->x * q->x;
		float sqy = q->y * q->y;
		float sqz = q->z * q->z;

		v->x = atan2f( 2.0f * q->y * q->w - 2.0f * q->x * q->z,
			1.0f - 2.0f * ( sqy + sqz ) );
		v->y = asinf( 2.0f * test );
		v->z = atan2f( 2.0f * q->x * q->w - 2.0f * q->y * q->z,
			1.0f - 2.0f * ( sqx + sqz ) );
	}
}
// x = heading, y = attitude, z = bank
inline
void quaternion_to_matrix( const Quaternion *q, Matrix_4x4 *m ) {
	float sqw = q->w * q->w;
	float sqx = q->x * q->x;
	float sqy = q->y * q->y;
	float sqz = q->z * q->z;

	m->_00 =  sqx - sqy - sqz + sqw;
	m->_11 = -sqx + sqy - sqz + sqw;
	m->_22 = -sqx - sqy + sqz + sqw;

	float tmp1 = q->x * q->y;
	float tmp2 = q->z * q->w;
	m->_01 = 2.0f * ( tmp1 + tmp2 );
	m->_10 = 2.0f * ( tmp1 - tmp2 );

	tmp1 = q->x * q->z;
	tmp2 = q->y * q->w;
	m->_02 = 2.0f * ( tmp1 - tmp2 );
	m->_20 = 2.0f * ( tmp1 + tmp2 );

	tmp1 = q->y * q->z;
	tmp2 = q->x * q->w;
	m->_12 = 2.0f * ( tmp1 + tmp2 );
	m->_21 = 2.0f * ( tmp1 - tmp2 );

	m->_30 = 0.0f;
	m->_31 = 0.0f;
	m->_32 = 0.0f;
	m->_03 = 0.0f;
	m->_13 = 0.0f;
	m->_23 = 0.0f;
	m->_33 = 1.0f;
}

inline
void quaternion_slerp( const Quaternion *o, const Quaternion *p, Quaternion *q,
	float theta )
{
	Vector_3d a = { o->x, o->y, o->z };
	Vector_3d b = { p->x, p->y, p->z };
	Quaternion r = *p;
	float k0, k1, cos_omega = o->w * r.w + vector_3d_dot( b, a );
	if( cos_omega < 0.0 ) {
		r.w = -r.w;
		r.x = -b.x;
		r.y = -b.y;
		r.z = -b.z;
		cos_omega = -cos_omega;
	}
	if( cos_omega > 0.9999f ) {
		k0 = 1.0f - theta;
		k1 = theta;
	} else {
		float sin_omega = sqrtf( 1.0f - cos_omega * cos_omega );
		float omega = atan2f( sin_omega, cos_omega );
		float oo_sin_omega = 1.0f / sin_omega;
		k0 = sinf( ( 1.0f - theta ) * omega ) * oo_sin_omega;
		k1 = sinf( theta * omega ) * oo_sin_omega;
	}
	q->w = o->w * k0 + r.w * k1;
	vector_3d_scale( &a, k0 );
	vector_3d_scale( &b, k1 );
	q->x = a.x + b.x;
	q->y = a.y + b.y;
	q->z = a.z + b.z;
}

//------------------------------------------------------------------------------

	/* MATRIX */

const Matrix_4x4 identity_matrix_4x4 = {
	._00 = 1.0f, ._10 = 0.0f, ._20 = 0.0f, ._30 = 0.0f,
	._01 = 0.0f, ._11 = 1.0f, ._21 = 0.0f, ._31 = 0.0f,
	._02 = 0.0f, ._12 = 0.0f, ._22 = 1.0f, ._32 = 0.0f,
	._03 = 0.0f, ._13 = 0.0f, ._23 = 0.0f, ._33 = 1.0f
};

inline
void matrix_4x4_set_translation_v( Matrix_4x4 *m, Vector_3d v ) {
	m->_30 = v.x;
	m->_31 = v.y;
	m->_32 = v.z;
}

inline
void matrix_4x4_set_neg_translation_v( Matrix_4x4 *m, Vector_3d v ) {
	m->_30 = -v.x;
	m->_31 = -v.y;
	m->_32 = -v.z;
}

inline
void matrix_4x4_set_scale( Matrix_4x4 *m, float sx, float sy, float sz ) {
	m->_00 = sx;
	m->_11 = sy;
	m->_22 = sz;
}

inline
void matrix_4x4_mul_vector_3d( const Matrix_4x4* m, Vector_3d v,
	Vector_3d* out )
{
	out->x = v.x * m->_00 + v.y * m->_10 + v.z * m->_20;
	out->y = v.x * m->_01 + v.y * m->_11 + v.z * m->_21;
	out->z = v.x * m->_02 + v.y * m->_12 + v.z * m->_22;
}

inline
void matrix_4x4_mul_vector_4d( const Matrix_4x4* m, Vector_4d v,
	Vector_4d* out )
{
	out->x = v.x * m->_00 + v.y * m->_10 + v.z * m->_20 + v.w * m->_30;
	out->y = v.x * m->_01 + v.y * m->_11 + v.z * m->_21 + v.w * m->_31;
	out->z = v.x * m->_02 + v.y * m->_12 + v.z * m->_22 + v.w * m->_32;
	out->w = v.x * m->_03 + v.y * m->_13 + v.z * m->_23 + v.w * m->_33;
}

//------------------------------------------------------------------------------

/*
inline
void matrix_4x4_perspective_projection( Perspective* perspective,
	Matrix_4x4* out )
{
	float uh = 1.0f / tanf( 0.5f * perspective->fov );
	float one_over_depth = 1.0f / ( perspective->far - perspective->near );

	out->_00 = uh / perspective->view_aspect;
	out->_11 = uh;
	out->_22 = -( perspective->far + perspective->near ) * one_over_depth;
	out->_23 = -1.0f;
	out->_32 = ( -2.0f * perspective->far * perspective->near )
		* one_over_depth;
//	out->_23 = ( -2.0f * perspective->far * perspective->near )
//		* one_over_depth;
//	out->_32 = -1.0f;
	out->_33 = 0.0f;
}
*/

inline
void matrix_4x4_project_frustum( Matrix_4x4 *m,
	float l, float r, float b, float t, float n, float f )
{
	float xd = r - l;
	float yd = t - b;
	float zd = f - n;
	float n2 = 2.0f * n;

	m->_00 = n2 / xd;
	m->_11 = n2 / yd;
	m->_20 = ( r + l ) / xd;
	m->_21 = ( t + b ) / yd;
	m->_22 = -( f + n ) / zd;
	m->_23 = -1.0f;
	m->_32 = -( f * n2 ) / zd;
	m->_33 = 0.0f;
}

inline
void matrix_4x4_perspective_projection( Perspective* perspective,
	Matrix_4x4* m )
{
	float r = perspective->near * tanf( 0.5f * perspective->fov );
	float l = -r;
	float b = l / perspective->view_aspect;
	float t = r / perspective->view_aspect;
	matrix_4x4_project_frustum( m, l, r, b, t,
		perspective->near, perspective->far );
}

inline
void matrix_4x4_set_orthographic_projection( Matrix_4x4 *out,
	float near, float far, float left, float right, float top, float bottom )
{
	out->_00 = 2.0f / ( right - left );
	out->_01 = 0.0f;
	out->_02 = 0.0f;
	out->_03 = -( ( right + left ) / ( right - left ) );
	out->_10 = 0.0f;
	out->_11 = 2.0f / ( top - bottom );
	out->_12 = 0.0f;
	out->_13 = -( ( top + bottom ) / ( top - bottom ) );
	out->_20 = 0.0f;
	out->_21 = 0.0f;
	out->_22 = -2.0f / ( far - near );
	out->_23 = -( ( far + near ) / ( far - near ) );
	out->_30 = 0.0f;
	out->_31 = 0.0f;
	out->_32 = 0.0f;
	out->_33 = 1.0f;
}

inline
void matrix_4x4_transpose( const Matrix_4x4 *m, Matrix_4x4 *out ) {
	out->_00 = m->_00;
	out->_01 = m->_10;
	out->_02 = m->_20;
	out->_03 = m->_30;
	out->_10 = m->_01;
	out->_11 = m->_11;
	out->_12 = m->_21;
	out->_13 = m->_31;
	out->_20 = m->_02;
	out->_21 = m->_12;
	out->_22 = m->_22;
	out->_23 = m->_32;
	out->_30 = m->_03;
	out->_31 = m->_13;
	out->_32 = m->_23;
	out->_33 = m->_33;
};

inline
void matrix_4x4_invert_tr( const Matrix_4x4 *m, Matrix_4x4 *out ) {
	/* This will only work if m is a translation/rotation matrix. */
	Vector_3d s, t = { .x = m->_30, .y = m->_31, .z = m->_32 };
	out->_00 = m->_00;
	out->_01 = m->_10;
	out->_02 = m->_20;
	out->_03 = 0.0f;
	out->_10 = m->_01;
	out->_11 = m->_11;
	out->_12 = m->_21;
	out->_13 = 0.0f;
	out->_20 = m->_02;
	out->_21 = m->_12;
	out->_22 = m->_22;
	out->_23 = 0.0f;
	matrix_4x4_mul_vector_3d( out, t, &s );
	out->_30 = -s.x;
	out->_31 = -s.y;
	out->_32 = -s.z;
	out->_33 = 1.0f;
}

//------------------------------------------------------------------------------

/*
	out->x = v.x * m->_00 + v.x * m->_10 + v.x * m->_20 + v.x * m->_30;
	out->y = v.y * m->_01 + v.y * m->_11 + v.y * m->_21 + v.y * m->_31;
	out->z = v.z * m->_02 + v.z * m->_12 + v.z * m->_22 + v.z * m->_32;
	out->w = v.w * m->_03 + v.w * m->_13 + v.w * m->_23 + v.w * m->_33;
*/
// Cij = Ai1 * B1j + Ai2 * B2j + ... + Ain * Bnj
void matrix_4x4_mul_matrix( const Matrix_4x4 *m, const Matrix_4x4 *n,
	Matrix_4x4 *out )
{
	out->_00 = m->_00 * n->_00 + m->_10 * n->_01 + m->_20 * n->_02 + m->_30 * n->_03;
	out->_01 = m->_01 * n->_00 + m->_11 * n->_01 + m->_21 * n->_02 + m->_31 * n->_03;
	out->_02 = m->_02 * n->_00 + m->_12 * n->_01 + m->_22 * n->_02 + m->_32 * n->_03;
	out->_03 = m->_03 * n->_00 + m->_13 * n->_01 + m->_23 * n->_02 + m->_33 * n->_03;

	out->_10 = m->_00 * n->_10 + m->_10 * n->_11 + m->_20 * n->_12 + m->_30 * n->_13;
	out->_11 = m->_01 * n->_10 + m->_11 * n->_11 + m->_21 * n->_12 + m->_31 * n->_13;
	out->_12 = m->_02 * n->_10 + m->_12 * n->_11 + m->_22 * n->_12 + m->_32 * n->_13;
	out->_13 = m->_03 * n->_10 + m->_13 * n->_11 + m->_23 * n->_12 + m->_33 * n->_13;

	out->_20 = m->_00 * n->_20 + m->_10 * n->_21 + m->_20 * n->_22 + m->_30 * n->_23;
	out->_21 = m->_01 * n->_20 + m->_11 * n->_21 + m->_21 * n->_22 + m->_31 * n->_23;
	out->_22 = m->_02 * n->_20 + m->_12 * n->_21 + m->_22 * n->_22 + m->_32 * n->_23;
	out->_23 = m->_03 * n->_20 + m->_13 * n->_21 + m->_23 * n->_22 + m->_33 * n->_23;

	out->_30 = m->_00 * n->_30 + m->_10 * n->_31 + m->_20 * n->_32 + m->_30 * n->_33;
	out->_31 = m->_01 * n->_30 + m->_11 * n->_31 + m->_21 * n->_32 + m->_31 * n->_33;
	out->_32 = m->_02 * n->_30 + m->_12 * n->_31 + m->_22 * n->_32 + m->_32 * n->_33;
	out->_33 = m->_03 * n->_30 + m->_13 * n->_31 + m->_23 * n->_32 + m->_33 * n->_33;
}

Vector_4d clip_to_eye_coordinates( const Matrix_4x4 *projection, Vector_4d u ) {
	Matrix_4x4 inverted;
	matrix_4x4_invert_tr( projection, &inverted );
	Vector_4d v;
	matrix_4x4_mul_vector_4d( &inverted, u, &v );
	v.z = -1.0f;
	v.w = 0.0f;
	return v;
}

Vector_3d eye_to_world_coordinates( const Matrix_4x4 *view, Vector_4d u ) {
	Vector_3d result;
	Vector_4d v;
	Matrix_4x4 inverted;
	matrix_4x4_invert_tr( view, &inverted );
	matrix_4x4_mul_vector_4d( &inverted, u, &v );
	result.x = v.x;
	result.y = v.y;
	result.z = v.z;
	vector_3d_normalize( &result );
	return result;
}

void look_at_r( const Vector_3d *eye, const Vector_3d *at, const Vector_3d *up,
	Matrix_4x4 *out )
{
//	Vector_3d fwd;
//	vector_3d_sub( *at, *eye, &fwd );
//	vector_3d_sub( *eye, *at, &fwd );
//	vector_3d_normalize( &fwd );

	Vector_3d axis, right;
//	vector_3d_cross( fwd, *up, &right );
	vector_3d_cross( *at, *up, &right );
	vector_3d_normalize( &right );
	vector_3d_cross( right, *at, &axis );
	Matrix_4x4 rot;
	rot._00 = right.x;
	rot._10 = right.y;
	rot._20 = right.z;
	rot._30 = 0.0f;
	rot._01 = axis.x;
	rot._11 = axis.y;
	rot._21 = axis.z;
	rot._31 = 0.0f;

	rot._02 = -at->x;
	rot._12 = -at->y;
	rot._22 = -at->z;

//	rot._02 = fwd.x;
//	rot._12 = fwd.y;
//	rot._22 = fwd.z;
	rot._32 = 0.0f;

	rot._03 = eye->x;
	rot._13 = eye->y;
	rot._23 = eye->z;
	rot._33 = 1.0f;
	matrix_4x4_invert_tr( &rot, out );
}

void look_at_l( const Vector_3d *eye, const Vector_3d *at, const Vector_3d *up,
	Matrix_4x4 *out )
{
	Vector_3d axis, fwd, left;
	vector_3d_sub( *eye, *at, &fwd ); // reversed for camera transform
	vector_3d_normalize( &fwd );
	vector_3d_cross( *up, fwd, &left );
	vector_3d_normalize( &left );
	vector_3d_cross( fwd, left, &axis );

	out->_00 = left.x;
	out->_10 = axis.x;
	out->_20 = fwd.x;
	out->_30 = 0.0f;
	out->_01 = left.y;
	out->_11 = axis.y;
	out->_21 = fwd.y;
	out->_31 = 0.0f;
	out->_02 = left.z;
	out->_12 = axis.z;
	out->_22 = fwd.z;
	out->_32 = 0.0f;
	out->_03 = -left.x * eye->x - left.y * eye->y - left.z * eye->z;
	out->_13 = -axis.x * eye->x - axis.y * eye->y - axis.z * eye->z;
	out->_23 = -fwd.x * eye->x - fwd.y * eye->y - fwd.z * eye->z;
	out->_33 = 1.0f;
}

inline
int matrix_4x4_invert( const Matrix_4x4 *m, Matrix_4x4 *out ) {
	float det;
	Matrix_4x4 inv;

	inv.array[ 0 ] = m->array[ 5 ] * m->array[ 10 ] * m->array[ 15 ] -
             m->array[ 5 ]  * m->array[ 11 ] * m->array[ 14 ] -
             m->array[ 9 ]  * m->array[ 6 ]  * m->array[ 15 ] +
             m->array[ 9 ]  * m->array[ 7 ]  * m->array[ 14 ] +
             m->array[ 13 ] * m->array[ 6 ]  * m->array[ 11 ] -
             m->array[ 13 ] * m->array[ 7 ]  * m->array[ 10 ];

	inv.array[ 4 ] = -m->array[ 4 ] * m->array[ 10 ] * m->array[ 15 ] +
              m->array[ 4 ]  * m->array[ 11 ] * m->array[ 14 ] +
              m->array[ 8 ]  * m->array[ 6 ]  * m->array[ 15 ] -
              m->array[ 8 ]  * m->array[ 7 ]  * m->array[ 14 ] -
              m->array[ 12 ] * m->array[ 6 ]  * m->array[ 11 ] +
              m->array[ 12 ] * m->array[ 7 ]  * m->array[ 10 ];

	inv.array[ 8 ] = m->array[ 4 ] * m->array[ 9 ] * m->array[ 15 ] -
             m->array[ 4 ]  * m->array[ 11 ] * m->array[ 13 ] -
             m->array[ 8 ]  * m->array[ 5 ] * m->array[ 15 ] +
             m->array[ 8 ]  * m->array[ 7 ] * m->array[ 13 ] +
             m->array[ 12 ] * m->array[ 5 ] * m->array[ 11 ] -
             m->array[ 12 ] * m->array[ 7 ] * m->array[ 9 ];

	inv.array[ 12 ] = -m->array[ 4 ] * m->array[ 9 ] * m->array[ 14 ] +
               m->array[ 4 ]  * m->array[ 10 ] * m->array[ 13 ] +
               m->array[ 8 ]  * m->array[ 5 ] * m->array[ 14 ] -
               m->array[ 8 ]  * m->array[ 6 ] * m->array[ 13 ] -
               m->array[ 12 ] * m->array[ 5 ] * m->array[ 10 ] +
               m->array[ 12 ] * m->array[ 6 ] * m->array[ 9 ];

	inv.array[ 1 ] = -m->array[ 1 ] * m->array[ 10 ] * m->array[ 15 ] +
              m->array[ 1 ]  * m->array[ 11 ] * m->array[ 14 ] +
              m->array[ 9 ]  * m->array[ 2 ] * m->array[ 15 ] -
              m->array[ 9 ]  * m->array[ 3 ] * m->array[ 14 ] -
              m->array[ 13 ] * m->array[ 2 ] * m->array[ 11 ] +
              m->array[ 13 ] * m->array[ 3 ] * m->array[ 10 ];

	inv.array[ 5 ] = m->array[ 0 ] * m->array[ 10 ] * m->array[ 15 ] -
             m->array[ 0 ]  * m->array[ 11 ] * m->array[ 14 ] -
             m->array[ 8 ]  * m->array[ 2 ] * m->array[ 15 ] +
             m->array[ 8 ]  * m->array[ 3 ] * m->array[ 14 ] +
             m->array[ 12 ] * m->array[ 2 ] * m->array[ 11 ] -
             m->array[ 12 ] * m->array[ 3 ] * m->array[ 10 ];

	inv.array[ 9 ] = -m->array[ 0 ] * m->array[ 9 ] * m->array[ 15 ] +
              m->array[ 0 ]  * m->array[ 11 ] * m->array[ 13 ] +
              m->array[ 8 ]  * m->array[ 1 ] * m->array[ 15 ] -
              m->array[ 8 ]  * m->array[ 3 ] * m->array[ 13 ] -
              m->array[ 12 ] * m->array[ 1 ] * m->array[ 11 ] +
              m->array[ 12 ] * m->array[ 3 ] * m->array[ 9 ];

	inv.array[ 13 ] = m->array[ 0 ] * m->array[ 9 ] * m->array[ 14 ] -
              m->array[ 0 ]  * m->array[ 10 ] * m->array[ 13 ] -
              m->array[ 8 ]  * m->array[ 1 ] * m->array[ 14 ] +
              m->array[ 8 ]  * m->array[ 2 ] * m->array[ 13 ] +
              m->array[ 12 ] * m->array[ 1 ] * m->array[ 10 ] -
              m->array[ 12 ] * m->array[ 2 ] * m->array[ 9 ];

	inv.array[ 2 ] = m->array[ 1 ] * m->array[ 6 ] * m->array[ 15 ] -
             m->array[ 1 ]  * m->array[ 7 ] * m->array[ 14 ] -
             m->array[ 5 ]  * m->array[ 2 ] * m->array[ 15 ] +
             m->array[ 5 ]  * m->array[ 3 ] * m->array[ 14 ] +
             m->array[ 13 ] * m->array[ 2 ] * m->array[ 7 ] -
             m->array[ 13 ] * m->array[ 3 ] * m->array[ 6 ];

	inv.array[ 6 ] = -m->array[ 0 ] * m->array[ 6 ] * m->array[ 15 ] +
              m->array[ 0 ]  * m->array[ 7 ] * m->array[ 14 ] +
              m->array[ 4 ]  * m->array[ 2 ] * m->array[ 15 ] -
              m->array[ 4 ]  * m->array[ 3 ] * m->array[ 14 ] -
              m->array[ 12 ] * m->array[ 2 ] * m->array[ 7 ] +
              m->array[ 12 ] * m->array[ 3 ] * m->array[ 6 ];

	inv.array[ 10 ] = m->array[ 0 ] * m->array[ 5 ] * m->array[ 15 ] -
              m->array[ 0 ]  * m->array[ 7 ] * m->array[ 13 ] -
              m->array[ 4 ]  * m->array[ 1 ] * m->array[ 15 ] +
              m->array[ 4 ]  * m->array[ 3 ] * m->array[ 13 ] +
              m->array[ 12 ] * m->array[ 1 ] * m->array[ 7 ] -
              m->array[ 12 ] * m->array[ 3 ] * m->array[ 5 ];

	inv.array[ 14 ] = -m->array[ 0 ] * m->array[ 5 ] * m->array[ 14 ] +
               m->array[ 0 ]  * m->array[ 6 ] * m->array[ 13 ] +
               m->array[ 4 ]  * m->array[ 1 ] * m->array[ 14 ] -
               m->array[ 4 ]  * m->array[ 2 ] * m->array[ 13 ] -
               m->array[ 12 ] * m->array[ 1 ] * m->array[ 6 ] +
               m->array[ 12 ] * m->array[ 2 ] * m->array[ 5 ];

	inv.array[ 3 ] = -m->array[ 1 ] * m->array[ 6 ] * m->array[ 11 ] +
              m->array[ 1 ] * m->array[ 7 ] * m->array[ 10 ] +
              m->array[ 5 ] * m->array[ 2 ] * m->array[ 11 ] -
              m->array[ 5 ] * m->array[ 3 ] * m->array[ 10 ] -
              m->array[ 9 ] * m->array[ 2 ] * m->array[ 7 ] +
              m->array[ 9 ] * m->array[ 3 ] * m->array[ 6 ];

	inv.array[ 7 ] = m->array[ 0 ] * m->array[ 6 ] * m->array[ 11 ] -
             m->array[ 0 ] * m->array[ 7 ] * m->array[ 10 ] -
             m->array[ 4 ] * m->array[ 2 ] * m->array[ 11 ] +
             m->array[ 4 ] * m->array[ 3 ] * m->array[ 10 ] +
             m->array[ 8 ] * m->array[ 2 ] * m->array[ 7 ] -
             m->array[ 8 ] * m->array[ 3 ] * m->array[ 6 ];

	inv.array[ 11 ] = -m->array[ 0 ] * m->array[ 5 ] * m->array[ 11 ] +
               m->array[ 0 ] * m->array[ 7 ] * m->array[ 9 ] +
               m->array[ 4 ] * m->array[ 1 ] * m->array[ 11 ] -
               m->array[ 4 ] * m->array[ 3 ] * m->array[ 9 ] -
               m->array[ 8 ] * m->array[ 1 ] * m->array[ 7 ] +
               m->array[ 8 ] * m->array[ 3 ] * m->array[ 5 ];

	inv.array[ 15 ] = m->array[ 0 ] * m->array[ 5 ] * m->array[ 10 ] -
              m->array[ 0 ] * m->array[ 6 ] * m->array[ 9 ] -
              m->array[ 4 ] * m->array[ 1 ] * m->array[ 10 ] +
              m->array[ 4 ] * m->array[ 2 ] * m->array[ 9 ] +
              m->array[ 8 ] * m->array[ 1 ] * m->array[ 6 ] -
              m->array[ 8 ] * m->array[ 2 ] * m->array[ 5 ];

	det = m->array[ 0 ] * inv.array[ 0 ] + m->array[ 1 ] * inv.array[ 4 ]
		+ m->array[ 2 ] * inv.array[ 8 ] + m->array[ 3 ] * inv.array[ 12 ];

	if( 0.0f < det || 0.0f > det ) {
		det = 1.0f / det;
		int i;
		for( i = 0; i < 16; ++i ) {
			out->array[ i ] = inv.array[ i ] * det;
		}
		return 1;
	}
	return 0;
}
