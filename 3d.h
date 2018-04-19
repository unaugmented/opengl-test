#ifndef CTOOL_3D
#define CTOOL_3D

#include "types.h"
#include <math.h>

//------------------------------------------------------------------------------

struct vector3f {
	float x, y, z;
};

struct vector4f {
	float x, y, z, w;
};

typedef struct vector3f Vector_3d;
typedef struct vector4f Vector_4d;
typedef struct vector4f Quaternion;
typedef struct vector4f Color;

typedef union {
	float array[ 16 ];
	struct { // row major
		float
			_00, _10, _20, _30,
			_01, _11, _21, _31,
			_02, _12, _22, _32,
			_03, _13, _23, _33;
	};
} Matrix_4x4;

typedef struct {
	Quaternion rotation;
	Vector_3d position;
} Camera;

typedef struct {
	Vector_4d position; // w = directional ? 0 : 1
	Color intensities; // .a = attenuation
	float ambient_coefficient;
} Light;

typedef struct {
	Vector_3d position;
	Color intensities; // .a = attenuation
	float ambient_coefficient;
	Quaternion rotation;
} Directional_Light;

typedef struct {
	int type;				/*! Projection type. */
	float fov,				/*! Field of view angle. */
		near,				/*! Z distance of the near plane. */
		far,				/*! -"- far plane. */
		view_aspect,		/*! The view aspect ratio. */
		view_oo_aspect,		/*! One over the aspect ratio. */
		ortho_scale_x,		/*! Ortho scale factor in X. */
		ortho_scale_y;		/*! -"- in Y. */
} Perspective;

#endif /* CTOOL_3D */
