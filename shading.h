#ifndef CTOOL_SHADING
#define CTOOL_SHADING

enum {
	PROGRAM_MODEL,
	PROGRAM_MAX
};

enum { /* uniform locations */
	ULOC_AMBIENT_COEFF,
	ULOC_ATLAS,
	ULOC_COLOR_BG,
	ULOC_COLOR_FG,
	ULOC_FLAGS,
	ULOC_INTENSITIES,
	ULOC_LOCATION,
	ULOC_MODEL,
	ULOC_PROJECTION,
	ULOC_RADIUS,
	ULOC_SCALE,
	ULOC_TEXTURE0,
	ULOC_TEXTURE1,
	ULOC_TEXTURE2,
	ULOC_TRANSLATION,
	ULOC_VIEW,
	ULOC_MAX // 16 assigned
};

typedef struct {
	u16 program_id;
	u16 num_tex_bindings;
	s16 uniform_locations[ ULOC_MAX ];
} Shader;

#endif /* CTOOL_SHADING */
