#ifndef CTOOL_ASSETS
#define CTOOL_ASSETS

#include "types.h"

enum { VBO_SEPARATE, VBO_BLOCKED, VBO_INTERLEAVED };

typedef struct { /*! Represents a vertex array object. */
	u16 vao;				/*! Handle to a vao. */
	u16 vbo;				/*! Handle to the vertex buffer. */
	u16 len;				/*! Number of elements to draw. */
	union {
		u16 ind;			/*! Handle to the index buffer. */
		u16 draw_mode;		/*! OpenGL draw mode. */
	};
} Vao;

typedef struct {
	u16 tex_id;
	u16 width;
	u16 height;
	u16 depth;
	s16 x;
	s16 y;
	s16 scale_x;
	s16 scale_y;
} Texture;

typedef struct { /*! For generating textures in OpenGL. */
	u32 target;
	u32 internal_format;
	u32 format;
	u32 type;
	u32 min_filter;
	u32 mag_filter;
	u32 wrap_s;
	u32 wrap_t;
	u16 mipmap;
	u16 anisotropy;
	u16 modulate;
	u16 pad_unused;
} Texture_Info;

typedef struct {
	u16 version;
	u16 num_joints;
	u16 num_animations;
	u16 type;
	u16 vertex_size;
	u16 normal_size;
	u16 coord_size;
	u16 index_size;
} MOB_Header;

#define KTX_UNPACK_ALIGNMENT	(4U)

typedef struct {
	u8 identifier[ 12 ];
	u32 endianess;
	u32 type;
	u32 type_size;
	u32 format;
	u32 internal_format;
	u32 base_internal_format;
	u32 pixel_width;
	u32 pixel_height;
	u32 pixel_depth;
	u32 num_array_elements;
	u32 num_faces;
	u32 num_mipmap_levels;
	u32 num_bytes_key_value;
} KTX_Header;

#endif /* CTOOL_ASSETS */
