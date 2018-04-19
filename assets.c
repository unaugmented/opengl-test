#include "assets.h"

/*! Creates a vao for a 3d model (position, normals and uv). */
void mk_indexed_model( Vao *obj, u32 num_vertices, const float *vertices,
	u32 idx_type_size, u32 idx_count, const void *indices, GLenum usage,
	int skinned )
{
	u32 vao, buffers[ 2 ];

	glGenVertexArrays( 1, &vao );
	glGenBuffers( 2, buffers );
	glBindVertexArray( vao );
	glBindBuffer( GL_ARRAY_BUFFER, buffers[ 0 ] );
	glBufferData( GL_ARRAY_BUFFER, num_vertices * sizeof( float ), vertices,
		usage );
	glEnableVertexAttribArray( 0 );
//	u32 sz = skinned ? ( 2 * MAX_NUM_INFLUENCES + 8 ) : 8;
	u32 sz = 8 * sizeof( float );
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sz, 0 );
	glEnableVertexAttribArray( 1 );
	glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, sz,
		( const GLvoid * ) ( 3 * sizeof( float ) ) );
	glEnableVertexAttribArray( 2 );
	glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, sz,
		( const GLvoid * ) ( 6 * sizeof( float ) ) );

	glBindBuffer( GL_ARRAY_BUFFER, 0 );
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, buffers[ 1 ] );
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, idx_count * idx_type_size, indices,
		usage );
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	obj->vao = vao;
	obj->vbo = buffers[ 0 ];
	obj->len = idx_count;
	obj->ind = buffers[ 1 ];
}

void read_mob( const char* file, u16* v_size, float** v_data,
	u16* i_size, u16** i_data, int* skinned )
{
	const u16 dae_version = 141;
	long len, tmp_v, tmp_i;
	MOB_Header header;

	FILE *fh = fopen( file, "r" );
	if( !fh ) {
		fprintf( stderr, "Could not read file %s\n", file );
		return;
	}
	fseek( fh, 0, SEEK_END );
	len = ftell( fh );
	fseek( fh, 0, SEEK_SET );
	fread( &header, 1, sizeof( MOB_Header ), fh );

	if( dae_version != header.version ) {
		fprintf( stderr, "Version %u invalid.\n", header.version );
		return;
	}
	if( VBO_INTERLEAVED != header.type ) {
		fprintf( stderr, "Type %u invalid.\n", header.type );
		return;
	}
	tmp_v = header.vertex_size * sizeof( float );
	tmp_i = header.index_size * sizeof( u16 );

	if( len != ( sizeof( MOB_Header ) + tmp_i + tmp_v ) ) {
		fprintf( stderr, "File length %lu inconsistent, vertex size: %"
			PRIu16 ", index size: %" PRIu16  ".\n",
			len, header.vertex_size, header.index_size );
		return;
	}
	u8 *buffer1 = calloc( header.vertex_size, sizeof( float ) );
	long read = fread( buffer1, 1, tmp_v, fh );

	if( read != tmp_v ) {
		free( buffer1 );
		fprintf( stderr,
			"Expected to read %ld bytes for vertices, but got %ld\n",
			read, tmp_v );
		return;
	}
	u8 *buffer2 = calloc( header.index_size, sizeof( u16 ) );
	read = fread( buffer2, 1, tmp_i, fh );

	if( read != tmp_i ) {
		free( buffer1 );
		free( buffer2 );
		fprintf( stderr,
			"Expected to read %ld bytes for indices, but got %ld\n",
			read, tmp_i );
		return;
	}
	fclose( fh );
	*v_data = ( float* ) buffer1;
	*i_data = ( u16* ) buffer2;
	*skinned = header.num_joints;
	*v_size = tmp_v / sizeof( float );
	*i_size = tmp_i / sizeof( u16 );
}

GLuint generate_texture( int w, int h, Texture_Info *info, const void *data ) {
	GLuint tex;
	glGenTextures( 1, &tex );
	glBindTexture( info->target, tex );

	if( GL_TEXTURE_1D == info->target ) {
		glTexImage1D( info->target, 0, info->internal_format, w, 0,
			info->format, info->type, data );
	} else if( GL_TEXTURE_2D == info->target ) {
		glTexImage2D( info->target, 0, info->internal_format, w, h, 0,
			info->format, info->type, data );
	}
	if( info->mipmap ) {
		glGenerateMipmap( info->target );
	}
	glTexParameteri( info->target, GL_TEXTURE_MIN_FILTER, info->min_filter );
	glTexParameteri( info->target, GL_TEXTURE_MAG_FILTER, info->mag_filter );
	glTexParameteri( info->target, GL_TEXTURE_WRAP_S, info->wrap_s );
	glTexParameteri( info->target, GL_TEXTURE_WRAP_T, info->wrap_t );

	if( info->modulate ) {
//		glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
		glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_BLEND );
	}
#if defined GL_EXT_texture_filter_anisotropic
	if( info->anisotropy ) {
		float anisotropic, max, k = 4.0;
		glGetFloatv( GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max );
		anisotropic = max_f( k, max );
		glTexParameterf( info->target, GL_TEXTURE_MAX_ANISOTROPY_EXT,
			anisotropic );
	}
#endif
	GLint actual_format;
	glGetTexLevelParameteriv( GL_TEXTURE_2D, 0,
		GL_TEXTURE_INTERNAL_FORMAT, &actual_format );
	glBindTexture( info->target, 0 );
	return tex;
}

int load_ktx( const char* file, u32 *tex_id,
	GLint min_filter, GLint mag_filter,
	GLint wrap_s, GLint wrap_t, int *img_width, int *img_height,
	GLint unpack_override )
{
	const u8 ktx_identifier[ 12 ] = {
		0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A
	};
	s32 unpack_alignment;
	u32 size, i, pad = 0; // convert
	KTX_Header ktx_header;
	FILE *fh = fopen( file, "r" );
	if( !fh ) {
		fprintf( stderr, "Could not read file %s\n", file );
		return -1;
	}
	fread( &ktx_header, 1, sizeof( KTX_Header ), fh );

	for( i = 0; i < 12; ++i ) {
		if( ktx_header.identifier[ i ] != ktx_identifier[ i ] ) {
			fprintf( stderr,
				"Invalid ktx identifier in file %s:\n %s,\n expected %s\n",
				file, ktx_header.identifier, ktx_identifier );
			fclose( fh );
			return -1;
		}
	}
//	convert = ( ktx_header.endianess == 0x01020304 ) ? 1 : 0;
	if( ktx_header.pixel_width == 0 ) ktx_header.pixel_width = 1;
	if( ktx_header.pixel_depth == 0 ) ktx_header.pixel_depth = 1;
	if( ktx_header.num_mipmap_levels == 0 ) ktx_header.num_mipmap_levels = 1;
	if( ktx_header.num_array_elements == 0 ) ktx_header.num_array_elements = 1;
	if( ktx_header.num_faces == 0 ) ktx_header.num_faces = 1;
	// skip meta data for now
	fseek( fh, ktx_header.num_bytes_key_value, SEEK_CUR );
	glGetIntegerv( GL_UNPACK_ALIGNMENT, &unpack_alignment );

	if( KTX_UNPACK_ALIGNMENT != unpack_alignment ) {
		glPixelStorei( GL_UNPACK_ALIGNMENT, unpack_override );
	}
	if( 1 != ktx_header.num_mipmap_levels ) {
		fprintf( stderr,
			"Error: loader for ktx files supports only one mip-map level.\n" );
		return -1;
	}
	i = 0;
//	while( i < ktx_header.num_mipmap_levels ) {
		fread( &size, 1, 4, fh );
		u32 s = ktx_header.pixel_width * ktx_header.pixel_height;

		if( ktx_header.format == GL_RGB ) {
			if( 3 * s != size ) {
				fprintf( stderr,
					"Image size %u is not equal 3 * width * height: %u.\n",
					size, s );
			}
		}
		pad = 4 - ( size % 4 );
		i += ( size + pad );
		u8 *pixel_data = malloc( size );

		if( !pixel_data ) {
			fclose( fh );
			return -1;
		}
		fread( pixel_data, 1, size, fh );
		// if( 1 == convert ) { whoever has big endian could implemented that }
		Texture_Info info = { GL_TEXTURE_2D,
			ktx_header.internal_format, ktx_header.format, GL_UNSIGNED_BYTE,
			min_filter, mag_filter, wrap_s, wrap_t, 1, 1, 0 };
		*tex_id = generate_texture(
			ktx_header.pixel_width, ktx_header.pixel_height,
			&info, pixel_data );
		*img_width = ktx_header.pixel_width;
		*img_height = ktx_header.pixel_height;
		free( pixel_data );
//	}
	if( KTX_UNPACK_ALIGNMENT != unpack_alignment ) {
		glPixelStorei( GL_UNPACK_ALIGNMENT, unpack_alignment );
	}
	fclose( fh );
	return 0;
}
