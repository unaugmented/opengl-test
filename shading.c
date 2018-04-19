#include <stdio.h>
#include "types.h"
#include "shading.h"

static Shader programs[ PROGRAM_MAX ];

static const char *uniforms[ ] = {
	"ambient_coeff",
	"atlas",
	"color_bg",
	"color_fg",
	"flags",
	"intensities",
	"location",
	"model",
	"projection",
	"radius",
	"scale",
	"texture0",
	"texture1",
	"texture2",
	"translation",
	"view"
};

const char model_vertex_shader[ ] =
"#version 130\n"
"in vec3 vertex;"
"in vec3 normal;"
"in vec2 uv;"
"out vec2 coords;"
"out vec3 to_camera;"
"out vec3 to_light;"
"out vec3 n_surface;"
"uniform vec2 atlas;"
"uniform mat4 model;"
"uniform mat4 view;"
"uniform mat4 projection;"
"uniform vec3 location;"
"uniform vec4 intensities;"
"void main( void ) {"
"vec4 world_pos = model * vec4( vertex, 1.0 );"
"gl_Position = projection * view * world_pos;"
"to_light = location.xyz - world_pos.xyz;"
"to_camera = ( inverse( view ) * vec4( 0, 0, 0, 1 ) ).xyz -world_pos.xyz;"
"n_surface = ( model * vec4( normal, 0 ) ).xyz;"
"coords = uv;"
//"coords = uv / atlas.x + atlas.y;"
"}"
;

const char model_fragment_shader[ ] =
"#version 130\n"
"in vec2 coords;"
"in vec3 to_camera;"
"in vec3 to_light;"
"in vec3 n_surface;"
"out vec4 pixel_color;"
"uniform sampler2D texture0;"
"uniform vec4 intensities;"
"uniform float ambient_coeff;"
"void main( void ) {"
"vec3 unit_sn = normalize( n_surface );"
"vec3 unit_lv = normalize( to_light );"
"vec3 unit_cv = normalize( to_camera );"
"float dist = length( to_light );"
"float af = 1.0 / ( 1.0 + intensities.a * pow( dist, 2.0 ) );"
"float dp = dot( unit_sn, unit_lv );"
"float bright = max( 0.0, dp );"
"vec4 sample = texture( texture0, coords );"
"vec3 color = sample.rgb * intensities.rgb;"
"vec3 diffuse = bright * color;"
"float damp = 0.0;"
"if( dp > 0.0 ) {"
"vec3 refl = reflect( -unit_lv, unit_sn );"
"float spec = max( 0.0, dot( refl, unit_cv ) );"
"int shini = 1;"
"damp = pow( spec, shini );"
"}"
"vec3 specular = damp * color;"
"vec3 ambient = ambient_coeff * color;"
"vec3 tmp = ambient + af * ( diffuse + specular );"
//"vec4 tmp2 = mix( vec4( intensities.rgb, 1.0 ), vec4( tmp, 1.0 ), 0.5 );"
"pixel_color = vec4( tmp, sample.a );"
//"pixel_color = sample;"
//"if( pixel_color.a < 0.5 ) {"
//"discard;"
//"}"
"}"
;

//------------------------------------------------------------------------------

inline
int shader_compile( u32 source, const GLchar *data ) {
	int result = 0;
	GLsizei len;
	s32 status;
	glShaderSource( source, 1, ( const GLchar** ) &data, 0 );
	glCompileShader( source );
	glGetShaderiv( source, GL_COMPILE_STATUS, &status );

	if( !status ) {
		GLchar log[ 256 ];
		glGetShaderInfoLog( source, 256, &len, log );
		fprintf( stderr, "glCompileShader(): %s\n", log );
		result = -1;
	}
	return result;
}

static
int init_shader( Shader *p, const GLchar *v_data,
	const GLchar *f_data, const GLchar *g_data )
{
	int result = 0;
	int attach_geometry = ( 0 != g_data );
	GLsizei len;
	s32 status;
	u32 program_id;
	u32 sources[ 3 ];
	sources[ 0 ] = glCreateShader( GL_VERTEX_SHADER );
	sources[ 2 ] = glCreateShader( GL_FRAGMENT_SHADER );

	if( 0 != ( result = shader_compile( sources[ 0 ], v_data ) ) ) {
		goto last;
	}
	if( 0 != ( result = shader_compile( sources[ 2 ], f_data ) ) ) {
		goto last;
	}
	if( attach_geometry ) {
		sources[ 1 ] = glCreateShader( GL_GEOMETRY_SHADER );

		if( 0 != ( result = shader_compile( sources[ 1 ], g_data ) ) ) {
			goto last;
		}
	}
	program_id = glCreateProgram( );

	if( 0 == program_id ) {
		fprintf( stderr, "Error: glCreateProgram() for program failed!\n" );
		result = -1;
		goto last;
	}
	glAttachShader( program_id, sources[ 0 ] );
	glAttachShader( program_id, sources[ 2 ] );

	if( attach_geometry ) {
		glAttachShader( program_id, sources[ 1 ] );
	}
	glLinkProgram( program_id );
	glGetProgramiv( program_id, GL_LINK_STATUS, &status );

	if( !status ) {
		GLchar log[ 256 ];
		glGetProgramInfoLog( program_id, 256, &len, log );
		fprintf( stderr, "glLinkProgram(): %s\n", log );
		result = -1;
		goto last;
	}
	glDetachShader( program_id, sources[ 0 ] );
	glDetachShader( program_id, sources[ 2 ] );

	if( attach_geometry ) {
		glDetachShader( program_id, sources[ 1 ] );
	}
	glValidateProgram( program_id );
	glGetProgramiv( program_id, GL_VALIDATE_STATUS, &status );

	if( !status ) {
		fprintf( stderr, "Error: program validation failed!\n" );
		result = -1;
	}
last:
	if( ( -1 != result ) && ( 0 != program_id ) ) p->program_id = program_id;
	return result;
}

//------------------------------------------------------------------------------

int init_shaders( void ) {
#define DEF_LOC(x) p->uniform_locations[(x)] = \
	(s16) glGetUniformLocation(p->program_id, uniforms[(x)]);

	Shader *p;

	p = &programs[ PROGRAM_MODEL ];

	if( init_shader( p,
		model_vertex_shader, model_fragment_shader, 0 ) < 0 )
	{
		return -1;
	}
	p->num_tex_bindings = 1;
	DEF_LOC( ULOC_MODEL );
	DEF_LOC( ULOC_VIEW );
	DEF_LOC( ULOC_PROJECTION );
	DEF_LOC( ULOC_ATLAS );
	DEF_LOC( ULOC_TEXTURE0 );
	DEF_LOC( ULOC_LOCATION );
	DEF_LOC( ULOC_INTENSITIES );
	DEF_LOC( ULOC_AMBIENT_COEFF );
#undef DEF_LOC
	return 0;
}
