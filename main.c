/* gcc -Wall -O2 -o test main.c -lm -ldl -lX11 -lXi -lXrandr -lGL */

#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/extensions/Xrandr.h>
#include <GL/glx.h>
#include "types.h"
#include "gl_lite.c"
#include "3d.c"
#include "shading.c"
#include "assets.c"

//------------------------------------------------------------------------------

enum { /*! Handle standard mouse with 3 buttons. */
	MOUSE_LEFT = 1,
	MOUSE_WHEEL,
	MOUSE_RIGHT,
	MOUSE_DOWN,
	MOUSE_UP
};

enum {
	SHAPE_PLANE,
	SHAPE_MAX
};

enum {
	TEXTURE_BLUEPRINT,
	TEXTURE_MAX
};

//------------------------------------------------------------------------------

static int camera_changed;
static int cull = 0;
static int cur_angle = 0;
static int width = 800;
static int height = 450;
static float mx = 0.0f;
static float my = 0.0f;

static const float MOVEMENT_SPEED = 0.01f;
static const float STRIDE_SPEED = 0.01f;
static const float ROTATION_SPEED = 0.17f; // 10 degrees
static const double dt_f = 0.01f;
static const Vector_3d X_AXIS = { 1.0f, 0.0f, 0.0f };
static const Vector_3d Y_AXIS = { 0.0f, 1.0f, 0.0f };
static const Vector_3d Z_AXIS = { 0.0f, 0.0f, 1.0f };

static double timer_start;
static double timer_end;
static double delta_t = 0.0f;
static float speeds[ 6 ];
static float cam_angles[ 3 ];

static Camera camera;
static Vao vaos[ SHAPE_MAX ];
static Texture textures[ TEXTURE_MAX ];
static Vector_3d plane_position;
static Quaternion plane_rotation;
static Perspective perspective;
static Matrix_4x4 view_matrix;
static Matrix_4x4 projection_matrix;
static Directional_Light sun;

//------------------------------------------------------------------------------

inline
double milliseconds( void ) {
	struct timespec time;
	clock_gettime( CLOCK_MONOTONIC, &time );
	return ( double ) time.tv_sec * 1000.0f
		+ ( double ) time.tv_nsec / 1000000.0f;
}

#define TTY_RED(x)		"\x1B[" #x ";31m"
#define TTY_RST			"\x1B[0m"

void gl_error( const char *file, int line ) {
	GLenum err = glGetError( );
	const char* msg;

	while( err != GL_NO_ERROR ) {
		switch( err ) {
			case GL_INVALID_OPERATION: msg = "GL_INVALID_OPERATION"; break;
			case GL_INVALID_ENUM: msg = "GL_INVALID_ENUM"; break;
			case GL_INVALID_VALUE: msg = "GL_INVALID_VALUE"; break;
			case GL_OUT_OF_MEMORY: msg = "GL_OUT_OF_MEMORY"; break;
			case GL_INVALID_FRAMEBUFFER_OPERATION:
				msg = "GL_INVALID_FRAMEBUFFER_OPERATION"; break;
			case GL_STACK_OVERFLOW: msg = "GL_STACK_OVERFLOW"; break;
			case GL_STACK_UNDERFLOW: msg = "GL_STACK_UNDERFLOW"; break;
			default: msg = "UNKNOWN ERROR";
		}
		fprintf( stderr,
			TTY_RED( 1 ) "%s, %s, %d\n" TTY_RST, msg, file, line );
		err = glGetError( );
	}
}

#define DEBUG_GL gl_error( __FILE__, __LINE__ )

void toggle_culling( void ) {
	cull = !cull;

	if( cull ) {
		glEnable( GL_CULL_FACE );
		glCullFace( GL_BACK );
	} else {
		glDisable( GL_CULL_FACE );
	}
}

void print_mat( const char *msg, const Matrix_4x4 *m, int prec ) {
	int i;
	printf( "%s\n", msg );
	for( i = 0; i < 16; i+= 4 ) {
		printf( "[ %.*f, %.*f, %.*f, %.*f ]\n",
			prec, m->array[ i ],
			prec, m->array[ i + 1 ],
			prec, m->array[ i + 2 ],
			prec, m->array[ i + 3 ] );
	}
	printf( "\n" );
}

void opengl_setup( void ) {
	glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
	glEnable( GL_DEPTH_TEST );
	glDepthFunc( GL_LEQUAL );
//	glDepthFunc( GL_LESS );
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
	glEnable( GL_TEXTURE_2D );
	toggle_culling( );
}

void setup_perspective( float width, float height ) {
	view_matrix = projection_matrix = identity_matrix_4x4;

	perspective.fov = radians( 75.0f );
	perspective.near = 1.0f;
	perspective.far = 20.0f;
	perspective.view_aspect = width / height;
	perspective.view_oo_aspect = 1.0f / perspective.view_aspect;
	perspective.ortho_scale_x = 2.0f / width;
	perspective.ortho_scale_y = -2.0f / height;
	matrix_4x4_perspective_projection( &perspective, &projection_matrix );

	Vector_3d rot = { 0.0f, 0.0f, 0.0f };
	camera.position.x = 0.0f;
	camera.position.y = 0.0f;
	camera.position.z = 6.0f;
	quaternion_from_euler_v( &camera.rotation, rot );
	quaternion_normalize( &camera.rotation );

	camera_changed = 1;

	print_mat( "projection matrix", &projection_matrix, 2 );
}

void setup_light( void ) {
	Vector_3d u = { radians( 180.0f ), 0.0f, 0.0f };
	quaternion_from_euler_v( &sun.rotation, u );
	quaternion_normalize( &sun.rotation );
	sun.position = ( Vector_3d ) { .x = 0.0f, .y = 10.0f, .z = 0.0f };
	sun.intensities = ( Color ) { 255.0f, 255.0f, 255.0f, 127.0f };
	sun.ambient_coefficient = 0.008f; // 80 %
}

int upd_cur_angle( int dir ) {
	int a = cur_angle + dir;

	if( a < 0 ) {
		a = 2;
	} else if( a > 2 ) {
		a = 0;
	}
	return a;
}

void translate_camera( double dt ) {
	Quaternion conj;
	Vector_3d v;
	speeds[ 0 ] = approach( speeds[ 1 ], speeds[ 0 ], dt * dt_f );
	speeds[ 2 ] = approach( speeds[ 3 ], speeds[ 2 ], dt * dt_f );
	v.x = speeds[ 2 ] * dt, v.y = speeds[ 0 ] * dt, v.z = 0.0f;
	quaternion_conjugate( &camera.rotation, &conj );
	quaternion_mul_vector( &conj, v, &v );
	vector_3d_add( camera.position, v, &camera.position );
}

void rotate_camera( double dt ) {
	Quaternion q, r;
	speeds[ 4 ] = dt * approach( speeds[ 5 ], speeds[ 4 ], dt * dt_f );
	quaternion_from_axis_angle( &r, Y_AXIS, -mx * speeds[ 4 ] );
	quaternion_from_axis_angle( &q, X_AXIS, -my * speeds[ 4 ] );
	quaternion_mul_quaternion( &q, &r, &q );
	quaternion_mul_quaternion( &q, &camera.rotation, &camera.rotation );
	quaternion_normalize( &camera.rotation );

	if( speeds[ 4 ] > speeds[ 5 ] ) {
		speeds[ 5 ] = 0.0f;
	}
	mx = my = 0;
}

void update_camera( double dt ) {
	if( camera_changed ) {
		rotate_camera( dt );
		translate_camera( dt );
		quaternion_to_matrix( &camera.rotation, &view_matrix );
		matrix_4x4_set_neg_translation_v( &view_matrix, camera.position );
		camera_changed = 0;
		print_mat( "view matrix", &view_matrix, 2 );
	}
}

void render( double dt ) {
	glViewport( 0, 0, width, height );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	Vao *obj = &vaos[ SHAPE_PLANE ];
	glActiveTexture( GL_TEXTURE0 );
	glBindVertexArray( obj->vao );
	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, obj->ind );

	Shader *p = &programs[ PROGRAM_MODEL ];
	const s16 *uni_loc = p->uniform_locations;
	glUseProgram( p->program_id );
	glUniform1i( uni_loc[ ULOC_TEXTURE0 ], 0 );

	glUniformMatrix4fv( uni_loc[ ULOC_VIEW ], 1, GL_FALSE,
		( GLfloat* ) &view_matrix );
	glUniformMatrix4fv( uni_loc[ ULOC_PROJECTION ], 1, GL_FALSE,
		( GLfloat* ) &projection_matrix );
	glUniform3fv( uni_loc[ ULOC_LOCATION ], 1, ( GLfloat * ) &sun.position );
	glUniform4fv( uni_loc[ ULOC_INTENSITIES ], 1,
		( GLfloat * ) &sun.intensities );
	glUniform1f( uni_loc[ ULOC_AMBIENT_COEFF ], sun.ambient_coefficient );

	Matrix_4x4 rotation;
	quaternion_to_matrix( &plane_rotation, &rotation );
	matrix_4x4_set_translation_v( &rotation, plane_position );

//	print_mat( "model matrix", &rotation, 2 );

	glBindTexture( GL_TEXTURE_2D, textures[ TEXTURE_BLUEPRINT ].tex_id );
	glUniform2f( uni_loc[ ULOC_ATLAS ], 1.0f, 0.0f );
	glUniformMatrix4fv( uni_loc[ ULOC_MODEL ],	1, GL_TRUE,
		( GLfloat* ) &rotation );
	glDrawElements( GL_TRIANGLES, obj->len, GL_UNSIGNED_SHORT, 0 );

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
	glBindVertexArray( 0 );
	glUseProgram( 0 );
	DEBUG_GL;
}

int load_assets( void ) {
	int skinned = 0;
	u16 v_size = 0, i_size = 0;
	float *v_data = 0;
	u16 *i_data = 0;
	char buffer[ 256 ];
	snprintf( buffer, 256, "%s", "assets/plane.mob" );
	read_mob( buffer, &v_size, &v_data, &i_size, &i_data, &skinned );

	if( v_data && i_data ) {
		mk_indexed_model( &vaos[ SHAPE_PLANE ],
			v_size, v_data,	sizeof( u16 ), i_size, i_data,
			GL_STATIC_DRAW, skinned );

		free( v_data );
		free( i_data );

		Vector_3d u = { 0.0f, 0.0f, 0.0f };
		quaternion_from_euler_v( &plane_rotation, u );
		plane_position = ( Vector_3d ) { 0.0f, 0.0f, -1.0f };
	} else {
		return -1;
	}

	Texture *tex = &textures[ TEXTURE_BLUEPRINT ];
	memset( &buffer, 0, sizeof( char ) * 256 );
	snprintf( buffer,256, "%s", "assets/blueprint.ktx" );
	u32 id = 0;
	GLint min_filter = GL_LINEAR_MIPMAP_LINEAR;
	GLint mag_filter = GL_LINEAR;
	GLint wrap_s = GL_REPEAT;
	GLint wrap_t = GL_REPEAT;
	int img_width, img_height;

	if( load_ktx( buffer, &id, min_filter, mag_filter, wrap_s, wrap_t,
		&img_width, &img_height, KTX_UNPACK_ALIGNMENT ) < 0 )
	{
		return -1;
	}
	tex->tex_id = id;
	tex->width = img_width;
	tex->height = img_height;
	tex->scale_x = 1;
	tex->scale_y = 1;

	return 0;
}

//------------------------------------------------------------------------------

int main( int argc, char** argv ) {
	Display *xlib_display = 0;
	Window xlib_window;
	XVisualInfo *xlib_visual_info;
	Atom wm_delete;
	char kbd_buffer[ 16 ];
	xlib_display = XOpenDisplay( 0 );

	if( !xlib_display ) {
		fprintf( stderr, "Error: could not connect to X server!\n" );
		return -1;
	}
	s32 glx_attr[ ] = {
		GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None
	};
	xlib_visual_info = glXChooseVisual( xlib_display, 0, glx_attr );
	if( !xlib_visual_info ) {
		fprintf( stderr, "Error: no appropriate visuals found!\n" );
		return -1;
	}
	XSetWindowAttributes window_attr = { 0 };
	window_attr.colormap = XCreateColormap( xlib_display,
		DefaultRootWindow( xlib_display ),
		xlib_visual_info->visual, AllocNone );
	window_attr.event_mask =
		ExposureMask
		| PointerMotionMask
		| ButtonPressMask
		| ButtonReleaseMask
		| KeyPressMask
		| KeyReleaseMask
		| VisibilityChangeMask
		| PropertyChangeMask
		| ResizeRedirectMask;

	xlib_window = XCreateWindow( xlib_display,
		DefaultRootWindow( xlib_display ),
		0, 0, width, height, 0,
		xlib_visual_info->depth, InputOutput, xlib_visual_info->visual,
		CWColormap | CWEventMask, &window_attr );
	XMapWindow( xlib_display, xlib_window );
	wm_delete = XInternAtom( xlib_display, "WM_DELETE_WINDOW", 0 );

	XSetWMProtocols( xlib_display, xlib_window, &wm_delete, 1 );
	XSizeHints *hints = XAllocSizeHints( );
	hints->flags = PMinSize;
	hints->min_width = width;
	hints->min_height = height;
	XSetWMNormalHints( xlib_display, xlib_window, hints );
	XFree( hints );

	GLXContext context = glXCreateContext(
		xlib_display, xlib_visual_info, 0, GL_TRUE );
	glXMakeCurrent( xlib_display, xlib_window, context );

	if( gl_lite_init( ) < 0 ) {
		return -1;
	}
	if( glXIsDirect( xlib_display, context ) ) {
		printf( "Direct GLX rendering context obtained.\n" );
	} else {
		printf( "Indirect GLX rendering context obtained.\n" );
	}
	opengl_setup( );
	init_shaders( );
	setup_perspective( ( float ) width, ( float ) height );
	setup_light( );

	if( 0 != load_assets( ) ) {
		return -1;
	}

	int run = 1;
	timer_start = milliseconds( );

	while( run ) {
		while( XPending( xlib_display ) ) {
			KeySym key_sym;
			XEvent event;
			XNextEvent( xlib_display, &event );

			switch( event.type ) {
				case Expose: {
					XWindowAttributes window_attr;
					XGetWindowAttributes( xlib_display, xlib_window, &window_attr );
				} break;
				case ButtonPress: {
					if( MOUSE_UP == event.xbutton.button ) {
						plane_position.z += 0.1;
					} else if( MOUSE_DOWN == event.xbutton.button ) {
						plane_position.z -= 0.1;
					}
				} break;
				case KeyRelease: {
					if( ( XK_W == key_sym ) || ( XK_S == key_sym ) ) {
						speeds[ 1 ] = 0.0f;
						camera_changed = 1;
					} else if( ( XK_A == key_sym ) || ( XK_D == key_sym ) ) {
						speeds[ 3 ] = 0.0f;
						camera_changed = 1;
					}
				} break;
				case KeyPress: {
					XLookupString( &event.xkey, kbd_buffer, 16, &key_sym, 0 );
					if( XK_space <= key_sym && XK_asciitilde >= key_sym ) {
						if( XK_W == key_sym ) {
							speeds[ 1 ] = -MOVEMENT_SPEED;
							camera_changed = 1;
						} else if( XK_A == key_sym ) {
							speeds[ 3 ] = -STRIDE_SPEED;
							camera_changed = 1;
						} else if( XK_S == key_sym ) {
							speeds[ 1 ] = MOVEMENT_SPEED;
							camera_changed = 1;
						} else if( XK_D == key_sym ) {
							speeds[ 3 ] = STRIDE_SPEED;
							camera_changed = 1;
						}
					} else if( XK_Escape == key_sym ) {
						run = 0;
					} else if( ( XK_minus == key_sym )
						|| ( XK_KP_Subtract == key_sym ) )
					{
						speeds[ 5 ] = ROTATION_SPEED;

						if( ShiftMask & event.xkey.state ) {
							cur_angle = upd_cur_angle( -1 );
						} else {
							cam_angles[ cur_angle ] -= radians( 1.0f );
							Vector_3d v = {
								cam_angles[ 0 ], cam_angles[ 1 ], cam_angles[ 2 ] };
							quaternion_from_euler_v( &camera.rotation, v );
							quaternion_normalize( &camera.rotation );

							camera_changed = 1;
						}
					} else if( ( XK_plus == key_sym )
						|| ( XK_KP_Add == key_sym ) )
					{
						speeds[ 5 ] = ROTATION_SPEED;

						if( ShiftMask & event.xkey.state ) {
							cur_angle = upd_cur_angle( 1 );
						} else {
							cam_angles[ cur_angle ] += radians( 1.0f );
							Vector_3d v = {
								cam_angles[ 0 ], cam_angles[ 1 ], cam_angles[ 2 ] };
							quaternion_from_euler_v( &camera.rotation, v );
							quaternion_normalize( &camera.rotation );

							camera_changed = 1;
						}
					} else if( XK_F12 == key_sym ) {
						toggle_culling( );
					}
				} break;
			}
		}
		if( run ) {
			update_camera( delta_t );
			render( delta_t );
			glXSwapBuffers( xlib_display, xlib_window );
			timer_end = milliseconds( );
			delta_t = timer_end - timer_start;
			timer_start = timer_end;
		}
	}
	XFree( xlib_visual_info );
	XDestroyWindow( xlib_display, xlib_window );
	XCloseDisplay( xlib_display );

	return 0;
}
