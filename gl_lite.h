/*
	From:
	http://apoorvaj.io/loading-opengl-without-glew.html
	https://github.com/ApoorvaJ/Papaya/blob/3808e39b0f45d4ca4972621c847586e4060c042a/src/libs/gl_lite.h
*/

#ifndef CTOOL_GL_LITE
#define CTOOL_GL_LITE

#if defined(__linux__)
#include <dlfcn.h>
#define GLDECL
#define HC_GL_LIST_WIN32
#endif /* __linux__ */

#if defined(_WIN32)
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define GLDECL WINAPI

#define GL_ARRAY_BUFFER				(0x8892)

typedef char GLchar;
typedef ptrdiff_t GLintptr;
typedef ptrdiff_t GLsizeiptr;

#define HC_GL_LIST_WIN32 \
	GLE( void,		ActiveTexture,	GLenum ) \
	GLE( void,		BlendEquation,	GLenum )

#endif /* _WIN32 */

#include <GL/gl.h>

#define HC_GL_LIST \
	GLE( void,	AttachShader,		GLuint, GLuint ) \
	GLE( void,	BindBuffer,			GLenum, GLuint ) \
	GLE( void,	BindBufferBase,		GLenum, GLuint, GLuint ) \
	GLE( void,	BindVertexArray,	GLuint ) \
	GLE( void,	BufferData,			GLenum, GLsizeiptr, const GLvoid *, GLenum ) \
	GLE( void,	BufferSubData,		GLenum, GLintptr, GLsizeiptr, const GLvoid * ) \
	GLE( void,	CompileShader,		GLuint ) \
	GLE( GLuint,	CreateProgram,		void ) \
	GLE( GLuint,	CreateShader,		GLenum ) \
	GLE( void,	DeleteBuffers,		GLsizei, GLuint * ) \
	GLE( void,	DeleteProgram,		GLuint ) \
	GLE( void,	DeleteVertexArrays,	GLsizei, GLuint * ) \
	GLE( void,	DetachShader,		GLuint, GLuint ) \
	GLE( void,	EnableVertexAttribArray,	GLuint ) \
	GLE( void,	GenBuffers,			GLsizei, GLuint * ) \
	GLE( void,	GenVertexArrays,	GLsizei, GLuint * ) \
	GLE( void,	GenerateMipmap,		GLenum ) \
	GLE( void,	GetProgramiv,		GLuint, GLenum, GLint * ) \
	GLE( void,	GetProgramInfoLog,	GLuint, GLsizei, GLsizei *, GLchar * ) \
	GLE( void,	GetShaderInfoLog,	GLuint, GLsizei, GLsizei *, GLchar * ) \
	GLE( void,	GetShaderiv,		GLuint, GLenum, GLint * ) \
	GLE( GLint,	GetUniformLocation,	GLuint, const GLchar * ) \
	GLE( void,	LinkProgram,		GLuint ) \
	GLE( void,	ShaderSource,		GLuint, GLsizei, const GLchar **, const GLint * ) \
	GLE( void,	Uniform1i,			GLint, GLint ) \
	GLE( void,	Uniform2i,			GLint, GLint, GLint ) \
	GLE( void,	Uniform1ui,			GLint, GLuint ) \
	GLE( void,	Uniform2ui,			GLint, GLuint, GLuint ) \
	GLE( void,	Uniform1f,			GLint, GLfloat ) \
	GLE( void,	Uniform2f,			GLint, GLfloat, GLfloat ) \
	GLE( void,	Uniform3f,			GLint, GLfloat, GLfloat, GLfloat ) \
	GLE( void,	Uniform3fv,			GLint, GLsizei, const GLfloat * ) \
	GLE( void,	Uniform4fv,			GLint, GLsizei, const GLfloat * ) \
	GLE( void,	UniformMatrix4fv,	GLint , GLsizei, GLboolean, const GLfloat * ) \
	GLE( void,	UseProgram,			GLuint ) \
	GLE( void,	ValidateProgram,	GLuint ) \
	GLE( void,	VertexAttribPointer,	GLuint, GLint, GLenum, GLboolean, GLsizei, const GLvoid * )


#define GLE( ret, name, ... ) \
	typedef ret GLDECL name##proc( __VA_ARGS__ ); \
	extern name##proc * gl##name;

HC_GL_LIST
HC_GL_LIST_WIN32
#undef GLE

int gl_lite_init( );

#endif /* CTOOL_GL_LITE */

#ifdef GL_LITE_IMPL

#include <stdio.h>

#define GLE( ret, name, ... ) name##proc * gl##name;
HC_GL_LIST
HC_GL_LIST_WIN32
#undef GLE

int gl_lite_init( ) {
#if defined(__linux__)

	void *libGL = dlopen( "libGL.so", RTLD_LAZY );
	if( !libGL ) {
		fprintf( stderr, "Error: libGL.so could not be loaded!\n" );
		return -1;
	}

	#define GLE( ret, name, ... ) \
		gl##name = ( name##proc * ) dlsym( libGL, "gl" #name ); \
		if( !gl##name ) { \
			fprintf( stderr, "Function gl" #name " could not be loaded from libGL.so!\n" ); \
			return -1; \
		}

		HC_GL_LIST
	#undef GLE

#elif defined(_WIN32)

	HINSTANCE dll = LoadLibraryA( "opengl32.dll" );
	typedef PROC WINAPI wglGetProcAddressproc( LPCSTR lpszProc );

	if( !dll ) {
		OutputDebugStringA( "Error: opengl32.dll not found!\n" );
		return -1;
	}

	wglProcAddressproc *wglGetProcAddress =
		( wglGetProcAddressproc * ) GetProcAddress( dll, "wglGetProcAddress" );

	#define GLE( ret, name, ... ) \
		gl##name = ( name##proc * ) wglGetProcAddress( "gl" #name ); \
		if( !gl##name ) { \
			OutputDebugStringA( "Function gl" #name " could not be loaded from opengl32.dll!\n" ); \
			return -1; \
		}

		HC_GL_LIST
		HC_GL_LIST_WIN32
	#undef GLE

#else
	#error "GL loading for this platform is not implemented!"
#endif

	return 1;
}

#endif /* GL_LITE_IMPL */
