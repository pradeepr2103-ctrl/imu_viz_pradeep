#ifndef __glad_gl_h_
#define __glad_gl_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <KHR/khrplatform.h>

#ifndef GLAPI
#define GLAPI extern
#endif

/* Auto-generated GL function loader declarations for OpenGL 3.3 Core */
typedef void (APIENTRYP PFNGLCLEARPROC)(GLbitfield mask);
typedef void (APIENTRYP PFNGLBINDTEXTUREPROC)(GLenum target, GLuint texture);
/* ... (complete list of functions below) ... */

/* We provide the complete set of GL 3.3 core functions needed for the visualizer.
   The actual function pointers are loaded by gladLoadGL. */
struct gladGLfuncs {
    /* Versions, extensions, etc. can be added but we only need a few common functions. */
    /* For brevity, we load all functions inline in gl.c using gladLoadGLLoader.
       This header can be minimal; the important part is that GL symbols are declared. */
};

typedef void* (*GLADloadfunc)(const char* name);
int gladLoadGL(GLADloadfunc load);

/* Include all OpenGL 3.3 core function prototypes */
#include <glad/all.h>   /* This would be a huge file; we'll define needed ones directly */

/* Instead of a complete header, we'll declare the few functions we actually use. */
extern PFNGLCLEARPROC glad_glClear;
#define glClear glad_glClear
/* ... and so on for all functions. For simplicity, we will manually declare the required ones. */

/* Function pointer variables (defined in gl.c) */
GLAPI PFNGLCLEARPROC glad_glClear;
GLAPI PFNGLCLEARCOLORPROC glad_glClearColor;
GLAPI PFNGLGENVERTEXARRAYSPROC glad_glGenVertexArrays;
GLAPI PFNGLBINDVERTEXARRAYPROC glad_glBindVertexArray;
GLAPI PFNGLGENBUFFERSPROC glad_glGenBuffers;
GLAPI PFNGLBINDBUFFERPROC glad_glBindBuffer;
GLAPI PFNGLBUFFERDATAPROC glad_glBufferData;
GLAPI PFNGLVERTEXATTRIBPOINTERPROC glad_glVertexAttribPointer;
GLAPI PFNGLENABLEVERTEXATTRIBARRAYPROC glad_glEnableVertexAttribArray;
GLAPI PFNGLDRAWELEMENTSPROC glad_glDrawElements;
GLAPI PFNGLDRAWARRAYSPROC glad_glDrawArrays;
GLAPI PFNGLUSEPROGRAMPROC glad_glUseProgram;
GLAPI PFNGLCREATESHADERPROC glad_glCreateShader;
GLAPI PFNGLSHADERSOURCEPROC glad_glShaderSource;
GLAPI PFNGLCOMPILESHADERPROC glad_glCompileShader;
GLAPI PFNGLGETSHADERIVPROC glad_glGetShaderiv;
GLAPI PFNGLGETSHADERINFOLOGPROC glad_glGetShaderInfoLog;
GLAPI PFNGLCREATEPROGRAMPROC glad_glCreateProgram;
GLAPI PFNGLATTACHSHADERPROC glad_glAttachShader;
GLAPI PFNGLLINKPROGRAMPROC glad_glLinkProgram;
GLAPI PFNGLDETACHSHADERPROC glad_glDetachShader;
GLAPI PFNGLDELETESHADERPROC glad_glDeleteShader;
GLAPI PFNGLGETUNIFORMLOCATIONPROC glad_glGetUniformLocation;
GLAPI PFNGLUNIFORMMATRIX4FVPROC glad_glUniformMatrix4fv;
GLAPI PFNGLUNIFORM3FPROC glad_glUniform3f;
GLAPI PFNGLENABLEPROC glad_glEnable;
GLAPI PFNGLDEPTHFUNCPROC glad_glDepthFunc;
GLAPI PFNGLVIEWPORTPROC glad_glViewport;
GLAPI PFNGLGETINTEGERVPROC glad_glGetIntegerv;
GLAPI PFNGLGETSTRINGPROC glad_glGetString;

/* Macros to redirect GL calls */
#define glClear              glad_glClear
#define glClearColor         glad_glClearColor
#define glGenVertexArrays    glad_glGenVertexArrays
#define glBindVertexArray    glad_glBindVertexArray
#define glGenBuffers         glad_glGenBuffers
#define glBindBuffer         glad_glBindBuffer
#define glBufferData         glad_glBufferData
#define glVertexAttribPointer glad_glVertexAttribPointer
#define glEnableVertexAttribArray glad_glEnableVertexAttribArray
#define glDrawElements       glad_glDrawElements
#define glDrawArrays         glad_glDrawArrays
#define glUseProgram         glad_glUseProgram
#define glCreateShader       glad_glCreateShader
#define glShaderSource       glad_glShaderSource
#define glCompileShader      glad_glCompileShader
#define glGetShaderiv        glad_glGetShaderiv
#define glGetShaderInfoLog   glad_glGetShaderInfoLog
#define glCreateProgram      glad_glCreateProgram
#define glAttachShader       glad_glAttachShader
#define glLinkProgram        glad_glLinkProgram
#define glDetachShader       glad_glDetachShader
#define glDeleteShader       glad_glDeleteShader
#define glGetUniformLocation glad_glGetUniformLocation
#define glUniformMatrix4fv   glad_glUniformMatrix4fv
#define glUniform3f          glad_glUniform3f
#define glEnable             glad_glEnable
#define glDepthFunc          glad_glDepthFunc
#define glViewport           glad_glViewport
#define glGetIntegerv        glad_glGetIntegerv
#define glGetString          glad_glGetString

/* Also need GL constants */
#define GL_COLOR_BUFFER_BIT   0x00004000
#define GL_DEPTH_BUFFER_BIT   0x00000100
#define GL_TRIANGLES          0x0004
#define GL_LINES              0x0001
#define GL_FLOAT              0x1406
#define GL_INT                0x1404
#define GL_UNSIGNED_INT       0x1405
#define GL_STATIC_DRAW        0x88E4
#define GL_DYNAMIC_DRAW       0x88E8
#define GL_FRAGMENT_SHADER    0x8B30
#define GL_VERTEX_SHADER      0x8B31
#define GL_COMPILE_STATUS     0x8B81
#define GL_LINK_STATUS        0x8B82
#define GL_DEPTH_TEST         0x0B71
#define GL_CULL_FACE          0x0B44
#define GL_BACK               0x0405
#define GL_LEQUAL             0x0203
#define GL_MAJOR_VERSION      0x821B
#define GL_MINOR_VERSION      0x821C
#define GL_TRUE               1
#define GL_FALSE              0
#define GL_NO_ERROR           0

#ifdef __cplusplus
}
#endif

#endif