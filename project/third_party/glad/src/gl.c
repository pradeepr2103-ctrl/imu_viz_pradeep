    #include <stdlib.h>
#include <string.h>
#include "glad/gl.h"

/* Declare function pointer storage */
PFNGLCLEARPROC glad_glClear;
PFNGLCLEARCOLORPROC glad_glClearColor;
PFNGLGENVERTEXARRAYSPROC glad_glGenVertexArrays;
PFNGLBINDVERTEXARRAYPROC glad_glBindVertexArray;
PFNGLGENBUFFERSPROC glad_glGenBuffers;
PFNGLBINDBUFFERPROC glad_glBindBuffer;
PFNGLBUFFERDATAPROC glad_glBufferData;
PFNGLVERTEXATTRIBPOINTERPROC glad_glVertexAttribPointer;
PFNGLENABLEVERTEXATTRIBARRAYPROC glad_glEnableVertexAttribArray;
PFNGLDRAWELEMENTSPROC glad_glDrawElements;
PFNGLDRAWARRAYSPROC glad_glDrawArrays;
PFNGLUSEPROGRAMPROC glad_glUseProgram;
PFNGLCREATESHADERPROC glad_glCreateShader;
PFNGLSHADERSOURCEPROC glad_glShaderSource;
PFNGLCOMPILESHADERPROC glad_glCompileShader;
PFNGLGETSHADERIVPROC glad_glGetShaderiv;
PFNGLGETSHADERINFOLOGPROC glad_glGetShaderInfoLog;
PFNGLCREATEPROGRAMPROC glad_glCreateProgram;
PFNGLATTACHSHADERPROC glad_glAttachShader;
PFNGLLINKPROGRAMPROC glad_glLinkProgram;
PFNGLDETACHSHADERPROC glad_glDetachShader;
PFNGLDELETESHADERPROC glad_glDeleteShader;
PFNGLGETUNIFORMLOCATIONPROC glad_glGetUniformLocation;
PFNGLUNIFORMMATRIX4FVPROC glad_glUniformMatrix4fv;
PFNGLUNIFORM3FPROC glad_glUniform3f;
PFNGLENABLEPROC glad_glEnable;
PFNGLDEPTHFUNCPROC glad_glDepthFunc;
PFNGLVIEWPORTPROC glad_glViewport;
PFNGLGETINTEGERVPROC glad_glGetIntegerv;
PFNGLGETSTRINGPROC glad_glGetString;

/* The loader function: loads all function pointers via the provided callback */
int gladLoadGL(GLADloadfunc load) {
    if (!load) return 0;

    glad_glClear = (PFNGLCLEARPROC)load("glClear");
    glad_glClearColor = (PFNGLCLEARCOLORPROC)load("glClearColor");
    glad_glGenVertexArrays = (PFNGLGENVERTEXARRAYSPROC)load("glGenVertexArrays");
    glad_glBindVertexArray = (PFNGLBINDVERTEXARRAYPROC)load("glBindVertexArray");
    glad_glGenBuffers = (PFNGLGENBUFFERSPROC)load("glGenBuffers");
    glad_glBindBuffer = (PFNGLBINDBUFFERPROC)load("glBindBuffer");
    glad_glBufferData = (PFNGLBUFFERDATAPROC)load("glBufferData");
    glad_glVertexAttribPointer = (PFNGLVERTEXATTRIBPOINTERPROC)load("glVertexAttribPointer");
    glad_glEnableVertexAttribArray = (PFNGLENABLEVERTEXATTRIBARRAYPROC)load("glEnableVertexAttribArray");
    glad_glDrawElements = (PFNGLDRAWELEMENTSPROC)load("glDrawElements");
    glad_glDrawArrays = (PFNGLDRAWARRAYSPROC)load("glDrawArrays");
    glad_glUseProgram = (PFNGLUSEPROGRAMPROC)load("glUseProgram");
    glad_glCreateShader = (PFNGLCREATESHADERPROC)load("glCreateShader");
    glad_glShaderSource = (PFNGLSHADERSOURCEPROC)load("glShaderSource");
    glad_glCompileShader = (PFNGLCOMPILESHADERPROC)load("glCompileShader");
    glad_glGetShaderiv = (PFNGLGETSHADERIVPROC)load("glGetShaderiv");
    glad_glGetShaderInfoLog = (PFNGLGETSHADERINFOLOGPROC)load("glGetShaderInfoLog");
    glad_glCreateProgram = (PFNGLCREATEPROGRAMPROC)load("glCreateProgram");
    glad_glAttachShader = (PFNGLATTACHSHADERPROC)load("glAttachShader");
    glad_glLinkProgram = (PFNGLLINKPROGRAMPROC)load("glLinkProgram");
    glad_glDetachShader = (PFNGLDETACHSHADERPROC)load("glDetachShader");
    glad_glDeleteShader = (PFNGLDELETESHADERPROC)load("glDeleteShader");
    glad_glGetUniformLocation = (PFNGLGETUNIFORMLOCATIONPROC)load("glGetUniformLocation");
    glad_glUniformMatrix4fv = (PFNGLUNIFORMMATRIX4FVPROC)load("glUniformMatrix4fv");
    glad_glUniform3f = (PFNGLUNIFORM3FPROC)load("glUniform3f");
    glad_glEnable = (PFNGLENABLEPROC)load("glEnable");
    glad_glDepthFunc = (PFNGLDEPTHFUNCPROC)load("glDepthFunc");
    glad_glViewport = (PFNGLVIEWPORTPROC)load("glViewport");
    glad_glGetIntegerv = (PFNGLGETINTEGERVPROC)load("glGetIntegerv");
    glad_glGetString = (PFNGLGETSTRINGPROC)load("glGetString");

    /* Check that we got the essential functions */
    if (!glad_glClear || !glad_glClearColor || !glad_glGenVertexArrays)
        return 0;
    return 1;
}