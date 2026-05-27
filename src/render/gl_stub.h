// ==========================================================
// gl_stub.h — Vulkan-only: GL 类型和函数的空桩定义
// 避免未声明的 gl* 调用导致 MSVC 解析器混乱
// ==========================================================
#pragma once

typedef unsigned int GLuint;
typedef int GLint;
typedef float GLfloat;
typedef unsigned int GLenum;
typedef unsigned char GLboolean;
// GLsizei 可能与 Windows SDK 冲突，使用条件定义
#ifndef GLsizei
typedef unsigned int GLsizei;
#endif
typedef unsigned int GLbitfield;

#define GL_TRUE 1
#define GL_FALSE 0
#define GL_NO_ERROR 0
#define GL_INVALID_ENUM 0
#define GL_INVALID_VALUE 0
#define GL_INVALID_OPERATION 0
#define GL_OUT_OF_MEMORY 0
#define GL_FRAMEBUFFER_COMPLETE 0

#define GL_COLOR_BUFFER_BIT 0
#define GL_DEPTH_BUFFER_BIT 0
#define GL_STENCIL_BUFFER_BIT 0

#define GL_POINTS 0
#define GL_LINES 0
#define GL_TRIANGLES 0
#define GL_TRIANGLE_STRIP 0

#define GL_UNSIGNED_INT 0
#define GL_UNSIGNED_BYTE 0
#define GL_FLOAT 0
#define GL_HALF_FLOAT 0
#define GL_RED 0
#define GL_RGBA 0
#define GL_RGB 0
#define GL_RGBA16F 0
#define GL_RGBA32F 0
#define GL_R32F 0
#define GL_DEPTH_COMPONENT 0
#define GL_DEPTH_COMPONENT24 0

#define GL_TEXTURE0 0
#define GL_TEXTURE1 0
#define GL_TEXTURE2 0
#define GL_TEXTURE3 0
#define GL_TEXTURE4 0
#define GL_TEXTURE5 0
#define GL_TEXTURE6 0
#define GL_TEXTURE7 0
#define GL_TEXTURE8 0

#define GL_TEXTURE_2D 0
#define GL_TEXTURE_MIN_FILTER 0
#define GL_TEXTURE_MAG_FILTER 0
#define GL_TEXTURE_WRAP_S 0
#define GL_TEXTURE_WRAP_T 0

#define GL_LINEAR 0
#define GL_NEAREST 0
#define GL_REPEAT 0
#define GL_CLAMP_TO_EDGE 0

#define GL_BLEND 0
#define GL_SRC_ALPHA 0
#define GL_ONE_MINUS_SRC_ALPHA 0
#define GL_ONE 0
#define GL_ZERO 0
#define GL_FUNC_ADD 0
#define GL_MAX 0

#define GL_DEPTH_TEST 0
#define GL_CULL_FACE 0
#define GL_BACK 0
#define GL_FRONT 0
#define GL_CCW 0
#define GL_CW 0
#define GL_LEQUAL 0
#define GL_LESS 0
#define GL_ALWAYS 0

#define GL_ARRAY_BUFFER 0
#define GL_ELEMENT_ARRAY_BUFFER 0
#define GL_STATIC_DRAW 0
#define GL_DYNAMIC_DRAW 0

#define GL_VERTEX_SHADER 0
#define GL_FRAGMENT_SHADER 0
#define GL_COMPILE_STATUS 0
#define GL_LINK_STATUS 0

#define GL_FRAMEBUFFER 0
#define GL_COLOR_ATTACHMENT0 0
#define GL_DEPTH_ATTACHMENT 0

// GL function stubs (no-op)
inline void glClear(GLbitfield) {}
inline void glClearColor(float,float,float,float) {}
inline void glViewport(int,int,int,int) {}
inline void glEnable(GLenum) {}
inline void glDisable(GLenum) {}
inline void glBlendFunc(GLenum,GLenum) {}
inline void glBlendEquation(GLenum) {}
inline void glDepthMask(GLboolean) {}
inline void glDepthFunc(GLenum) {}
inline void glCullFace(GLenum) {}
inline void glFrontFace(GLenum) {}
inline void glBindFramebuffer(GLenum,GLuint) {}
inline void glUseProgram(GLuint) {}
inline void glActiveTexture(GLenum) {}
inline void glBindTexture(GLenum,GLuint) {}
inline void glBindVertexArray(GLuint) {}
inline void glBindBuffer(GLenum,GLuint) {}
inline void glUniform1i(GLint,int) {}
inline void glUniform1f(GLint,float) {}
inline void glUniform2f(GLint,float,float) {}
inline void glUniform3f(GLint,float,float,float) {}
inline void glUniform4f(GLint,float,float,float,float) {}
inline void glUniformMatrix4fv(GLint,int,GLboolean,const float*) {}
inline void glBufferData(GLenum,long long,const void*,GLenum) {}
inline void glBufferSubData(GLenum,long long,long long,const void*) {}
inline void glTexImage2D(GLenum,int,int,int,int,int,GLenum,GLenum,const void*) {}
inline void glTexParameteri(GLenum,GLenum,int) {}
inline void glCopyTexSubImage2D(GLenum,int,int,int,int,int,int,int) {}
inline void glDrawArrays(GLenum,int,GLsizei) {}
inline void glDrawElements(GLenum,GLsizei,GLenum,const void*) {}
inline void glDrawElementsInstanced(GLenum,GLsizei,GLenum,const void*,GLsizei) {}
inline void glGenVertexArrays(GLsizei,GLuint*) {}
inline void glGenBuffers(GLsizei,GLuint*) {}
inline void glGenTextures(GLsizei,GLuint*) {}
inline void glGenFramebuffers(GLsizei,GLuint*) {}
inline void glDeleteVertexArrays(GLsizei,const GLuint*) {}
inline void glDeleteBuffers(GLsizei,const GLuint*) {}
inline void glDeleteTextures(GLsizei,const GLuint*) {}
inline void glDeleteFramebuffers(GLsizei,const GLuint*) {}
inline void glDeleteProgram(GLuint) {}
inline void glDeleteShader(GLuint) {}
inline GLuint glCreateShader(GLenum) { return 0; }
inline GLuint glCreateProgram() { return 0; }
inline void glShaderSource(GLuint,int,const char**,const int*) {}
inline void glCompileShader(GLuint) {}
inline void glAttachShader(GLuint,GLuint) {}
inline void glLinkProgram(GLuint) {}
inline GLint glGetUniformLocation(GLuint,const char*) { return -1; }
inline void glGetShaderiv(GLuint,GLenum,int*) {}
inline void glGetProgramiv(GLuint,GLenum,int*) {}
inline void glGetShaderInfoLog(GLuint,int,int*,char*) {}
inline void glGetProgramInfoLog(GLuint,int,int*,char*) {}
inline void glVertexAttribPointer(GLuint,int,GLenum,GLboolean,int,const void*) {}
inline void glEnableVertexAttribArray(GLuint) {}
inline void glVertexAttribDivisor(GLuint,GLuint) {}
inline GLenum glCheckFramebufferStatus(GLenum) { return GL_FRAMEBUFFER_COMPLETE; }
inline void glFramebufferTexture2D(GLenum,GLenum,GLenum,GLuint,int) {}
