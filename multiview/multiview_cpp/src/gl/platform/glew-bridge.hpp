
#pragma once

#ifndef USE_QT_GL_FUNCTIONS

#include <GL/glew.h>
#include <GL/gl.h>

#else

#include <GL/gl.h>

namespace perceive
{
// See http://apoorvaj.io/loading-opengl-without-glew.html

GLuint glCreateProgram();
void glDeleteProgram(GLuint program);

GLuint glCreateShader(GLenum type);
void glDeleteShader(GLuint shader);
void glDetachShader(GLuint program, GLuint shader);
void glCompileShader(GLuint shader);
void glGetShaderiv(GLuint shader, GLenum pname, GLint* params);
void glGetShaderInfoLog(GLuint shader, GLsizei bufsize, GLsizei* length,
                        char* infolog);
void glShaderSource(GLuint shader, GLsizei count, const char** string,
                    const GLint* length);

} // namespace perceive

#endif

// struct GlFunctions
// {

// // GLES2 + OpenGL1 common subset
// void glBindTexture(GLenum target, GLuint texture);
// void glBlendFunc(GLenum sfactor, GLenum dfactor);
// void glClear(GLbitfield mask);
// void glClearColor(GLclampf red, GLclampf green,
//                   GLclampf blue, GLclampf alpha);
// void glClearStencil(GLint s);
// void glColorMask(GLboolean red, GLboolean green,
//                  GLboolean blue, GLboolean alpha);
// void glCopyTexImage2D(GLenum target, GLint level,
//                       GLenum internalformat, GLint x,
//                       GLint y, GLsizei width,
//                       GLsizei height, GLint border);
// void glCopyTexSubImage2D(GLenum target, GLint level,
//                          GLint xoffset, GLint yoffset,
//                          GLint x, GLint y, GLsizei width,
//                          GLsizei height);
// void glCullFace(GLenum mode);
// void glDeleteTextures(GLsizei n, const GLuint* textures);
// void glDepthFunc(GLenum func);
// void glDepthMask(GLboolean flag);
// void glDisable(GLenum cap);
// void glDrawArrays(GLenum mode, GLint first, GLsizei count);
// void glDrawElements(GLenum mode, GLsizei count, GLenum type,
//                     const GLvoid* indices);
// void glEnable(GLenum cap);
// void glFinish();
// void glFlush();
// void glFrontFace(GLenum mode);
// void glGenTextures(GLsizei n, GLuint* textures);
// void glGetBooleanv(GLenum pname, GLboolean* params);
// GLenum glGetError();
// void glGetFloatv(GLenum pname, GLfloat* params);
// void glGetIntegerv(GLenum pname, GLint* params);
// const GLubyte *glGetString(GLenum name);
// void glGetTexParameterfv(GLenum target, GLenum pname, GLfloat* params);
// void glGetTexParameteriv(GLenum target, GLenum pname, GLint* params);
// void glHint(GLenum target, GLenum mode);
// GLboolean glIsEnabled(GLenum cap);
// GLboolean glIsTexture(GLuint texture);
// void glLineWidth(GLfloat width);
// void glPixelStorei(GLenum pname, GLint param);
// void glPolygonOffset(GLfloat factor, GLfloat units);
// void glReadPixels(GLint x, GLint y, GLsizei width, GLsizei height,
//                   GLenum format, GLenum type, GLvoid* pixels);
// void glScissor(GLint x, GLint y, GLsizei width, GLsizei height);
// void glStencilFunc(GLenum func, GLint ref, GLuint mask);
// void glStencilMask(GLuint mask);
// void glStencilOp(GLenum fail, GLenum zfail, GLenum zpass);
// void glTexImage2D(GLenum target, GLint level,
//                   GLint internalformat, GLsizei width,
//                   GLsizei height, GLint border,
//                   GLenum format, GLenum type,
//                   const GLvoid* pixels);
// void glTexParameterf(GLenum target, GLenum pname, GLfloat param);
// void glTexParameterfv(GLenum target, GLenum pname,const GLfloat* params);
// void glTexParameteri(GLenum target, GLenum pname, GLint param);
// void glTexParameteriv(GLenum target, GLenum pname, const GLint* params);
// void glTexSubImage2D(GLenum target, GLint level, GLint xoffset,
//                      GLint yoffset, GLsizei width, GLsizei height,
//                      GLenum format, GLenum type,
//                      const GLvoid* pixels);
// void glViewport(GLint x, GLint y, GLsizei width, GLsizei height);

// // GL(ES)2
// void glActiveTexture(GLenum texture);
// void glAttachShader(GLuint program, GLuint shader);
// void glBindAttribLocation(GLuint program, GLuint index,const char* name);
// void glBindBuffer(GLenum target, GLuint buffer);
// void glBindFramebuffer(GLenum target, GLuint framebuffer);
// void glBindRenderbuffer(GLenum target, GLuint renderbuffer);
// void glBlendColor(GLclampf red, GLclampf green, GLclampf blue,
//                   GLclampf alpha);
// void glBlendEquation(GLenum mode);
// void glBlendEquationSeparate(GLenum modeRGB, GLenum modeAlpha);
// void glBlendFuncSeparate(GLenum srcRGB, GLenum dstRGB,
//                          GLenum srcAlpha, GLenum dstAlpha);
// void glBufferData(GLenum target, qopengl_GLsizeiptr size,
//                   const void* data, GLenum usage);
// void glBufferSubData(GLenum target, qopengl_GLintptr offset,
//                      qopengl_GLsizeiptr size, const void* data);
// GLenum glCheckFramebufferStatus(GLenum target);
// void glClearDepthf(GLclampf depth);
// void glCompressedTexImage2D(GLenum target, GLint level,
//                             GLenum internalformat,
//                             GLsizei width, GLsizei height,
//                             GLint border, GLsizei imageSize,
//                             const void* data);
// void glCompressedTexSubImage2D(GLenum target, GLint level, G
//                                Lint xoffset, GLint yoffset,
//                                GLsizei width, GLsizei height,
//                                GLenum format,
//                                GLsizei imageSize, const void* data);
// void glDeleteBuffers(GLsizei n, const GLuint* buffers);
// void glDeleteFramebuffers(GLsizei n, const GLuint* framebuffers);
// void glDeleteRenderbuffers(GLsizei n, const GLuint* renderbuffers);
// void glDepthRangef(GLclampf zNear, GLclampf zFar);

// void glDisableVertexAttribArray(GLuint index);
// void glEnableVertexAttribArray(GLuint index);
// void glFramebufferRenderbuffer(GLenum target, GLenum attachment,
//                                GLenum renderbuffertarget,
//                                GLuint renderbuffer);
// void glFramebufferTexture2D(GLenum target, GLenum attachment,
//                             GLenum textarget, GLuint texture,
//                             GLint level);
// void glGenBuffers(GLsizei n, GLuint* buffers);
// void glGenerateMipmap(GLenum target);
// void glGenFramebuffers(GLsizei n, GLuint* framebuffers);
// void glGenRenderbuffers(GLsizei n, GLuint* renderbuffers);
// void glGetActiveAttrib(GLuint program, GLuint index,
//                        GLsizei bufsize, GLsizei* length,
//                        GLint* size, GLenum* type, char* name);
// void glGetActiveUniform(GLuint program, GLuint index,
//                         GLsizei bufsize, GLsizei* length,
//                         GLint* size, GLenum* type, char* name);
// void glGetAttachedShaders(GLuint program, GLsizei maxcount,
//                           GLsizei* count, GLuint* shaders);
// GLint glGetAttribLocation(GLuint program, const char* name);
// void glGetBufferParameteriv(GLenum target, GLenum pname, GLint* params);
// void glGetFramebufferAttachmentParameteriv(GLenum target,
//                                            GLenum attachment,
//                                            GLenum pname, GLint* params);
// void glGetProgramiv(GLuint program, GLenum pname, GLint* params);
// void glGetProgramInfoLog(GLuint program, GLsizei bufsize,
//                          GLsizei* length, char* infolog);
// void glGetRenderbufferParameteriv(GLenum target, GLenum pname,
//                                   GLint* params);

// void glGetShaderPrecisionFormat(GLenum shadertype,
//                                 GLenum precisiontype, GLint* range,
//                                 GLint* precision);
// void glGetUniformfv(GLuint program, GLint location, GLfloat* params);
// void glGetUniformiv(GLuint program, GLint location, GLint* params);
// GLint glGetUniformLocation(GLuint program, const char* name);
// void glGetVertexAttribfv(GLuint index, GLenum pname, GLfloat* params);
// void glGetVertexAttribiv(GLuint index, GLenum pname, GLint* params);
// void glGetVertexAttribPointerv(GLuint index, GLenum pname,
//                                void** pointer);
// GLboolean glIsBuffer(GLuint buffer);
// GLboolean glIsFramebuffer(GLuint framebuffer);
// GLboolean glIsProgram(GLuint program);
// GLboolean glIsRenderbuffer(GLuint renderbuffer);
// GLboolean glIsShader(GLuint shader);
// void glLinkProgram(GLuint program);
// void glReleaseShaderCompiler();
// void glRenderbufferStorage(GLenum target, GLenum internalformat,
//                            GLsizei width, GLsizei height);
// void glSampleCoverage(GLclampf value, GLboolean invert);
// void glShaderBinary(GLint n, const GLuint* shaders,
//                     GLenum binaryformat, const void* binary,
//                     GLint length);
// void glStencilFuncSeparate(GLenum face, GLenum func,
//                            GLint ref, GLuint mask);
// void glStencilMaskSeparate(GLenum face, GLuint mask);
// void glStencilOpSeparate(GLenum face, GLenum fail,
//                          GLenum zfail, GLenum zpass);
// void glUniform1f(GLint location, GLfloat x);
// void glUniform1fv(GLint location, GLsizei count, const GLfloat* v);
// void glUniform1i(GLint location, GLint x);
// void glUniform1iv(GLint location, GLsizei count, const GLint* v);
// void glUniform2f(GLint location, GLfloat x, GLfloat y);
// void glUniform2fv(GLint location, GLsizei count, const GLfloat* v);
// void glUniform2i(GLint location, GLint x, GLint y);
// void glUniform2iv(GLint location, GLsizei count, const GLint* v);
// void glUniform3f(GLint location, GLfloat x, GLfloat y, GLfloat z);
// void glUniform3fv(GLint location, GLsizei count, const GLfloat* v);
// void glUniform3i(GLint location, GLint x, GLint y, GLint z);
// void glUniform3iv(GLint location, GLsizei count, const GLint* v);
// void glUniform4f(GLint location, GLfloat x, GLfloat y, GLfloat z,
//                  GLfloat w);
// void glUniform4fv(GLint location, GLsizei count, const GLfloat* v);
// void glUniform4i(GLint location, GLint x, GLint y, GLint z, GLint w);
// void glUniform4iv(GLint location, GLsizei count, const GLint* v);
// void glUniformMatrix2fv(GLint location, GLsizei count,
//                         GLboolean transpose, const GLfloat* value);
// void glUniformMatrix3fv(GLint location, GLsizei count,
//                         GLboolean transpose, const GLfloat* value);
// void glUniformMatrix4fv(GLint location, GLsizei count,
//                         GLboolean transpose, const GLfloat* value);
// void glUseProgram(GLuint program);
// void glValidateProgram(GLuint program);
// void glVertexAttrib1f(GLuint indx, GLfloat x);
// void glVertexAttrib1fv(GLuint indx, const GLfloat* values);
// void glVertexAttrib2f(GLuint indx, GLfloat x, GLfloat y);
// void glVertexAttrib2fv(GLuint indx, const GLfloat* values);
// void glVertexAttrib3f(GLuint indx, GLfloat x, GLfloat y, GLfloat z);
// void glVertexAttrib3fv(GLuint indx, const GLfloat* values);
// void glVertexAttrib4f(GLuint indx, GLfloat x, GLfloat y,
//                       GLfloat z, GLfloat w);
// void glVertexAttrib4fv(GLuint indx, const GLfloat* values);
// void glVertexAttribPointer(GLuint indx, GLint size, GLenum type,
//                            GLboolean normalized, GLsizei stride,
//                            const void* ptr);

// };
