// Digital Video Stabilization and Rolling Shutter Correction using Gyroscopes
// Copyright (C) 2011 Alexandre Karpenko
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// complie on osx with:
// mex -I"/Developer/SDKs/MacOSX10.6.sdk/usr/X11/include/" -I"/Users/alex/Source/boost_1_42_0/" -L"/Developer/SDKs/MacOSX10.6.sdk/usr/X11/lib/" -lgl -lglut -lglu meshwarpmex.cpp

#include "mex.h"
#define GL_GLEXT_PROTOTYPES
#include <gl/glut.h>
#include <boost/la/all.hpp>
#include <vector>

using namespace boost::la;
typedef vec<double,2> vec2;
typedef vec<double,3> vec3;
typedef mat<double,3,3> mat33;

static int glutWin = 0;
static GLuint frameBuffer = 0;
static GLuint renderBuffer = 0;
static GLuint texture = 0;
static std::vector<vec3> mesh_verts;
static std::vector<vec2> mesh_uv;
static std::vector<GLushort> mesh_strip;


static void FinishGlut(void)
{
    mexPrintf("Cleaning up GLUT.\n");
    
    glDeleteRenderbuffersEXT(1, &renderBuffer);
    renderBuffer = 0;
    
    glDeleteFramebuffersEXT(1, &frameBuffer);
    frameBuffer = 0;
    
    mesh_verts.clear();
    mesh_uv.clear();
    mesh_strip.clear();
    
    glutDestroyWindow(glutWin);
    glutWin = 0;
}


template <typename T>
vec<T,2> Vec(T x, T y) { vec<T,2> v = {x, y}; return v; };

template <typename T>
vec<T,3> Vec(T x, T y, T z) { vec<T,3> v = {x, y, z}; return v; };

void InitMesh(int width, int height, int num_rows, double focal_len)
{
    double w = width;
    double h = height;
    double dh = h / (num_rows-1);
    int num_cols = (int)round(width / dh) + 1;
    double dw = w / (num_cols-1);
    
    double du = 1.0 / (num_cols-1);
    double dv = 1.0 / (num_rows-1);
    
    mesh_verts.clear();
    mesh_uv.clear();
    mesh_verts.reserve(num_rows * num_cols);
    mesh_uv.reserve(num_rows * num_cols);
    
    double y = -h/2;
    double v = 0;
    for (int r=0; r < num_rows; ++r)
    {
        double x = -w/2;
        double u = 0;
        for (int c=0; c < num_cols; ++c)
        {
            mesh_verts.push_back(Vec(x, y, -focal_len));
            mesh_uv.push_back(Vec(u, v));
            
            x += dw;
            u += du;
        }
        y += dh;
        v += dv;
    }
    
    mesh_strip.clear();
    mesh_verts.reserve(num_rows * num_cols * 2 + num_rows * 2);
    for (int r=1; r < num_rows; ++r)
    {
        mesh_strip.push_back(num_cols*r);
        for (int c=0; c < num_cols; ++c)
        {
            mesh_strip.push_back(num_cols*r + c);
            mesh_strip.push_back(num_cols*(r-1) + c);
        }
        mesh_strip.push_back(num_cols*(r-1) + num_cols - 1);
    }
}

std::vector<vec3> WarpMesh(vec3* theta, int num_rows)
{
    std::vector<vec3> warped;
    warped.reserve(mesh_verts.size());
    int num_cols = mesh_verts.size() / num_rows;
    
    for (int r=0; r < num_rows; ++r)
    {
        mat33 R = rotz_matrix<3>(-*theta|Z) *
                  roty_matrix<3>(*theta|X) *
                  rotx_matrix<3>(*theta|Y);
        for (int c=0; c < num_cols; ++c)
        {
            warped.push_back(R * mesh_verts[num_cols*r + c]);
        }
        ++theta;
    }
    return warped;
}

// glut initialization to get a window, w/o this opengl calls fail
void InitGlut(int width, int height)
{
    int argc = 1;
	char* argv = "meshwarp";
	glutInit(&argc, &argv);	
	glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
	glutInitWindowPosition(100,100);
	glutInitWindowSize(width, height);
	glutWin = glutCreateWindow("Offscreen rendering test");
}

void InitFramebuffer(int width, int height)
{
    glGenFramebuffersEXT(1, &frameBuffer);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, frameBuffer);

    glGenRenderbuffersEXT(1, &renderBuffer);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, renderBuffer);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGBA8, width, height);

    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT,
                                 GL_COLOR_ATTACHMENT0_EXT,
                                 GL_RENDERBUFFER_EXT,
                                 renderBuffer);

    GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
    if (status != GL_FRAMEBUFFER_COMPLETE_EXT) {
        mexErrMsgTxt("Framebuffer error.");
    }
    
    glViewport(0, 0, width, height);
    glClearColor(0,0,0,1);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
}


void CreateTexture(int w, int h, uint8_t* textureData)
{
	int dataSize = w * h * 3;
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, textureData);
}

void ConfigureCamera(int width, int height, double focal_len)
{
    double w = width;
    double h = height;
    double f = focal_len;
    
    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-w/(2*f), w/(2*f), -h/(2*f), h/(2*f), 1, f*4);
}

void RenderScene(int width, int height, double focal_len, vec3& camtheta, std::vector<vec3>& warped)
{
    double w = width;
    double h = height;
    double f = focal_len;
    
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_CULL_FACE);
	
    glMatrixMode(GL_MODELVIEW);
    glRotated((camtheta|Y)*180/M_PI, 1,0,0);
    glRotated((camtheta|X)*180/M_PI, 0,1,0);
    glRotated((camtheta|Z)*180/M_PI, 0,0,-1);
	
	glDisableClientState(GL_COLOR_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_DOUBLE, 0, &warped[0]);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_DOUBLE, 0, &mesh_uv[0]);	
	glBindTexture(GL_TEXTURE_2D, texture);
	
	glDrawElements(GL_TRIANGLE_STRIP, mesh_strip.size(), GL_UNSIGNED_SHORT, &mesh_strip[0]);
}

#define IMAGE     prhs[0]
#define CAMTHETA  prhs[1]
#define ROWTHETA  prhs[2]
#define FOCAL_LEN prhs[3]

#define WARPED    plhs[0]

// warped = meshwarpmex(image, camtheta, rowtheta, focal_len)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nlhs != 1 || nrhs != 4 || !mxIsUint8(IMAGE) || !mxIsDouble(CAMTHETA) || !mxIsDouble(ROWTHETA) || !mxIsDouble(FOCAL_LEN))
    {
        mexErrMsgTxt("Incorrect input argument format. Usage: warped = meshwarp(image, camtheta, rowtheta, focal_len)");
    }
    if (mxGetDimensions(IMAGE)[0] != 3) mexErrMsgTxt("image dims must be [3 w h]");
    
    vec3* camtheta = (vec3*)mxGetData(CAMTHETA);
    vec3* rowtheta = (vec3*)mxGetData(ROWTHETA);
    double focal_len = abs(mxGetScalar(FOCAL_LEN));
    
    int width = mxGetDimensions(IMAGE)[1];
    int height = mxGetDimensions(IMAGE)[2];
    int num_rows = mxGetN(ROWTHETA);
    
    if (!glutWin)
    {
        InitGlut(width, height);        // BUG: width & height could change between calls
        InitFramebuffer(width, height);
        
        mexAtExit(FinishGlut);
    }
    else glutSetWindow(glutWin);
    
    InitMesh(width, height, num_rows, focal_len);
    std::vector<vec3> warped = WarpMesh(rowtheta, num_rows);
    
    CreateTexture(width, height, (uint8_t*)mxGetData(IMAGE));
    
    ConfigureCamera(width, height, focal_len);
    RenderScene(width, height, focal_len, *camtheta, warped);
    
    const mwSize dims[] = {3, width, height};
    WARPED = mxCreateNumericArray(3, dims, mxUINT8_CLASS, mxREAL);
    
    glPixelStorei(GL_PACK_ALIGNMENT, 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, (uint8_t*)mxGetData(WARPED));
    
    glDeleteTextures(1, &texture);
    texture = 0;
}