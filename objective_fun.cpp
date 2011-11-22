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

// mex -g -I"/Users/alex/Source/boost_1_42_0/" objective_fun.cpp
// mex -I"/Users/alex/Source/boost_1_42_0/" objective_fun.cpp

#include "mex.h"
#include <boost/la/all.hpp>
#include <vector>

using namespace boost::la;
typedef vec<double,2> vec2;
typedef vec<double,3> vec3;
typedef mat<double,3,3> mat3;

template <typename T>
vec<T,2> Vec(T x, T y) { vec<T,2> v = {x, y}; return v; }

struct CameraParams {
    double f;
    double td;
    double ts;
    vec3 d;
};

// start is used to begin search from where we last finished.
vec3 interp(double* tx, vec3* x, double ty, int num, int& start)
{
    int i = start;
    mxAssert(0 <= i && i < num-1, "i is outside array range");
    
    while (i >= 0 && tx[i] > ty) { --i; }
    if (i < 0) return x[0];
    
    while (i+1 < num && tx[i+1] < ty) { ++i; }
    if (i+1 >= num) return x[num-1];
    
    start = i;
    
    double dt = tx[i+1] - tx[i];
    double dy = ty - tx[i];
    double w = dy / dt;
    
    mxAssert(dt >= 0, "time is not monotonically increasing");
    mxAssert(0 <= dy && dy <= dt, "");
    
    return x[i] * (1-w) + x[i+1] * w;
}

mat3 getW(const vec3& theta, double w, double h, double f)
{
    mat3 K = {1, 0, -w/2,
              0, 1, -h/2,
              0, 0, f};
              
    mat3 invK = {1, 0, w/(2*f),
                 0, 1, h/(2*f),
                 0, 0, 1/f};
    
    return invK * rotz_matrix<3>(-theta|Z) * roty_matrix<3>(theta|X) * rotx_matrix<3>(theta|Y) * K;
}

#define PT_FRAME_ID prhs[0]
#define PT_X        prhs[1]
#define PT_Y        prhs[2]
#define FRAME_TIME  prhs[3]
#define FRAME_SIZE  prhs[4]
#define GYRO_THETA  prhs[5]
#define GYRO_TIME   prhs[6]
#define X0          prhs[7]

#define ERR         plhs[0]

typedef int int32;

// f = objective_fun(frame_idx, x, y, frame_times, frame_size, theta, theta_times, x0)
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if ( nlhs != 1 || nrhs != 8 ||
        !mxIsInt32(PT_FRAME_ID) ||
        !mxIsDouble(PT_X) ||
        !mxIsDouble(PT_Y) ||
        !mxIsDouble(FRAME_TIME) ||
        !mxIsDouble(FRAME_SIZE) ||
        !mxIsDouble(GYRO_THETA) ||
        !mxIsDouble(GYRO_TIME) ||
        !mxIsDouble(X0) )
    {
        mexErrMsgTxt("Incorrect input/output argument format. Usage: f = objective_fun(frame_idx, x, y, frame_times, frame_size, theta, theta_times, x0)");
    }
    
    int32* frame_idx = (int32*)mxGetData(PT_FRAME_ID);
    vec2* x = (vec2*)mxGetData(PT_X);
    vec2* y = (vec2*)mxGetData(PT_Y);
    double* frame_time = mxGetPr(FRAME_TIME);
    vec3* theta = (vec3*)mxGetData(GYRO_THETA);
    double* gyro_time = mxGetPr(GYRO_TIME);
    CameraParams& p = *((CameraParams*)mxGetData(X0));
    const double w = mxGetPr(FRAME_SIZE)[0];
    const double h = mxGetPr(FRAME_SIZE)[1];

    ERR = mxCreateDoubleMatrix(1, 1, mxREAL);
    double &err = *mxGetPr(ERR);

    int num_pts = mxGetNumberOfElements(PT_FRAME_ID);
    int num_gyro_samp = mxGetNumberOfElements(GYRO_TIME);
    
    //mexPrintf("camera params: f (%lf), td (%lf), ts (%lf)\n", p.f, p.td, p.ts);
    
    int start1 = 0;
    int start2 = 0;
    
    err = 0;
    for (int i=0; i < num_pts; ++i)
    {
        const int f = frame_idx[i];

        
        const double w1 = ((x[i]|Y) - h/2) / h;
        const double t1 = frame_time[f] + p.td + p.ts * w1;
        vec3 th1 = interp(gyro_time, theta, t1, num_gyro_samp, start1);
        
        const double w2 = ((y[i]|Y) - h/2) / h;
        const double t2 = frame_time[f+1] + p.td + p.ts * w2;
        vec3 th2 = interp(gyro_time, theta, t2, num_gyro_samp, start2);
        double dt = t2 - t1;
        vec3 dth = th2 - th1 + p.d * dt;
        
        vec3 pX = getW(dth, w, h, p.f) * (x[i]|XY1);
        
        vec2 dxy = y[i] - ( Vec(pX|X, pX|Y) / (pX|Z) );
        err += sqrt(dot(dxy, dxy));
    }
    
    err /= num_pts;
}