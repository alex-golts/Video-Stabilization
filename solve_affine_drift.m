% Digital Video Stabilization and Rolling Shutter Correction using Gyroscopes
% Copyright (C) 2011 Alexandre Karpenko
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

function [c t0 ts d err] = solve_affine_drift(gyro, frame_time, W, video_file)

gyro(:,1) = -gyro(:,1);

dft = diff(frame_time);
dT = diff(gyro(:,4));
theta = ((gyro(1:end-1,1:3) + gyro(2:end,1:3)) / 2) .* dT(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];

c = -1572;
t0 = -0.1564;
ts = 0;

alpha_c = 1e-3;
alpha_t0 = 1e-12;
alpha_ts = 1e-12;
alpha_d = 1e-10;
d = -mean(gyro(:,1:3),1);
%d = [0 0 0];        % gyro drift

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

frame1 = read(xyloObj, 200);
frame2 = read(xyloObj, 201);

T1 = translation_matrix(vid_width/2, vid_height/2);
T2 = translation_matrix(-vid_width/2, -vid_height/2);

for iter = 1:5000
    tt0 = lininterp(gyro(:,4) + t0, theta, frame_time);
    dth = diff(tt0) + d(ones(num_frames-1,1),:) .* dft(:,[1 1 1]);
    dths = diff(lininterp(gyro(:,4) + t0 + ts, theta, frame_time) - tt0);
    
    ww0 = lininterp(gyro(:,4) + t0, gyro(:,1:3), frame_time);
    dw = diff(ww0);
    
    wws = lininterp(gyro(:,4) + t0 + ts, gyro(:,1:3), frame_time);
    dwt0 = diff(wws - ww0);
    dwts = diff(lininterp(gyro(:,4) + t0 + ts, gyro(:,1:2), frame_time));
    
    dE_dc = 0;
    dE_dt0 = 0;
    dE_dts = 0;
    dE_d0 = 0;
    dE_d1 = 0;
    dE_d2 = 0;
    
    err = 0;
    denom = 0;
    
    for f=1:num_frames-1
        x = [W(f).x2 ones(size(W(f).x1,1),1)]';
        y = [W(f).x1 ones(size(W(f).x1,1),1)]';
        
        %S = eye(3);
        S = [1 c*dths(f,1)/720 0; 0 1+c*dths(f,2)/720 0; 0 0 1];
        R = T1 * [cos(dth(f,3)) -sin(dth(f,3)) 0; sin(dth(f,3)) cos(dth(f,3)) 0; 0 0 1] * T2;
        T = [1 0 c*dth(f,1); 0 1 c*dth(f,2); 0 0 1]; 
        M = T*R*S;
        %{
        if f == 200 && mod(iter-1,20) == 0
            figure(1); clf;
            h1 = imshow(frame2); hold on;
            h2 = imshow(imtransform(frame1, maketform('affine', M'), 'XData', [1 vid_width], 'YData', [1 vid_height]));
            set(h2, 'AlphaData', 0.6);
            hold off;
            pause(0.01);
        end
        %}
        yMx = (y - M*x);
        err = err + sum(sqrt(sum(yMx .* yMx, 1)));
        denom = denom + size(yMx,2);
        
        dS_dc = [0 dths(f,1)/720 0; 0 dths(f,2)/720 0; 0 0 0];
        dT_dc = [0 0 dth(f,1); 0 0 dth(f,2); 0 0 0];
        dM_dc = (dT_dc * R * S) + (T * R * dS_dc);
        dE_dc = dE_dc + sum(sum(yMx .* (-dM_dc * x)));
        
        dS_dt0 = [0 c*dwt0(f,1)/720 0; 0 c*dwt0(f,2)/720 0; 0 0 0];
        dR_dt0 = T1 * ([-sin(dth(f,3)) -cos(dth(f,3)) 0; cos(dth(f,3)) -sin(dth(f,3)) 0; 0 0 0] * dw(f,3)) * T2;
        dT_dt0 = [0 0 c*dw(f,1); 0 0 c*dw(f,2); 0 0 0];
        dM_dt0 = (dT_dt0 * R * S) + (T * dR_dt0 * S) + (T * R * dS_dt0);
        dE_dt0 = dE_dt0 + sum(sum(yMx .* (-dM_dt0 * x)));
        
        dT_d0 = [0 0 c*dft(f); 0 0 0; 0 0 0];
        dM_d0 = dT_d0 * R * S;
        dE_d0 = dE_d0 + sum(sum(yMx .* (-dM_d0 * x)));
        
        dT_d1 = [0 0 0; 0 0 c*dft(f); 0 0 0];
        dM_d1 = dT_d1 * R * S;
        dE_d1 = dE_d1 + sum(sum(yMx .* (-dM_d1 * x)));
        
        dR_d2 = T1 * ([-sin(dth(f,3)) -cos(dth(f,3)) 0; cos(dth(f,3)) -sin(dth(f,3)) 0; 0 0 0] * dft(f)) * T2;
        dM_d2 = T * dR_d2 * S;
        dE_d2 = dE_d2 + sum(sum(yMx .* (-dM_d2 * x)));
        
        dS_dts = [0 c*dwts(f,1)/720 0; 0 c*dwts(f,2)/720 0; 0 0 0];
        dM_dts = T * R * dS_dts;
        dE_dts = dE_dts + sum(sum(yMx .* (-dM_dts * x)));
    end
    
    err = err / denom;
    
    c = c - alpha_c * dE_dc;
    t0 = t0 + alpha_t0 * dE_dt0;
    ts = ts + alpha_ts * dE_dts;
    
    d(1) = d(1) - alpha_d * dE_d0;
    d(2) = d(2) - alpha_d * dE_d1;
    d(3) = d(3) - alpha_d * dE_d2;
    
    fprintf('c: %f, t0: %f, ts: %f, d: (%f, %f, %f), error: %f\n', c, t0, ts, d(1), d(2), d(3), err);
end