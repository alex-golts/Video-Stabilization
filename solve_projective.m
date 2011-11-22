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

function [ff td ts wd err] = solve_projective(gyro, frame_time, W, video_file)

gyro(:,[2 1]) = gyro(:,1:2);    % x & y are swapped
gyro(:,3) = -gyro(:,3);         % rotations about z
dft = diff(frame_time);
dT = diff(gyro(:,4));
theta = ((gyro(1:end-1,1:3) + gyro(2:end,1:3)) / 2) .* dT(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];

ff = -1572;
td = -0.1564;
ts = 0;

alpha_f = 1e-12;
alpha_td = 1e-12;
alpha_ts = 1e-12;
alpha_wd = 1e-10;
wd = -mean(gyro(:,1:3),1);
%d = [0 0 0];        % gyro drift

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
num_frames = xyloObj.NumberOfFrames;
vh = xyloObj.Height;
vw = xyloObj.Width;

frame1 = read(xyloObj, 200);
frame2 = read(xyloObj, 201);

for iter = 1:5000
    for f=100:num_frames-1
        dE_df = 0;
        dE_dtd = 0;
        dE_dts = 0;

        err = 0;
        denom = 0;
    
        x = W(f).x1;
        y = W(f).x2;
        for i = 1:size(x,1)
            K = [1 0 -vw/2; 0 1 -vh/2; 0 0 ff];
            
            X = K*[x(i,:)'; 1];
            Y = K*[y(i,:)'; 1];
            
            tx = frame_time(f) - td - ts * X(2) / vh;
            ty = frame_time(f+1) - td - ts * Y(2) / vh;
            
            w = interp1(gyro(:,4), gyro(:,1:3), [tx ty], 'linear', 'extrap');
            dw = diff(w);
            dws = (w(2,:) * Y(2) - w(1,:) * X(2)) / vh;
            dth = diff(interp1(gyro(:,4), theta(:,1:3), [tx ty], 'linear', 'extrap'));
            
            Rx = [1            0           0;
                  0 cos(dth(1)) -sin(dth(1));
                  0 sin(dth(1))  cos(dth(1))];
            
            Ry = [ cos(dth(2)) 0 sin(dth(2));
                   0           1           0;
                  -sin(dth(2)) 0 cos(dth(2))];
            
            Rz = [cos(dth(3)) -sin(dth(3)) 0;
                  sin(dth(3))  cos(dth(3)) 0;
                  0           0            1];

              
            dRx =[0             0           0;
                  0 -sin(dth(1)) -cos(dth(1));
                  0 cos(dth(1))  -sin(dth(1))];
            
            dRy =[-sin(dth(2)) 0  cos(dth(2));
                   0           0            0;
                  -cos(dth(2)) 0 -sin(dth(2))];
            
            dRz =[-sin(dth(3)) -cos(dth(3)) 0;
                   cos(dth(3)) -sin(dth(3)) 0;
                   0           0            0];
            
            R = Rz * Ry * Rx;
            RY = R * Y;
            
            XXY_XYX = X'*X*RY' - X'*RY*X';
            dE_dts = dE_dts + 2 * (XXY_XYX) * (Rz * Ry * dRx * dws(1) + Rz * dRy * Rx * dws(2) + dRz * Ry * Rx * dws(3)) * Y;
            dE_dtd = dE_dtd + 2 * (XXY_XYX) * (Rz * Ry * dRx * dw(1) + Rz * dRy * Rx * dw(2) + dRz * Ry * Rx * dw(3)) * Y;
            dE_df = dE_df + 2*(RY'*RY)*X(3)*ff + 2*(X'*X)*(RY'*(R*[0;0;f])) - 2*(X'*RY)*(ff*RY(3) + X'*(R*[0;0;f]));
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
            err = err + (X'*X)*(Y'*Y) - (X'*RY)^2;
            denom = denom + 1;
        end
        
        err = err / denom;
    
        ff = ff + alpha_f * dE_df;
        %td = td - alpha_td * dE_dtd;
        %ts = ts - alpha_ts * dE_dts;

        fprintf('f: %f, td: %f, ts: %f, d: (%f, %f, %f), error: %f\n', ff, td, ts, 0, 0, 0, err);
    end
end