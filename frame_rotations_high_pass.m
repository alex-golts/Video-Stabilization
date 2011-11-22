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

function [dth, dths] = frame_rotations_high_pass(gyro, gyro_time, frame_time, t0, ts)
% computes the delta in theta between frame times, as well as the shear
% at frame time
dgt = diff(gyro_time);
theta = ((gyro(1:end-1,:) + gyro(2:end,:)) / 2) .* dgt(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];
dths = lininterp(gyro_time + t0 + ts, theta, frame_time) - lininterp(gyro_time + t0, theta, frame_time);

sigma2 = 4000;
gauss = exp(-(-120:120).^2 / sigma2);
gauss = gauss ./ sum(gauss);
gyro(:,1) = gyro(:,1) - conv(gyro(:,1), gauss, 'same');
gyro(:,2) = gyro(:,2) - conv(gyro(:,2), gauss, 'same');
gyro(:,3) = gyro(:,3) - conv(gyro(:,3), gauss, 'same');
theta = ((gyro(1:end-1,:) + gyro(2:end,:)) / 2) .* dgt(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];
%dth = diff(lininterp(gyro_time + t0, theta, frame_time));
%dth = diff(lininterp(gyro_time + t0 + ts/2, theta, frame_time));
%dth = diff((lininterp(gyro_time + t0, theta, frame_time) + lininterp(gyro_time + t0 + ts, theta, frame_time)) ./ 2);
dth = diff( ...
    ((lininterp(gyro_time + t0, theta, frame_time) + lininterp(gyro_time + t0 + ts, theta, frame_time)) ./ 2 + ...
     lininterp(gyro_time + t0 + ts/2, theta, frame_time)) ./2 ...
    );
