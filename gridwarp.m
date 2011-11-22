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

function warped = gridwarp(image, M1, M2, theta, gyro_time, frame_time, ts)

num_pixels = size(image,1)*size(image,2);
w = size(image,2);
h = size(image,1);
[x,y] = meshgrid(1:size(image,2),1:size(image,1));
XI = x;
YI = y;

t = linspace(frame_time - ts/2, frame_time+ts/2, h);

th = lininterp(gyro_time, theta, t);

for i = 1:h
    Rx = [cos(th(i,1)) 0 sin(th(i,1));
          0            1            0;
         -sin(th(i,1)) 0 cos(th(i,1))];
     
    Ry = [1              0             0;
          0 cos(-th(i,2)) -sin(-th(i,2));
          0 sin(-th(i,2)) cos(-th(i,2))];
      
    Rz = [cos(th(i,3)) -sin(th(i,3)) 0;
          sin(th(i,3))  cos(th(i,3)) 0;
          0             0            1];
      
    W = M2*Rz*Ry*Rx*M1;
    X = W * [x(i,:); y(i,:); ones(1,w)];
    x(i,:) = X(1,:);
    y(i,:) = X(2,:);
end

warped = double(image);
F = TriScatteredInterp(x(:), y(:), reshape(warped(:,:,1), num_pixels,1));
z = F(XI(:), YI(:));
warped(:,:,1) = reshape(z, h, w);

F = TriScatteredInterp(x(:), y(:), reshape(warped(:,:,2), num_pixels,1));
z = F(XI(:),YI(:));
warped(:,:,2) = reshape(z, h, w);

F = TriScatteredInterp(x(:), y(:), reshape(warped(:,:,3), num_pixels,1));
z = F(XI(:),YI(:));
warped(:,:,3) = reshape(z, h, w);

warped = uint8(round(warped));