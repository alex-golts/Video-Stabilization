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

function warped = meshwarp(image, dth, theta, gyro_time, frame_time, td, ts, f)

ft = linspace(frame_time + td - ts/2, frame_time + td + ts/2, 20);

th = interp1(gyro_time, theta, ft, 'linear', 'extrap');
thm = interp1(gyro_time - td, theta, frame_time, 'linear', 'extrap');
th = th - thm(ones(size(th,1),1),:);

warped = meshwarpmex(permute(image, [3 2 1]), dth, th', f);
warped = ipermute(warped, [3 2 1]);


%{
	ft = linspace(frame_time - td, frame_time - td - ts, 20);

	th = interp1(gyro_time, theta, ft, 'linear', 'extrap');
	thm = interp1(gyro_time + td + ts/2, theta, frame_time, 'linear', 'extrap');
	th = th - thm(ones(size(th,1),1),:);
%}