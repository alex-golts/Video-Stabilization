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

function theta = integrate_rotation_rates(w, timestamps)

dT = timestamps(2:end) - timestamps(1:end-1);

dtheta = ((w(1:end-1,:) + w(2:end,:)) / 2) .* dT(:,[1 1 1]);

sinz = sin(cumsum(dtheta(:,3)));
cosz = cos(cumsum(dtheta(:,3)));

theta = [0 0 0; dtheta];
theta(2:end,1) = (cosz .* dtheta(:,1)) - (sinz .* dtheta(:,2));
theta(2:end,2) = (sinz .* dtheta(:,1)) + (cosz .* dtheta(:,2));

theta = cumsum(theta, 1);
