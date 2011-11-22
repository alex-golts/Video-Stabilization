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

function nw = resample_rotation_rate(w, old_time, new_time)

dT = old_time(2:end) - old_time(1:end-1);
theta = cumsum( w(2:end) .* dT );

new_theta = interp1(old_time(2:end), theta(:,1), new_time);

%{
figure(1);
plot(old_time(2:end), theta, 'rx-'); hold on;
plot(new_time, new_theta, 'bo-'); hold off;
%}

nw = (new_theta(2:end) - new_theta(1:end-1)) ./ (new_time(2:end) - new_time(1:end-1));
nw(isnan(nw)) = 0;
nw = [0; nw];

%{
figure(2);
plot(old_time, w(:,1), 'rx-'); hold on;
plot(new_time, nw, 'bo-'); hold off;
%}