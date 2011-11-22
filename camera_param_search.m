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

function [p0 J] = camera_param_search(p0, dp, term_dp, frame_idx, x1, x2, frame_time, vid_dim, theta, gyro_time)

J = objective_fun(frame_idx, x1, x2, frame_time, vid_dim, theta, gyro_time, p0);
    
while (any(abs(dp) > term_dp))
    for d=1:numel(dp)
        p0(d) = p0(d) + dp(d);
        newJ = objective_fun(frame_idx, x1, x2, frame_time, vid_dim, theta, gyro_time, p0);
        if (newJ > J)
            p0(d) = p0(d) - dp(d);
            dp(d) = -dp(d)/3;
        end
        J = newJ;
    end
end

display(J);