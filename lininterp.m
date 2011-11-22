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

function y = lininterp(t1, x, t2)

if (t2(1) < t1(1))
    t1 = [t2(1); t1];
    x = [x(1,:); x];
end
if (t2(end) > t1(end))
    t1 = [t1; t2(end)];
    x = [x; x(end,:)];
end

y = interp1(t1, x, t2);