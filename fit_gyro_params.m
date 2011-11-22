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

function [c, t] = fit_gyro_params(xx, yy, sigma)

% move to center along time axis, scale to 1 along y axis

t_offset = mean(yy(:,1));

yy(:,1) = yy(:,1) - t_offset;
xx(:,1) = xx(:,1) - t_offset;

max_y = max(abs(yy(:,2)));
yy(:,2) = yy(:,2) / max_y;
xx(:,2) = xx(:,2) / max_y;


% use only peaks/valleys for matching

dx = xx(2:end, 2) - xx(1:end-1,2);
idx = logical([ 0; ((dx(2:end) > 0) & (dx(1:end-1) < 0) | (dx(2:end) < 0) & (dx(1:end-1) > 0)); 0 ]);
x = xx(idx,:);

dy = yy(2:end, 2) - yy(1:end-1,2);
idx = logical([ 0; ((dy(2:end) > 0) & (dy(1:end-1) < 0) | (dy(2:end) < 0) & (dy(1:end-1) > 0)); 0 ]);
y = yy(idx,:);

c = [1; max(y(:,2))/max(x(:,2))];
t = [0; 0];

nx = size(x,1);
ny = size(y,1);

for k = 1:100

    wx1 = y(:,ones(nx,1)) - (c(1) * x(:,ones(ny,1))' + t(1));
    wx2 = y(:,ones(nx,1)*2) - (c(2) * x(:,ones(ny,1)*2)' + t(2));

    W = exp( ((wx1 .* wx1) + (wx2 .* wx2)) / (-2*sigma*sigma) );
    
    Wj = sum(W,1)';
    Wi = sum(W,2);
    
    %c(1) = ( (W' * (y(:,1) - t(1)))' * x(:,1) ) ./ (Wj' * (x(:,1) .* x(:,1)));
    c(2) = ( (W' * (y(:,2) - t(2)))' * x(:,2) ) ./ (Wj' * (x(:,2) .* x(:,2)));
    
    sW = sum(Wj);
    t(1) = (Wi' * y(:,1) - c(1) * Wj' * x(:,1)) / sW;
    t(2) = (Wi' * y(:,2) - c(2) * Wj' * x(:,2)) / sW;
    
    figure(3);
    plot(xx(:,1)*c(1) + t(1), xx(:,2)*c(2) + t(2), 'r-'); hold on;
    plot(x(:,1)*c(1) + t(1), x(:,2)*c(2) + t(2), 'rx');
    plot(yy(:,1), yy(:,2), 'b-');
    plot(y(:,1), y(:,2), 'bo');
    hold off;
end

sigma = sigma/4;
for k = 1:100

    wx1 = y(:,ones(nx,1)) - (c(1) * x(:,ones(ny,1))' + t(1));
    wx2 = y(:,ones(nx,1)*2) - (c(2) * x(:,ones(ny,1)*2)' + t(2));

    W = exp( ((wx1 .* wx1) + (wx2 .* wx2)) / (-2*sigma*sigma) );
    
    Wj = sum(W,1)';
    Wi = sum(W,2);
    
    %c(1) = ( (W' * (y(:,1) - t(1)))' * x(:,1) ) ./ (Wj' * (x(:,1) .* x(:,1)));
    c(2) = ( (W' * (y(:,2) - t(2)))' * x(:,2) ) ./ (Wj' * (x(:,2) .* x(:,2)));
    
    sW = sum(Wj);
    t(1) = (Wi' * y(:,1) - c(1) * Wj' * x(:,1)) / sW;
    %t(2) = (Wi' * y(:,2) - c(2) * Wj' * x(:,2)) / sW;
    
    figure(3);
    plot(xx(:,1)*c(1) + t(1), xx(:,2)*c(2) + t(2), 'r-'); hold on;
    plot(x(:,1)*c(1) + t(1), x(:,2)*c(2) + t(2), 'rx');
    plot(yy(:,1), yy(:,2), 'b-');
    plot(y(:,1), y(:,2), 'bo');
    hold off;
end

% rescale offset along y back to unscaled input data
t(2) = t(2) * max_y;