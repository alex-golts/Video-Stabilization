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

function [c, d, t] = registersignals(t1, y1, t2, y2)

dy2 = diff(y2) ./ diff(t2);

c = -500;
d = 0;
t = 0;
alpha1 = 1e-11;
alpha2 = 2e-4;
alpha3 = 1e-3;

stop_thresh = 1e-2;

for i=1:1000
    y2i = lininterp(t2 - t, y2, t1);
    dy2i = lininterp(t2(1:end-1) - t, dy2, t1);
    
    if i==1
    figure(500);
    plot(t1 - t1(1), y1, 'rx-'); hold on;
    plot(t1 - t1(1), c*y2i + d, 'bo-'); hold off;
    xlabel('time (s)');
    ylabel('rate of translation (pixels/s)');
    legend('video data', 'gyro data');
    end
    
    s = y1 - c*y2i + d;
    
    dt = -c*sum(s .* dy2i);
    dc = -sum(s .* y2i);
    dd = sum(s);
    
    t = t - alpha1*dt;
    c = c - alpha2*dc;
    d = d - alpha3*dd;
    
    figure(1);
    plot(t1 - t1(1), y1, 'rx-'); hold on;
    plot(t1 - t1(1), c*y2i + d, 'bo-'); hold off;
    pause(0.05);
    
    fprintf('%d, %d, %d\n', alpha1*abs(dt), alpha2*abs(dc), alpha3*abs(dd));
    if alpha1*abs(dt) + alpha2*abs(dc) + alpha2*abs(dd) < stop_thresh
        display(i);
        break;
    end
end

figure(1);
plot(t1 - t1(1), y1, 'rx-'); hold on;
plot(t2-t - t1(1), c*y2, 'bo-'); hold off;
pause(0.05);
xlabel('time (s)');
ylabel('rate of translation (pixels/s)');
legend('video data', 'gyro data');
%{
alpha = 0.005;
y2i = lininterp(t2 - t, y2, t1);
display('part 2');
for i=1:100

    
    s = y1 - c*y2i;
    sy = s .* y2i;
    
    dc = -sum(sy);
    dd = sum(s);
    
    c = c - alpha*dc;
    d = d - alpha*dd;
    
    figure(1);
    plot(t1, y1, 'rx-'); hold on;
    plot(t2-t, c*y2, 'bo-'); hold off;
    pause(0.01);
end
%}