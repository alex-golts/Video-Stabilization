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

%% plot gyro traces
clear all;
close all;

video_file = 'output16';
[gyro frame_time] = import_video_data(video_file);

figure(1);
xlim([0 1.5]);
ylim([-0.5 0.65]);
hold on;
plot(gyro(:,4) - min(gyro(:,4)) - 2, gyro(:,1), 'r');
plot(gyro(:,4) - min(gyro(:,4)) - 2, gyro(:,2), 'g');
plot(gyro(:,4) - min(gyro(:,4)) - 2, gyro(:,3), 'b');
xlabel('time (s)');
ylabel('\omega (rad/s)');

%%
figure(2);
xlim([0 4]);
ylim([-0.7 0.7]);
hold on;
plot(gyro(:,4) - min(gyro(:,4)), gyro(:,2), 'g');
xlabel('time (s)');
ylabel('\omega_y (rad/s)');


figure(3);
xlim([0 4]);
ylim([-0.7 0.7]);
hold on;
plot(gyro(:,4) - min(gyro(:,4)), gyro(:,3), 'b');
xlabel('time (s)');
ylabel('\omega_z (rad/s)');