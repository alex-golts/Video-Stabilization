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

function [gyro frame_time] = import_video_data(file)
% loads gyro data from text file. format: x y z timestamp

fid = fopen(['data/' file '.txt']);
gyro = read_mat_from_file(fid);
frame_time = read_mat_from_file(fid);
fclose(fid);

d = diff(frame_time);
frame_gap = (d ./ min(d)) - 1;

display(['avg frame rate: ' num2str(1/mean(d))]);
display(['max frame rate: ' num2str(1/min(d))]);
display(['max frame gap: ' num2str(max(frame_gap))]);
display(['num frames dropped: ' num2str(sum(round(frame_gap)))]);

d = gyro(2:end,4) - gyro(1:end-1,4);
display(['max gyro rate: ' num2str(1/min(d))]);
display(['min gyro rate: ' num2str(1/max(d))]);

function d = read_mat_from_file(fid)
    s = textscan(fid, 'size: [%n %n]', 1, 'commentStyle', '#');
    s = [s{2} s{1}];
    d = textscan(fid, '%n', prod(s), 'commentStyle', '#');
    d = reshape([d{:}], s)';
