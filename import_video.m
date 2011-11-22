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

function [mov num_frames vid_height vid_width frame_rate duration] = import_video(file)

% read corresponding movie
xyloObj = mmreader(['data/' file '.mov']);
display(xyloObj);

num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;
frame_rate = xyloObj.FrameRate;
duration = xyloObj.Duration;

% Preallocate movie structure.
mov(1:num_frames) = ...
    struct('cdata', zeros(vid_height, vid_width, 3, 'uint8'),...
           'colormap', []);
     
% Read one frame at a time.
for k = 1 : num_frames
    mov(k).cdata = read(xyloObj, k);
end
% Play back the movie once at the video's frame rate.
% movie(mov, 1, xyloObj.FrameRate);

%save(['mat/' file], 'gyro', 'frame_time', 'mov', 'num_frames', 'vid_height', 'vid_width', 'frame_rate', 'duration');
