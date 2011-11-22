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

function video_to_tga(video_file)

xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
num_frames = xyloObj.NumberOfFrames;

mkdir(['data/' video_file]);

for f = 1:num_frames
    display(['frame ' num2str(f) ' of ' num2str(num_frames)]);
    frame = read(xyloObj, f);
    imwrite(frame, ['data/' video_file '/' num2str(f) '.jpg']);
end