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

function X = track_video_markers(video_file)

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);

num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

% initialize markers for first frame
figure(1); imshow(read(xyloObj,1));

x=[];
while isempty(x)
    [x, y] = ginput;
    display([x y]);
end

rect_size = 160;

X = cell(numel(x), 1);

% Read one frame at a time.
for f = 1:num_frames
    frame = read(xyloObj, f);
    
    for i = 1:numel(x)
        min_x = max(round(x(i) - rect_size/2), 1);
        max_x = min(round(x(i) + rect_size/2), vid_width);

        min_y = max(round(y(i) - rect_size/2), 1);
        max_y = min(round(y(i) + rect_size/2), vid_height);
        
        patch = double(rgb2gray(frame(min_y:max_y, min_x:max_x, :)));
        min_patch = min(patch(:));
        patch = (patch - min_patch) / (max(patch(:)) - min_patch);
        
        figure(2); subplot(numel(x), 1, i);
        imshow(patch < 0.2);
        
        [py, px] = find(patch < 0.2);
        x(i) = min_x + mean(px);
        y(i) = min_y + mean(py);
        X{i} = [X{i}; x(i) y(i)];
    end
    
    
    figure(1);
    imshow(frame); hold on;
    colors = {'rx', 'gx', 'bx', 'cx'};
    for i = 1:numel(x)
        plot(x(i), y(i), colors{mod(i-1,4)+1});
    end
    hold off;
end