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

function coords = find_dot(mov)
movie(mov, 1, 30);

frames = cellfun(@rgb2gray, {mov(:).cdata}, 'UniformOutput', false);
dims = [size(frames{1}) numel(frames)];
frames = [frames{:}];
frames = reshape(frames, dims);

mask = zeros(dims(1:2));
mask(round(dims(1)/3):round(dims(1)*2/3), round(dims(2)/3):round(dims(2)*2/3)) = 1;

frames(repmat(uint8(mask), [1 1 dims(3)]) < 0.1) = 255;
mask = frames < 100;
for i=1:188; figure(1); imshow(mask(:,:,i)); pause(1/30); end

p = zeros(size(mask,3),2);

for i=1:size(mask,3)
    [y,x] = find(mask(:,:,i));
    p(i,:) = [mean(x), mean(y)];
end

coords = p;

%for i=1:188; figure(1); imshow(frames(:,:,i)); pause(0.1); end

