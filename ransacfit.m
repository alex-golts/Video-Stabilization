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

function [BEST_TFORM, inlier_idx, best_error] = ransacfit(input_frame, base_frame, input_points, base_points, transformtype, dist_thresh, inlier_thresh)
% uses ransac to robustly fit input_points to base_points

% turn off warning for near singlar matrixes, since we won't be using them
% anyway
s = warning('off', 'Images:maketform:conditionNumberofAIsHigh');

if (strcmp(transformtype, 'affine'))
    num_cps = 3;
elseif strcmp(transformtype, 'projective')
	num_cps = 4;
else
    error('Unknown transform type "%s".', transformtype);
end

if size(input_points,2) < 3
    input_points(:,3) = ones(size(input_points,1),1);
end

num_points = size(input_points,1);
num_iter = 1000;

dist_thresh = dist_thresh * dist_thresh; % make the distance squared

best_error = inf;
BEST_TFORM = [];
inlier_idx = [];

for i=1:num_iter
    found_points = 0;
    while ~found_points
        try
            % randomly pick a set of points
            while 1
                idx = floor(rand(num_cps,1)*num_points) + 1;
                if numel(unique(idx)) == numel(idx)
                    break;
                end
            end

            % compute a transformation
            TFORM = cp2tform(input_points(idx,1:2), base_points(idx,:), transformtype);
            if isfield(TFORM.tdata, 'T')
                found_points = 1;
            end
        catch e
            %warning(e.message);
        end
    end
    
    warped_points = input_points * TFORM.tdata.T;
    warped_points = warped_points(:,1:2) ./ warped_points(:,[3 3]);
    err = base_points - warped_points;
    err = sum(err .* err, 2);
    
    idx = err < dist_thresh;
    num_inliers = sum(idx);
    if (num_inliers > inlier_thresh)
        % refine our transformation estimate
        TFORM = cp2tform(input_points(idx,1:2), base_points(idx,:), transformtype);
        warped_points = input_points(idx,:) * TFORM.tdata.T;
        warped_points = warped_points(:,1:2) ./ warped_points(:,[3 3]);
        err = base_points(idx,:) - warped_points;
        err = mean(sum(err .* err, 2));
        
        inlier_thresh = num_inliers;
    
        %if (err < best_error)
            best_error = err;
            inlier_idx = find(idx);
            BEST_TFORM = TFORM;

            %{
            figure(1);
            display(err);
            display(num_inliers);
            imshow(imtransform(input_frame, TFORM, 'XData', [1 size(input_frame,2)], 'YData', [1 size(input_frame,1)]));
            hold on;
            h = imshow(base_frame);
            set(h, 'AlphaData', 0.6);
            pause;

            a = input_points(idx,:)';
            b = base_points(idx,:)';

            figure(2);
            imshow([input_frame; base_frame]);
            line([a(1,:); b(1,:)],[a(2,:); b(2,:) + 720],'Color','b');
            %}
        %end
    end
end

% restore warning state
warning(s);