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

%% extract data from captured video & gyro
clear all;

video_file = 'shake';

[gyro frame_time] = import_video_data(video_file);
%gyro(:,1) = -gyro(:,1);
run ../vlfeat-0.9.9/toolbox/vl_setup.m

transformtype = 'projective';

warning('off', 'Images:initSize:adjustingMag');

%% 

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);

num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

base_frame = read(xyloObj, 1);
Ib = single(rgb2gray(base_frame)) / 255;
[fb, db] = vl_sift(Ib, 'PeakThresh', 0.02) ;

W = struct('tform', {}, 'x1', {}, 'x2', {});

% Read one frame at a time.
for f = 2:num_frames
    frame = read(xyloObj, f);
    
    Ia = single(rgb2gray(frame)) / 255;
    [fa, da] = vl_sift(Ia, 'PeakThresh', 0.01) ;
%     imshow(frame);
%     vl_plotframe(fa);
    
    [matches, scores] = vl_ubcmatch(da, db);
    matches = matches(:, scores > median(scores));

    ma = fa(1:2,matches(1,:));
    mb = fb(1:2,matches(2,:));

%     imshow([base_frame; frame]);
%     line([ma(1,:); mb(1,:)],[ma(2,:); mb(2,:) + 720],'Color','b');
    
    [tform, inlier_idx, best_error] = ransacfit(frame, base_frame, ma', mb', transformtype, 1, 50);

    imshow([base_frame; frame]);
    line([ma(1,inlier_idx); mb(1,inlier_idx)],[ma(2,inlier_idx); mb(2,inlier_idx) + 720],'Color','b');
    pause(0.1);
    
%     imshow(imtransform(frame, tform, 'XData', [1 size(frame,2)], 'YData', [1 size(frame,1)]));
%     hold on;
%     h = imshow(base_frame);
%     set(h, 'AlphaData', 0.6);
%     pause(0.1);

    base_frame = frame;
    fb = fa;
    db = da;
    W(f-1).tform = tform;
    W(f-1).x1 = ma(:,inlier_idx)';
    W(f-1).x2 = mb(:,inlier_idx)';
    fprintf('num inliers: %d, error: %d, frame %d of %d\n', numel(inlier_idx), best_error, f, num_frames);
end

%%

load(['mat/exp_register_frames_' transformtype '.mat'], 'W', 'transformtype', 'c', 't0', 'ts');
%%
% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
 
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

A = eye(3);

outvid = VideoWriter(['mat/' video_file '_registered_' transformtype '.avi']);
open(outvid);

frame = read(xyloObj, 1);
writeVideo(outvid, frame);

% Read one frame at a time.
for f = 2:num_frames
    frame = read(xyloObj, f);
    
%     A = W{f-1};
    A = A * (W{f-1}.tdata.T');
    stabilized = imtransform(frame,  maketform(transformtype, A'), 'XData', [1 size(frame,2)], 'YData', [1 size(frame,1)]);
%     imshow(stabilized);
%     pause(0.01);
    
    writeVideo(outvid, stabilized);
    display([num2str(f) ' of ' num2str(num_frames)]);
end

close(outvid);

%%

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);

num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

frame1 = read(xyloObj, 1);

w = vid_width;
h = vid_height;

x = [0 0 1; 0 h 1; w 0 1; w h 1; w/2 h/2 1];
tx = ones([size(x) num_frames-1]);

% Read one frame at a time.
for f = 2:num_frames
%     frame2 = read(xyloObj, f);
    i = f-1;
    
    A = W(i).tform.tdata.T;
    tx(:,:,i) = x * A;

%     figure(1); clf;
%     h1 = imshow(frame1); hold on;
%     h2 = imshow(imtransform(frame2, maketform(transformtype, A), 'XData', [1 w], 'YData', [1 h]));
%     set(h2, 'AlphaData', 0.6);
%     plot(tx([1 2 4 3 1],1,i), tx([1 2 4 3 1],2,i), 'c-');
%     plot(tx(5,1,i), tx(5,2,i), 'rx');
%     hold off;
%     pause(0.1);
%     
%     frame1 = frame2;
end

%%

top_left_to_right = reshape(tx(3,1:2,:) - tx(1,1:2,:), 2, num_frames-1)';
top_left_to_bottom = reshape(tx(2,1:2,:) - tx(1,1:2,:), 2, num_frames-1)';

mag = sqrt(sum(top_left_to_right .* top_left_to_right,2));
dlr = top_left_to_right ./ mag(:,[1 1]);

% project top_left_to_bottom onto top_left_to_right to find the shear
a = sum(dlr .* top_left_to_bottom,2) / h;

% project top_left_to_bottom onto perp(top_left_to_right) to find the
% stretch
pdlr = [dlr(:,2) dlr(:,1)];
pdlr(:,1) = -pdlr(:,1);

s(:,1) = (mag / w);
s(:,2) = (sum(pdlr .* top_left_to_bottom,2) / h);

theta = asin(dlr(:,2));

t = reshape(tx(1,1:2,:), 2, num_frames-1)';

%%

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);

num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

frame1 = read(xyloObj, 1);

w = vid_width;
h = vid_height;

% Read one frame at a time.
for f = 2:num_frames
    frame2 = read(xyloObj, f);
    i = f-1;
    
    A = [1 a(i) 1; 0 1 0; 0 0 1];
    S = [s(i,1) 0 0; 0 s(i,2) 0; 0 0 1];
    R = [cos(theta(i)) -sin(theta(i)) 0; sin(theta(i)) cos(theta(i)) 0; 0 0 1];
    T = [1 0 t(i,1); 0 1 t(i,2); 0 0 1];
    M = T*R*S*A;

    figure(1); clf;
    h1 = imshow(frame1); hold on;
    h2 = imshow(imtransform(frame2, maketform(transformtype, M'), 'XData', [1 w], 'YData', [1 h]));
    set(h2, 'AlphaData', 0.6);
    plot(tx([1 2 4 3 1],1,i), tx([1 2 4 3 1],2,i), 'c-');
    plot(tx(5,1,i), tx(5,2,i), 'rx');
    hold off;
    pause;
    
    frame1 = frame2;
end

%%

dtime = diff(frame_time);
dt = t ./ dtime(:,[1 1]);
dt_time = (frame_time(1:end-1) + frame_time(2:end)) / 2;
%%

[c1, d1, t1] = registersignals(dt_time, dt(:,1), gyro(:,4), gyro(:,1));
%%
figure(2);
plot(gyro(:,4) - t1, c1*gyro(:,1) - d1, 'rx-'); hold on;
plot(dt_time, dt(:,1), 'bo-'); hold off;
legend('gyro', 'image');

%%

dT = diff(gyro(:,4));
theta = ((gyro(1:end-1,1:3) + gyro(2:end,1:3)) / 2) .* dT(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];

[c t0 ts d err] = solve_affine_drift(gyro, frame_time, W, video_file);
%%
load('mat/camera_params.mat', 'c', 't0', 'ts', 'd', 'err');

%% affine alignment

g = gyro(:,1:3);
g(:,1) = -g(:,1);
%d = -mean(g,1);
g = g + d(ones(size(g,1),1),:);

[dth, dths] = frame_rotations_high_pass(g, gyro(:,4), frame_time, t0, ts);
dths = dths(2:end,:);

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

A = eye(3);
T1 = translation_matrix(vid_width/2, vid_height/2);
T2 = translation_matrix(-vid_width/2, -vid_height/2);

outvid = VideoWriter(['mat/' video_file '_registered_affine.avi']);
open(outvid);

crop_amount = 0;
frame = read(xyloObj, 1);
writeVideo(outvid, cropim(frame, crop_amount, crop_amount));

% Read one frame at a time.
for f = 1:num_frames-1
    frame = read(xyloObj, f+1);
    
    S = [1 c*dths(f,1)/720 0; 0 1+c*dths(f,2)/720 0; 0 0 1];
    R = T1 * [cos(dth(f,3)) -sin(dth(f,3)) 0; sin(dth(f,3)) cos(dth(f,3)) 0; 0 0 1] * T2;
    T = [1 0 c*dth(f,1); 0 1 c*dth(f,2); 0 0 1]; 
    M = T*R*S;
    
    A2 = A/M;
    
    M = T*R;
    A = A/M;
    
    stabilized = imtransform(frame,  maketform(transformtype, A2'), 'XData', [1 size(frame,2)], 'YData', [1 size(frame,1)]);
%     imshow(cropim(stabilized, crop_amount, crop_amount));
%     pause(0.01);
    
    writeVideo(outvid, cropim(stabilized, crop_amount, crop_amount));
    display([num2str(f+1) ' of ' num2str(num_frames)]);
end

close(outvid);

%% projective alignment

g = gyro(:,1:3);
g(:,1) = -g(:,1);
%d = -mean(g,1);
g = g + d(ones(size(g,1),1),:);

[dth, dths] = frame_rotations_high_pass(g, gyro(:,4), frame_time, t0, ts);
dths = dths(2:end,:);

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
 
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

crop_amount = 50;

A = eye(3);
w = vid_width / 2;
h = vid_height / 2;
fl = -1280*180/(pi*45);

frame = read(xyloObj, 1);
outvid = VideoWriter(['mat/' video_file '_registered_projective_high_pass4.avi']);
open(outvid);
writeVideo(outvid, cropim(frame, crop_amount, crop_amount));

% Read one frame at a time.
for f = 1:num_frames-1
    frame = read(xyloObj, f+1);
    
    S = [1 c*dths(f,1)/720 0; 0 1+c*dths(f,2)/720 0; 0 0 1];
    Rx = [cos(dth(f,1)) 0 sin(dth(f,1)); 0 1 0; -sin(dth(f,1)) 0 cos(dth(f,1))];
    Ry = [1 0 0; 0 cos(-dth(f,2)) -sin(-dth(f,2)); 0 sin(-dth(f,2)) cos(-dth(f,2))];
    Rz = [cos(dth(f,3)) -sin(dth(f,3)) 0; sin(dth(f,3)) cos(dth(f,3)) 0; 0 0 1];
    
    M1 = [1 0 -w; 0 1 -h; 0 0 c];
    M2 = [1 0 w/c; 0 1 h/c; 0 0 1/c];
    
    A = A*Rz*Ry*Rx;
    W = M2*A*S*M1;
    
    stabilized = imtransform(frame,  maketform(transformtype, inv(W)'), 'XData', [1 size(frame,2)], 'YData', [1 size(frame,1)]);
%     imshow(cropim(stabilized, crop_amount, crop_amount));
%     pause(0.01);

    writeVideo(outvid, cropim(stabilized, crop_amount, crop_amount));
    display([num2str(f+1) ' of ' num2str(num_frames)]);
end

close(outvid);

%% 

g = gyro(:,1:3);
g(:,1) = -g(:,1);
%d = -mean(g,1);
g = g + d(ones(size(g,1),1),:);

[dth, dths] = frame_rotations_high_pass(g, gyro(:,4), frame_time, t0, ts);
dths = dths(2:end,:);

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
 
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

crop_amount = 50;

A = eye(3);
w = vid_width / 2;
h = vid_height / 2;
fl = -1280*180/(pi*45);


lbl = {'shake', 'shear_x', 'shear_y', 'shear_xy'};
sxy = [1 1 0; 0 1 1; 1 0 1; 0 0 1];

for i = 1:3

frame = read(xyloObj, 1);
outvid = VideoWriter(['mat/' video_file '_registered_' lbl{i} '.avi']);
open(outvid);
writeVideo(outvid, cropim(frame, crop_amount, crop_amount));

% Read one frame at a time.
for f = 1:num_frames-1
    frame = read(xyloObj, f+1);
    
    S = [1 c*sxy(i,1)*dths(f,1)/720 0; 0 1+c*sxy(i,2)*dths(f,2)/720 0; 0 0 1];
    Rx = [cos(dth(f,1)) 0 sin(dth(f,1)); 0 1 0; -sin(dth(f,1)) 0 cos(dth(f,1))];
    Ry = [1 0 0; 0 cos(-dth(f,2)) -sin(-dth(f,2)); 0 sin(-dth(f,2)) cos(-dth(f,2))];
    Rz = [cos(dth(f,3)) -sin(dth(f,3)) 0; sin(dth(f,3)) cos(dth(f,3)) 0; 0 0 1];
    
    M1 = [1 0 -w; 0 1 -h; 0 0 c];
    M2 = [1 0 w/c; 0 1 h/c; 0 0 1/c];
    
    if sxy(i,3) == 1, A = A*Rz*Ry*Rx; end
    W = M2*A*S*M1;
    
    stabilized = imtransform(frame,  maketform(transformtype, inv(W)'), 'XData', [1 size(frame,2)], 'YData', [1 size(frame,1)]);
%     imshow(cropim(stabilized, crop_amount, crop_amount));
%     pause(0.01);

    writeVideo(outvid, cropim(stabilized, crop_amount, crop_amount));
    display([num2str(f+1) ' of ' num2str(num_frames)]);
end

close(outvid);
break;

end

%% gridwarp

g = gyro(:,1:3);
g(:,1) = -g(:,1);
%d = -mean(g,1);
g = g + d(ones(size(g,1),1),:);

[dth, theta] = frame_rotations_non_lin(g, gyro(:,4), frame_time, t0, ts);

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
 
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

crop_amount = 50;

th = [0 0 0];
w = vid_width / 2;
h = vid_height / 2;
M1 = [1 0 -w; 0 1 -h; 0 0 c];
M2 = [1 0 w/c; 0 1 h/c; 0 0 1/c];

frame = read(xyloObj, 1);
outvid = VideoWriter(['mat/' video_file '_registered_nonlin.avi']);
open(outvid);
% writeVideo(outvid, cropim(frame, crop_amount, crop_amount));
tic;
% Read one frame at a time.
for f = 10:15
    frame = read(xyloObj, f+1);
%     imshow(warped);
%     pause(0.1);
    
    th = th + dth(f,:);
    
    stabilized = gridwarp(frame, M1, M2, theta, gyro(:,4), frame_time(f) - t0, ts);
%     imshow(stabilized);
%     pause;

    writeVideo(outvid, cropim(stabilized, crop_amount, crop_amount));
    display([num2str(f+1) ' of ' num2str(num_frames)]);
end
toc;
close(outvid);

%% meshwarp
clear meshwarpmex

g = gyro(:,1:3);

sigma2 = 4000;
gauss = exp(-(-120:120).^2 / sigma2);
gauss = gauss ./ sum(gauss);
g(:,1) = g(:,1) - conv(gyro(:,1), gauss, 'same');
g(:,2) = g(:,2) - conv(gyro(:,2), gauss, 'same');
g(:,3) = g(:,3) - conv(gyro(:,3), gauss, 'same');
dgt = diff(gyro(:,4));
theta = ((g(1:end-1,:) + g(2:end,:)) / 2) .* dgt(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];
dth = diff(interp1(gyro(:,4) + t0 + ts/2, theta, frame_time, 'linear', 'extrap'));

theta = ((gyro(1:end-1,1:3) + gyro(2:end,1:3)) / 2) .* dgt(:,[1 1 1]);
theta = [0 0 0; cumsum(theta, 1)];

% read corresponding movie
xyloObj = mmreader(['data/' video_file '.mov']);
display(xyloObj);
 
num_frames = xyloObj.NumberOfFrames;
vid_height = xyloObj.Height;
vid_width = xyloObj.Width;

crop_amount = 100;

w = vid_width / 2;
h = vid_height / 2;
M1 = [1 0 -w; 0 1 -h; 0 0 c];
M2 = [1 0 w/c; 0 1 h/c; 0 0 1/c];

frame = read(xyloObj, 1);
outvid = VideoWriter(['mat/' video_file '_meshwarp.avi']);
open(outvid);
writeVideo(outvid, cropim(frame, crop_amount, crop_amount));

% Read one frame at a time.
for f = 1:num_frames-1
    frame = read(xyloObj, f+1);
%     imshow(warped);
%     pause(0.1);
    
    stabilized = meshwarp(frame, dth(f,:), theta, gyro(:,4), frame_time(f+1), t0, ts, c);
%     imshow(stabilized);
%     pause(0.1);

    writeVideo(outvid, cropim(stabilized, crop_amount, crop_amount));
    display([num2str(f+1) ' of ' num2str(num_frames)]);
end

close(outvid);

%%
%{

%%
W = imalign(frame2, frame1, [0 0 0 0 0 0]);
%%
Ia = single(rgb2gray(frame1)) / 255;
Ib = single(rgb2gray(frame2)) / 255;

[fa, da] = vl_sift(Ia, 'PeakThresh', 0.02) ;
[fb, db] = vl_sift(Ib, 'PeakThresh', 0.02) ;
[matches, scores] = vl_ubcmatch(da, db);

matches = matches(:, scores > mean(scores));

ma = fa(1:2,matches(1,:));
mb = fb(1:2,matches(2,:));

tform = ransacfit(frame1, frame2, ma', mb', 'affine', 1, 40);

imshow([frame1; frame2]);
% h1 = vl_plotframe(fa);
% h2 = vl_plotframe(fb);
% set(h1,'color','y','linewidth',2) ;
% set(h2,'color','g','linewidth',2) ;
line([ma(1,:); mb(1,:)],[ma(2,:); mb(2,:) + 720],'Color','b');

%}
