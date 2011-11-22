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

function W = imalign(T, I, p)
% align the template image T to the input image I

if size(T,3) == 3
    T = rgb2gray(T);
end
if size(I,3) == 3
    I = rgb2gray(I);
end
if max(T(:)) > 1
    T = double(T) / 255;
end
if max(I(:)) > 1
    I = double(I) / 255;
end

gradTx = filter2(fspecial('sobel')', T);
gradTy = filter2(fspecial('sobel') , T);
%{
figure(1); showim(reshape(gradTx, size(T)));
figure(2); showim(reshape(gradTy, size(T)));
pause;
%}

x = repmat((1:size(T,2)) , size(T,1), 1);
y = repmat((1:size(T,1))', 1, size(T,2));

gradT_dWdp = [gradTx(:) .* x(:) ...
              gradTy(:) .* x(:) ...
              gradTx(:) .* y(:) ...
              gradTy(:) .* y(:) ...
              gradTx(:)         ...
              gradTy(:)         ];

H = gradT_dWdp' * gradT_dWdp;

p = p(:);

for i=1:100
    W = [1+p(1)  p(3)  p(5); ...
          p(2)  1+p(4) p(6)];
      
    figure(1); clf;
    imshow(imtransform(T,  maketform('affine', W'), 'XData', [1 size(I,2)], 'YData', [1 size(I,1)]));
    hold on;
    h = imshow(I);
    set(h, 'AlphaData', 0.6);
    pause(0.01);
    
    WI = imtransform(I,  maketform('affine', W'), 'XData', [1 size(I,2)], 'YData', [1 size(I,1)], 'FillValues', NaN);
    errorIm = WI - T;
    errorIm(isnan(errorIm)) = 0;
    delp = H \ (gradT_dWdp' * errorIm(:));
    p = p + delp;
    display(p);
end