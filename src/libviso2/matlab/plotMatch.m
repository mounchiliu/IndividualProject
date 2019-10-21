function plotMatch(I, p_matched, method, inliers)

if nargin<3
  method = 2;
end

if nargin<4
  inliers = 1:size(p_matched,2);
end

p_matched = p_matched';

% show image
cla,imshow(uint8(I)),hold on;

% show matches
if method==0

  for i=1:size(p_matched,1)
    col = [1 0 0];
    if ~any(inliers==i)
      col = [0 0 1];
    end
    line([p_matched(i,1) p_matched(i,3)], ...
         [p_matched(i,2) p_matched(i,4)], 'Color', col,'LineWidth',1);
    plot(p_matched(i,3),p_matched(i,4),'s', 'Color', col,'LineWidth',1,'MarkerSize',2);
    %text(p_matched(i,3),p_matched(i,4),sprintf('%d',i));
  end
  
elseif method==1
  
  disparity = p_matched(:,1)-p_matched(:,3);
  disparity = min(disparity,50);
  max_disp  = max(disparity(inliers));
  %max_disp  = min(max_disp,50);

  for i=1:size(p_matched,1)
    c = abs(disparity(i)/max_disp);
    col = [c 1-c 0];
    if ~any(inliers==i)
      col = [0 0 1];
    end
    line([p_matched(i,1) p_matched(i,1)], ...
         [p_matched(i,2) p_matched(i,4)], 'Color', col,'LineWidth',2);
    plot(p_matched(i,1),p_matched(i,2),'s', 'Color', col,'LineWidth',2,'MarkerSize',2);
  end
  
elseif method == 2
  % Scene flow rendering
  
  disparity = p_matched(:,1) - p_matched(:,3);
  max_disp  = max(disparity(inliers));
  %max_disp  = 80;
  
  c_vals = min(abs(disparity / (max_disp + 0.1)), 1);
  
  % Used to colored the rendered points based on the disparity.
  cols = [c_vals, 1 - c_vals, zeros(size(c_vals))];
  
  outlier_idxs = setdiff(1:length(p_matched), inliers);
  % Color outliers blue.
  if ~isempty(outlier_idxs)
    cols(outlier_idxs, :) = [0; 0; 1];
  end

  % Draw the blobs
  scatter(p_matched(:, 5), p_matched(:, 6), 24, cols, 'filled');

  % Draw the projected scene flow arrows
  us = p_matched(:, 5) - p_matched(:, 1);
  vs = p_matched(:, 6) - p_matched(:, 2);
  q = quiver(p_matched(:, 5), p_matched(:, 6), us, vs, 'LineWidth', 1.5, ...
    'MaxHeadSize', 0.75);
  
  % A few hacks to color the arrows properly without looping...
  cmap = uint8([cols, ones(size(c_vals))] * 255);
  cmap = reshape(cmap, [], 1, 4);
  
  % We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
  cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);
  set(q.Head, ...
    'ColorBinding', 'interpolated', ...
    'ColorData', reshape(cmap, [], 4).');
  set(q.Tail, ...
    'ColorBinding', 'interpolated', ...
    'ColorData', reshape(cmap(1:2, :, :), [], 4).');
    
  % Old, slower, plotting code, for reference
%   for i=1:size(p_matched,1)
%     c = min(abs(disparity(i)/(max_disp+0.1)),1);
%     col = [c 1-c 0];
%     if ~any(inliers==i)
%       col = [0 0 1];
%     end
%     line([p_matched(i,1) p_matched(i,5)], ...
%          [p_matched(i,2) p_matched(i,6)], 'Color', col,'LineWidth',2);
%     plot(p_matched(i,5),p_matched(i,6),'s', 'Color', col,'LineWidth',2,'MarkerSize',3);
%   end
  
else
  error(['Unknown method ID ' num2str(method)]);
end
  
end
