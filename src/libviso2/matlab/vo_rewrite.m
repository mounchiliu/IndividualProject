% Simple visual odometry plotting from scratch.
% Can compare the ground truth to the VO and visualize the results using the
% KITTI, KITTI-odometry, and Karlsruhe datasets.
%
% TODO(andrei): Autodetect frame count.
% TODO(andrei): GT plotting support for Karlsruhe.
% TODO(andrei): Compute ATE and related metrics using identical technique as in
% the official C++ devkit.

% OxTS: X = forward, Y = left, Z = up (see: http://www.cvlibs.net/datasets/kitti/setup.php)

clear all;
close all;
dbclear if error;   % Disable pausing on error
first_frame = 0;
addpath('kitti-devkit/');

% Make sure you set the 'is_kitti' flag depending on what dataset you're reading!
% img_dir     = '/Users/andrei/Datasets/karlsruhe/2010_03_09_drive_0019';

% The 77 frame sequence #52 is particularly challenging, because it features
% the car stopping at a red light surrounded by other vehicles, while a large
% truck on the right starts turning right, confusing the VO system (but not by
% much).
% img_dir = '/Users/andrei/Datasets/kitti/2011_09_26/2011_09_26_drive_0052_sync';
% last_frame  = 77;

% Bike and van, smooth curve, short.
img_dir = '/Users/andrei/Datasets/kitti/2011_09_26/2011_09_26_drive_0005_sync';
last_frame = 150;

% Slightly curved road through forest with few cars coming from opposing
% direction (481 frames). In the end, the camera stops, and a few cars drive
% in front of it, but it remains robust to that motion and correctly detects it
% is actually not moving.
% img_dir = '/Users/andrei/Datasets/kitti/2011_09_26/2011_09_26_drive_0019_sync';
% first_frame = 0;
% last_frame = 481;

% img_dir = '/Users/andrei/Datasets/kitti/2011_10_03/2011_10_03_drive_0047_sync';
% first_frame = 0;
% last_frame = 836;

% img_dir = '/Users/andrei/Datasets/kitti/2011_09_26/2011_09_26_drive_0101_sync';
% last_frame = 935;

% Works well.
% img_dir = '/Users/andrei/Datasets/kitti/2011_09_29/2011_09_29_drive_0004_sync';
% last_frame = 338;

% img_dir = '/Users/andrei/Datasets/kitti/2011_09_26/2011_09_26_drive_0018_sync';
% last_frame = 269;

% Straight road through forest, many cars.
% img_dir = '/Users/andrei/Datasets/kitti/2011_09_26/2011_09_29_drive_0004_sync';

% Kitti odometry dataset sample: sequence 06, car moving straight ahead.
img_dir = '/Users/andrei/Datasets/kitti/odometry-dataset/sequences/06/';
% Sequence '02' involves the car going up a gentle hill.
img_dir = '/Users/andrei/Datasets/kitti/odometry-dataset/sequences/02/';
% img_dir = '/Users/andrei/Datasets/kitti/odometry-dataset/sequences/05/';
first_frame = 0;
last_frame = 2760;

param.f     = 645.2;
param.cu    = 635.9;
param.cv    = 194.1;
param.base  = 0.571;    % Stereo rig baseline, in meters.

% last_frame = 350;
% last_frame = 50;

visualize_odometry_3d = false;
% Select kitti vs. kitti-odometry vs. Karlsruhe. TODO(andrei): String or enum.
is_kitti = false;
is_kitti_odometry = true;

% last_frame = 372;
plot_ground_truth = true;
plot_every = 10;

param.multi_stage = 1;
param.ransac_iters = 200;   % Default is 200
% param.match_disp_tolerance = 1;   % Default is 2

param.half_resolution = 1;
param.refinement = 0;

visualOdometryStereoMex('init', param);
fprintf('Stereo visual odometry code initialized successfully.\n');

figure('units', 'normalized', 'position', [0.05 0.15 0.9 0.8]);
% xlim([-10, 10]);
% ylim([  0, 20]);

% 'process(I1, I2, [replace])', 'process_matched(NxM)', 'num_matches()',
% 'get_matches()', 'num_inliers()', 'get_inliers()', 'get_indices()' (indices of
% matches (in what?)), 'close'.

current_pose = eye(4);
last_t = current_pose(1:3, 4);

last_gt_pose = eye(4);

if ~is_kitti && ~is_kitti_odometry && plot_ground_truth
  error('Plotting the ground truth for Karlsruhe data sequences is not supporteddl.')
end

if plot_ground_truth
  % Use MATLAB helpers from the KITTI devkit to convert the OxTS IMU/GPS ground
  % truth data to easy-to-use poses.
  if is_kitti
    oxts_frames = loadOxtsliteData(img_dir);
    pose = convertOxtsToPose(oxts_frames);
  elseif is_kitti_odometry
    % This is used for when KITTI-odometry sequences are used, instead of raw
    % KITTI or raw Karlsruhe sequences.
    vo_bench_poses_fpath = [img_dir, '../../poses/02.txt'];
    vo_bench_poses = dlmread(vo_bench_poses_fpath);
  else
    printf("No GT pose support for KITTI-odometry for now.\n");
  end
end

for i = first_frame:last_frame
  tic
  if is_kitti
    left_path   = sprintf('%s/image_00/data/%010d.png', img_dir, i);
    right_path  = sprintf('%s/image_01/data/%010d.png', img_dir, i);
    % Ground truth pose computed by the OxTS system.
%     oxts_path   = sprintf('%s/oxts/data/%010d.txt', img_dir, i);
  elseif is_kitti_odometry
    left_path   = sprintf('%s/image_0/%06d.png', img_dir, i);
    right_path  = sprintf('%s/image_1/%06d.png', img_dir, i);
  else
    left_path   = sprintf('%s/I1_%06d.png', img_dir, i);
    right_path  = sprintf('%s/I2_%06d.png', img_dir, i);
  end
  
  if left_path == right_path
    error('Left and right paths are identical. Odometry computation will be nonnsensical. This is probably a bug; please check your filenames!');
  end
  
  left = imread(left_path);
  right = imread(right_path);
  
%   tic
  transform = visualOdometryStereoMex('process', left, right);
  p_matched = visualOdometryStereoMex('get_matches');
  current_pose = current_pose * inv(transform);
  t_new = current_pose(1:3, 4).';
%   fprintf('Visual odometry: %.4f\n', toc);

%   tic
  % Hand-tweaked subplot size to optimize space usage.
  subplot(4, 2, [1, 2], 'Position', [0.1, 0.73, 0.85, 0.28]);

  % Render sparse scene flow in real time for visualization purposes.
  plotMatch(left, p_matched, 2);
  
  subplot(4, 2, 3:8, 'Position'); %, [0.05, 0.05, 0.90, 0.7]);
  hold on;
  % Enforce consistent aspect ratio in trajectory plot.
  daspect([1, 1, 1]);
  
  if mod(i + 1, plot_every) == 0
    if visualize_odometry_3d
      scatter3(t_new(1), t_new(2), t_new(3), 'k');
      plot3([last_t(1), t_new(1)], [last_t(2), t_new(2)], [last_t(3), t_new(3)], '-k');

      if plot_ground_truth
        warning('Ground truth plotting not available in 3D.');
      end
    else
      scatter(t_new(1), t_new(3), 'k');
      plot([last_t(1), t_new(1)], [last_t(3), t_new(3)], '-k');

      if plot_ground_truth
        if is_kitti
          gt_pose = pose{i+1};
          t_gt = gt_pose(1:3, 4).';
          last_t_gt = last_gt_pose(1:3, 4).';

          % Plot the ground truth trajectory
          plot([-last_t_gt(2), -t_gt(2)], [last_t_gt(1), t_gt(1)], 'g-');
          scatter(-t_gt(2), t_gt(1), 50, 'g');
          
          % Plot the discrepancy between the estimated and actual pose in red.
          plot([-t_gt(2), t_new(1)], [t_gt(1), t_new(3)], 'r-', 'LineWidth', 2);

          last_gt_pose = gt_pose;
        else
          vo_bench_P = reshape(vo_bench_poses(i, :), 4, 3).';
          vo_bench_t = vo_bench_P(1:3, 4);
          % Z = forward/backward, Y = down/up, X = left/right.
          % Remember, if you go up a hill, Y becomes SMALLER.
          % This can clearly be seen by messing around with the following lines.
          scatter(vo_bench_t(1), vo_bench_t(3), 75, 'b');
          fprintf("Height:          %.6f\n", vo_bench_t(2));
          fprintf("FWD (wrt start): %.6f\n", vo_bench_t(3));
        end
      end
    end
    
    last_t = t_new;
  end
  
%   fprintf('Plotting: %.4f\n', toc);

  pause(0.001);
  refresh;
  frame_delta = toc;
  if mod(i + 1, 10) == 0
    fprintf('Total: %.4f @ %.2f FPS\n', frame_delta, 1/frame_delta);
  end
end

visualOdometryStereoMex('close', param);
fprintf('Stereo visual odometry code shut down successfully.\n');