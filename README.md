# IndividualProject
Master Individual Project on Simultaneous Localization and Mapping (SLAM) in dynamic outdoor environment

## Records for encoutered questions or problems
For the evaluation of the system, do I need to validate the tracking accuracy of the dynamic objects?

How can I implement it?

The public datasets for SLAM did not provide the ground truth for the dynamic objects only the ground truth for the robot (camera) pose.

**ANS**

(Ref: email of Dr. Julier)

As you've seen in your literature review, basically nobody has ground truth of the other objects. The reason is that they need to be "cooperative": they need to have their own ground truthing (e.g., GPS) on board or you have some other kind of super accurate sensing system. The other thing would be to manually set things up. E.g., with a dense laser scanner, you could manually fit a 3D model and figure out stuff from there.


## Records for things need to do and things have done
--------------------------------------------------------------------------------------------------------
### 26th Feb
**Need to do**

1.Have a look at DynSLAM (https://github.com/AndreiBarsan/DynSLAM).
  This system provides a dense SLAM system which attempts to reconstruct moving objects.

2.Try cuda 8

3.Implement DynSLAM, try it on different datasets and evaluate it.  
  Have a try to test it with scenes in London which are quite different with scenes in KITTI dataset.
  (London has more tall buildings & cars may drive in a different way.)
  Try the algorithm for sparse mapping.

4.Have a look at DATMO (https://www.ri.cmu.edu/pub_files/pub4/wang_chieh_chih_2007_1/wang_chieh_chih_2007_1.pdf) 
  which describes the problem of jointly doing SLAM and object tracking. 
  It is very detailed in the maths. The main thing to check at this stage 
  would be the introduction which states the problem
  
--------------------------------------------------------------------------------------------------------------------------
### 09th May
**Done**

Have installed cuda 8 on my laptop.


Have got a try on DynSlam on the provided demo sequence.

For the records of setting up the DynaSLAM: see https://github.com/mounchiliu/EverythingAboutSLAM/blob/master/Tip2_DynSLAM


-------------------------------------------------------------------------------------------------------------------------- 
### 10th May
**Things need to do**

Have a look at DynSLAM, does it provide the trajectory of the dynamic onjects?

Have a look at the code of DynSLAM.

Try DynSLAM on other datesets （e.g. KITTI dataset)

Find limitations of the system.
- What if we increase the number of moving objects?
- What if the objects are occluded?
- Laser scanner?  --- Mentioned in paper:

` 'We base our experiments on the video sequences from (a)
the KITTI odometry and (b) the KITTI tracking benchmarks,
using the LIDAR as a ground truth for evaluating the quality
of both the reconstructed static maps, as well as of the
dynamic object instances.'`

-------------------------------------------------------------------------------------------------------------------------- 
### 30th May
**Things need to do**

Real-time?

Frame-to-frame tracked reconstruction of dynamic objects? Yes. The system reconstructs all the potential dynamic objects

Which kind of semantic method it use? MNC Network

-------------------------------------------------------------------------------------------------------------------------

### 10th June
**Things have done**

Successfully extracted objects' motion and the camera poses.  

Draw the camera pose with the objects trajectory from frame to frame.

Problem: For the provided dataset, there are few dynamic objects.  Therefore, the drawing cant show the trajectoty of the dynamic objects.  I will try some other datasets to dest the robustness of the system.


**Things need to do**

Have a look at the segmented network, try on it.  

Test the system with other datasets which have several dynamic objects.

--------------------------------------------------------------------------------------------------------------------------

### 24th June
Have test on kitti tracking datasets which have several dynamic objects in one sequence.

Current problem:

- Shows large error on the estimation of vehicle motion when the camera do the rotation.
- Some of dynamic objects determined to be 'uncertain' state.  It seems the system does not track these uncertain objects.  Therefore, according to the result shown, for a sequence with multiple dynamic objects, the system can only track some of them.

**Things need to do**

Track the objects with 3d model (the reconstruction)

--------------------------------------------------------------------------------------------------------------------------

#### 1st July

**1. Reconstruction**

All the potential dynamic objects are reconstructed using the first frame that observe this object.  For each object, its corresponding RGB and depth data are extracted using the mask resulting from the segmentation procedure.  With these information and the estimated motions of the objects from frame to frame, the object can be reconstructed and also update the reconstruction from frame to frame.

- **Results:**

Rotation:

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/1-object0.png)

Some other object:

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/1-object9.png)


The redlines show the sparse flow estimation.  

Potential problem:

For this condition, (I guess) the surface of the dynamic object change from frame to frame which may cause errors when we do the sparse flow estimation.  We can not find enough matches for motion estimation.  Therefore, the reconstruction is not perfect compare to the other object.


- **video**

(The system also update the dynamic object reconstruction in the static sceen from frame to frame.)

[![video]()](https://www.youtube.com/watch?v=MMSZ_37sGRY" )



The estimated trajectory of each object: (red line for camera poses, the others for tracked objects)

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/1-1.png)
![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/1-2.png)
![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/1-3.png)



**2.Crop the sequence to have less dynamic objects to track,**


![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/2-screen.png)

- **video**

[![video]()](https://www.youtube.com/watch?v=2u-oRcYf4Bk)




The estimated trajectory of each object:
![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/2-1-2.png)
![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/2-2-2.png)


**Things need to do**

1. Have a look at libviso2 to see whether I can have some improvement on the sparse flow estimation.
(Have a try on optical flow?)

**3. Test on detected static objects** (e.g. parked car)

For these objects, I can set the motion to the identity matrix if the system detects the static object to improve the accuracy.

Here is the result:
![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/static-1.png)

However, without the setting of identity matrix, the motion matrix should be approximate to the identity matrix from frame to frame.

Here is the result,

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/static-2.png)

There is small shift for the static objects.  It seems the error shifts grow with time.

For static objects, identity matrix can be set to improve the accuracy.  However, for dynamic objects, there isnt such kind of method to improve the accuracy. 

[Note: This may because of the error of the camera poses. For Dense SLAM, we may focus only on the camera poses or the objects poses separately.  We do not expect the 'overlap errors'].

**Question:**

For the dense slam, is there any optimization method to optimize the estimated the motion of the objects?  
e.g. ORB-SLAM uses Bundle Adjustment to minimize the reprojection error.  Can we do a 'local optimization' for each dynamic object?

**->**

[InfiniTAM (Dense SLAM) has its own Strategies to do the optimization.]

Try InifiTam by blocking the static scene and some other objects. Just focuus on one object.

--------------------------------------------------------------------------------------------------------------------------
### 8 July
**1. Combine detected bounding boxes to get more information of the object**

Previously, based on the results of the segmentation, there may be more than one detected bounding box on the same object.

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/pre_seg.png)

I combined these bounding boxes according to the intersection area of each two boxes.  If the ratio of the intersection area to the bouding box is greater than a threshold, the two boxes will be combined.

Result:

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/cur_seg.png)

**Potential problem and ideas to solve it**

There are several missed or false detections from the semantic segmentation algorithm, which may cause the problem of data association. (The object may lose several frames to do the tracking and reconstruction).  

e.g.

Previous frame:

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/seg_problem_pre.png)

Current frame:

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/seg_problem_cur.png)


ideas:

We may propose a assumption that the dynamic object moves in a constant speed in a short period (e.g. in two consecutive frames).  Then, the moving speed of the object can be used to predict the bounding box of the object in the next frame.  Then, the missed detection can be compensated.

**2. Separate Sparse flow of the scene**

Previously, the system computes the camera poses by finding matches in the scene (including dynamic objects).  The camera pose is estimated based on the RANSAC on these matches.  Then the matches of the whole scene (before doing the RANSAC) are passed to the dynamic instance reconstruction process.  The system obtains the matches (before RANSAC) located on the dynamic objects based on the bounding box obtained from the semantic segmentation results.

e.g. The flow (after RANSAC) for estimating the camera pose

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/flow_camera_pose_pre.png)


I separated the sparse flow by only retaining the matches located on the static scenes.  Then, the camera pose is obtained by doing the RANSAC using these matches.  

e.g. (After RANSAC)

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/flow_camera_pose.png)


Raw flow on potential dynamic objects:

e.g. (Raw flow Before RANSAC) 

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/flow_object_before_RANSAC.png)


**Some ideas to improve the reconstruction**

Do the RANSAC on the flow of each object? 
Or change to the dense optical flow to get more matches for tracking the object.


**Notes**

- For the evaluation of the system, we need to considier the camera pose, the object pose (velocity, position...).

Ground truth of the dynamic objects? -> use some vedios for simulation? Or just see where they should be?

- For the improvement of the estimation of dynamic object poses, we may use the ground information (e.g. shadow) to enhance the tracking?

  Or use the bounding box which may represent the average motion of the dynamic objects to help the estimation?
  
- (Evaluation) Compare the results after removing matches on dynamic objects.
-----------------------------------------------------------------------------------------------------------------------
### 15 July
![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/semi-dense_1.jpeg)

![image](https://github.com/mounchiliu/IndividualProject/blob/master/image/semi-dense_2.jpeg)

------------------------------------------------------------------------------------------------------------------------



**Notes**

There are some proposed SLAM systems that deal with dynamic environments.
According to the paper (https://www.cs.ox.ac.uk/files/9926/Visual%20Slam.pdf), the problem of SLAM in dynamic environments can be viewed from two perspectives.

  1. As a robustness problem -> pose estimation in visual SLAM should remain accurate (segmenting the static and dynamic features in the image and regarding the dynamic parts as outliers.  Pose estimation is computed based on the static parts)
  
  2. Extending visual SLAM into dynamic environments -> (Do data association) system should be capable of segmenting the tracked features into different clusters, each associated with a different object.

(Note: systems like the ORBSLAM work reasonably well when most of the scene does not change.  They are based on the geometric consistency to remove the fewer points of dynamic objects in the system.) 

**The first perspective**

e.g. 

**DynaSLAM** uses Mask R-CNN to segment those classes that are potentially dynamic.
The camera is tracked using the static part of the image.  Dynamic objects are not used
for tracking and mapping.   The system also finds a way to complete the parts of the 3D map that is temporally occluded by a moving object.

**DSSLAM** adopts SegNet to provide pixel-wise semantic segmentation which helps us to filter out dynamic objects effectively.
The uses moving consistency check algorithm to determine whether the key points in segmentation results are moving.  For dynamic targets, remove all the ORB feature points that fall within the outline of the targets before matching.  Furthermore, dynamic targets will not be used for construct the map.

The approaches mentioned above both drop the key points on the dynamic objects and do the localization and mapping tasks without these points.  However, for this project, we consider to keep the points located on dynamic objects as well and use these points for tracking and reconstruction of dynamic objects.




**The second perspecctive**

**Notes for DynSLAM** 

**DynSLAM** presents a stereo-based dense mapping algorithm which is able to simultaneously reconstruct the static background, the moving objects and the potentially moving but currently stationary objects (e.g. parked cars) separately.

  1. Pre-process the input by computing a dense map, sparse scene flow and semantic segmentation of the RGB data.

  2. Compute VO from the sparse scene flow.

  3. Separate inputs (colour, depth, and sparse flow) into multiple frames: background & potentially dynamic object in the frame.

  4. Estimate the 3D motion of each new detection using the scene flow and semantic segmentation information, comparing it to the camera pose to classify each object as static, dynamic, or uncertain.



     How to estimate whether an object is dynamic or not?

     - For each segmented object, the masked scene flow associated with the specific object instance is used as input to estimate the motion of the camera w.r.t the object instance which is assumed to be static.  If the estimation is successful, then the 3D motion of the object is equal to the inverse of the camera motion.

     - For static objects, the resulting 3D object motion will be nearly identical to the camera's movement.


  5. For each rigid object of interest (moving or potentially moving), initialize or update its reconstruction.

  6. Update the static map reconstruction

   
     For the static map and individual object reconstruction, DynSLAM uses InfiniTAM for volumetric fusion.  DynSLAM separates the static background from the dynamic objects.  
     - The estimated vehicle movement (I suppose here it mentioned as camera pose) by the visual odometry is used to fuse the static parts of the input colour and depth maps, which are identified based on the instance-aware semantic segmentation component.
    
    
     - Both moving and potentially moving objects are reconstructed individually.  The estimated 3D motions of the individual objects are used for the object volumetric fusion.
  
  7. Perform voxel garbage collection to remove voxels allocated spuriously due to artifacts in the depth map.
  
  
  
  **Notes for SLAM with DATMO (Wang, 2007)** 
  
  1. SLAM with generalized objects calculates a joint posterior over all generated objects and the robot.  It contains a structure for motion modelling of generalized objects.  --> It is computationally demanding and infeasible.
  
  2. SLAM with DATMO decomposes the estimation problem into two separate estimators (posteriors for stationary objects and moving objects).  --> lower dimensional than SLAM with generalized objects.
  The system is able to deal with issues of perception modelling, data association, and moving objects detection.

Generally, a Bayesian formula was introduced to solve SLAM and DATMO.

   -  How does this system detect moving objects?
      Two approaches -> one for consistency based approach 
                        and one for motion object map based approach.
                        
      1. Consistency-based Detection
      
      detect moving objects -> combination of the result from segmentation and moving point detection for determining the dynamic objects   
      
      - Detect moving objects:    
        for a new scan of the local surrounding map, find the relative pose -> transform the local surrounding map tp the current coordinate system -> transform to a polar coordinate system -> detect moving points by comparing values along the range axis of the polar coordinate system.
        
      - Determine dynamic objects:      
        A segment is identified as a potential moving object if the ratio of the number of moving points to the number of total points is greater than 0.5.
        
        Problem: temporary stationary objects cannot be detected & If the time period between consecutive measurements is very short, the motion of moving objects will be too small to detect.
        e.g. Detection of pedestrians at very low speed is difficult to detect.
        
      2. Moving Object Map based Detection
      
      The system has a map that contains information from previous moving objects.  If a blob is in an area that was previously occupied by moving objects, this object can be considered as a potential moving object.

----------------------------------------------------------------------------------------------------------------------------
