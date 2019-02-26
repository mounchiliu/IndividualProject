# IndividualProject
Master Individual Project on Simultaneous Localization and Mapping (SLAM) in dynamic outdoor environment

## Records for things need to do and things have done

### 26th Feb
-------------------------------------------------------------------------------------------------------
**Need to do**

1.Have a look at DynSLAM (https://github.com/AndreiBarsan/DynSLAM)
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
**Notes**

There are some proposed SLAM systems that deal with dynamic environments.

e.g. 

**DynaSLAM** uses Mask R-CNN to segment those classes that are potentially dynamic.
The camera is tracked using the static part of the image.  Dynamic objects are not used
for tracking and mapping.   The system also finds a way to complete the parts of the 3D map that is temporally occluded by a moving object.

**DSSLAM** adopts SegNet to provide pixel-wise semantic segmentation which helps us to filter out dynamic objects effectively.
The uses moving consistency check algorithm to determine whether the key points in segmentation results are moving.  
For dynamic targets, remove all the ORB feature points that fall within the outline of the targets before matching.  
Furthermore, dynamic targets will not be used for construct the map.

The approaches mentioned above both drop the key points on the dynamic objects and do the localization and mapping tasks without these points.  However, for this project, we consider to keep the points located on dynamic objects as well and use these points for tracking and reconstruction of dynamic objects.


**Notes for DynSLAM** 

**DynSLAM** present a stereo-based dense mapping algorithm which is able to simultaneously reconstruct the static background, the moving objects and the potentially moving but currently stationary objects (e.g. parked cars) separately.

  1. Pre-process the input by computing a dense map, sparse scene flow and semantic segmentation of the RGB data.

  2. Compute VO from the sparse scene flow.

  3. Separate inputs (colour, depth, and sparse flow) into multiple frames: background & potentially dynamic object in the frame.

  4. Estimate the 3D motion of each new detection using the scene flow and semantic segmentation information, comparing it to the camera pose to classify each object as static, dynamic, or uncertain.



     How to estimate whether the object is dynamic or not?

     - For each segmented object, the masked scene flow associated with the specific object instance is used as input to estimate the motion of the camera w.r.t the object instance which is assumed to be static.  If the estimation is successful, then the 3D motion of the object is equal to the inverse of the camera motion.

     - For static objects, the resulting 3D object motion will be nearly identical to the camera's movement.


  5. For each rigid object of interest (moving or potentially moving), initialize or update its reconstruction.

  6. Update the static map reconstruction

   
     For the static map and individual object reconstruction, DynSLAM uses InfiniTAM for volumetric fusion.  DynSLAM separates the static background from the dynamic objects.  
     - The estimated vehicle movement (I suppose here it mentioned as camera pose) by the visual odometry is used to fuse the static parts of the input colour and depth maps, which are identified based on the instance-aware semantic segmentation component.
    
    
     - Both moving and potentially moving objects are reconstructed individually.  The estimated 3D motions of the individual objects are used for the object volumetric fusion.
  
  7. Perform voxel garbage collection to remove voxels allocated spuriously due to artifacts in the depth map.
  

