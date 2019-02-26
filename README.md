# IndividualProject
Master Individual Project on Simultaneous Localization and Mapping (SLAM) in dynamic outdoor environment

## Records for things need to do and things have done

### 26th Feb
-------------------------------------------------------------------------------------------------------
**Need to do**

1.Have a look at DynSLAM (https://github.com/AndreiBarsan/DynSLAM)
  This system provides a dense SLAM system which attempts to reconstruct moving objects

2.Try cuda 8

3.Implement DynSLAM, try it on different datesets and evaluate it.  
  Have a try to test it with scenes in London which are quite different with scenes in KITTI dataset.
  (London has more tall buildings and cars may drives in a different way.)

4.Have a look at DATMO (https://www.ri.cmu.edu/pub_files/pub4/wang_chieh_chih_2007_1/wang_chieh_chih_2007_1.pdf) 
  which describes the problem of jointly doing SLAM and object tracking. 
  It is very detailed in the maths. The main thing to check at this stage 
  would be the introduction which states the problem
  
--------------------------------------------------------------------------------------------------------------------------  
**Notes**

There are some proposed SLAM systems that deal with dynamic environments.

e.g. 

**DynaSLAM** uses Mask R-CNN to segment thoese classes that are potentionally dynamic.
The camera is tracked using the static part of the image.  Dynamic objects are not used
for tracking and mapping.   The system also find a way to complete the parts of the 3D map 
that is temporally occluded by a moving objects.

**DSSLAM** adpots SegNet to provide pixel-wise semantic segmentation which helps us to fliter out dynamic objects effectively.
The uses moving consistency check algorithm to determine whether the keypoints in segmentation results are moving.  
For dynamic targets, remove all the ORB feature points that fall within the outline of the targets before matching.  
Furthermore, dynamic targets will not be used for construct the map.

The approaches mentioned above both drop the key points on the dynamic objects and do the localization and mapping tasks without these points.  However, for this project, we consider to keep the points located on dynamic objects as well and use these points for tracking and reconstruction of dynamic objects.


**Notes for DynSLAM** 




  
  


