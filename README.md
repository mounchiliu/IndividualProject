# IndividualProject
Master Individual Project on Simultaneous Localization and Mapping (SLAM) in dynamic outdoor environment

## Records for things need to do and things have done

**26th Feb**

There are some proposed SLAM systems that deal with dynamic environments.

e.g. 
DynaSLAM uses Mask R-CNN to segment thoese classes that are potentionally dynamic.
The camera is tracked using the static part of the image.  Dynamic objects are not used
for tracking and mapping.

DSSLAM adpots SegNet to provide pixel-wise semantic segmentation.  


Need to
1.Have a look at DynSLAM (https://github.com/AndreiBarsan/DynSLAM)
  This system provides a dense SLAM system which attempts to reconstruct moving objects
2. try cuda 8
3.Implement it, try it on different dateset and evaluate it.  
  Have a try to test it with scenes in London which are quite different with KITTI dataset.
  (London has more tall buildings and cars may drives in a different way.)

4.Have a look at DATMO (https://www.ri.cmu.edu/pub_files/pub4/wang_chieh_chih_2007_1/wang_chieh_chih_2007_1.pdf) 
  which describes the problem of jointly doing SLAM and object tracking. 
  It is very detailed in the maths. The main thing to check at this stage 
  would be the introduction which states the problem


