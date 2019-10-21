#include "Track.h"
#include "InstanceTracker.h"
#include "../../libviso2/src/viso.h"

#include "../Opticalflow.h"




namespace instreclib {
namespace reconstruction {

using namespace std;
using namespace dynslam::utils;
using namespace instreclib::segmentation;
using namespace instreclib::utils;


 float Track::ScoreMatch(const TrackFrame& new_frame,  float& areascore) const {
  // TODO(andrei): Use fine mask, not just bounding box.
  // TODO(andrei): Ensure this is modular enough to allow many different matching strategies.
  // TODO-LOW(andrei): Take time into account---if I overlap perfectly but with a very old track,
  // the score should probably be discounted.

  assert(!this->frames_.empty() && "A track with no frames cannot exist.");
  const TrackFrame& latest_frame = this->frames_[this->frames_.size() - 1];

  // We don't want to accidentally add multiple segments from the same frame to the same track.
  // This is not 100% elegant, but it works.
  int delta_time = new_frame.frame_idx - this->GetEndTime();

  if (delta_time == 0) {
    return 0.0f;
  }

  //detectection for current frame
  const InstanceDetection& new_detection = new_frame.instance_view.GetInstanceDetection();
  //the last tracked frame for this instance
  const InstanceDetection& latest_detection = latest_frame.instance_view.GetInstanceDetection();

  // We don't want to associate segments from different classes.
  // TODO(andrei): Sometimes the segmentation pipeline may flicker between, e.g., ``car'' and
  // ``truck'' so we may want a more complex reasoning system here in the future.
  if (new_detection.class_id != latest_detection.class_id) {
    return 0.0f;
  }

  const BoundingBox& new_bbox = new_detection.GetCopyBoundingBox();
  const BoundingBox& last_bbox = latest_detection.GetCopyBoundingBox();

  //Added
  //Calculating and do the association

  const instreclib::utils::BoundingBox NewPredictBox = PredictBoundingbox(last_bbox);


  //--------------------------------------------------------------------------------------------------

  // Score the overlap using the standard intersection-over-union (IoU) measure.
  //Original:
  //int intersection = last_bbox.IntersectWith(NewPredictBox).GetArea();
  //Modified
  int intersection = new_bbox.IntersectWith(NewPredictBox).GetArea();

  //If there is no bounding box in the predicted position, we should create one
  //to compensate the miss detection


  int union_area = new_bbox.GetArea() + last_bbox.GetArea() - intersection;
  float area_score = static_cast<float>(intersection) / union_area;

  areascore =   static_cast<float>(intersection)/new_bbox.GetArea();







  // Modulate the score by the detection probability. If we see a good overlap but it's a dodgy
  // detection, we may not want to add it to the track. For instance, when using MNC for
  // segmentation, it may sometimes detect both part of a car and the entire car as separate
  // instances. Luckily, in these situations, the proper full detection gets a score near 1.0,
  // while the partial one is around 0.5-0.6. We'd prefer to fuse in the entire car, so we take
  // the probability into account. Similarly, in a future frame, we prefer adding the new data to
  // the track with the most confident detections.
  float score = area_score * new_detection.class_probability * latest_detection.class_probability;


  float time_discount = 1.0f;
  // 1 = no gap in the track
  if (delta_time == 2) {
    time_discount = 0.5f;
  }
  else if (delta_time > 2) {
    time_discount = 0.25;
  }

  return score * time_discount;
}

string Track::GetAsciiArt() const {
  stringstream out;
  out << "Object #" << setw(4) << id_ << " [";
  int idx = 0;
  for (const TrackFrame& frame : frames_) {
    while (idx < frame.frame_idx) {
      out << "   ";
      ++idx;
    }
    out << setw(3) << frame.frame_idx;
    ++idx;
  }
  out << "]";

  return out.str();
}

/// \brief Returns the relative pose of the specified frame w.r.t. the first one.
Option<Eigen::Matrix4d> Track::GetFramePose(size_t frame_idx) const {
  assert(frame_idx < GetFrames().size() && "Cannot get the relative pose of a non-existent frame.");

  // Skip the original very distant frames with no relative pose info.
  bool found_good_pose = false;
  Eigen::Matrix4d *pose = new Eigen::Matrix4d;
  pose->setIdentity();

  // Start from 1 since we care about relative pose to 1st frame.
  for (size_t i = 1; i <= frame_idx; ++i) {
    if (frames_[i].relative_pose->IsPresent()) {
      found_good_pose = true;
      const Eigen::Matrix4d &rel_pose = frames_[i].relative_pose->Get().matrix_form; //relative_pose -> motion_delta.
      *pose = rel_pose * (*pose);
    }
    else {
      // Gap caused by instances switching (static/dynamic) -> uncertain -> (static/dynamic).
      if (found_good_pose) {
        cout << "(static/dynamic) -> uncertain -> (static/dynamic) case detected; ignoring first "
            << "reconstruction attempt." << endl;
        found_good_pose = false;
        pose->setIdentity();
      }
    }
  }

  return Option<Eigen::Matrix4d>(pose);
}


dynslam::utils::Option<Eigen::Matrix4d> Track::GetFramePoseDeprecated(size_t frame_idx) const {
  assert(frame_idx < GetFrames().size() && "Cannot get the relative pose of a non-existent frame.");

  bool found_good_pose = false;
  Eigen::Matrix4d *pose = new Eigen::Matrix4d;
  pose->setIdentity();

  Eigen::Matrix4d first_good_cam_pose;
  first_good_cam_pose.setIdentity();

  for (size_t i = 1; i <= frame_idx; ++i) {
    if (frames_[i].relative_pose_world->IsPresent()) {
      if (! found_good_pose) {
        first_good_cam_pose = frames_[i].camera_pose.cast<double>();
        found_good_pose = true;
      }

      const Eigen::Matrix4d &rel_pose = frames_[i].relative_pose_world->Get().cast<double>();
      const Eigen::Matrix4d new_pose = rel_pose * (*pose);

      *pose = new_pose;
    }
    else {
      if (found_good_pose) {
        // This is OK even if the previos "streak" had triggered a reconstruction, since as soon
        // as we detect an resumed reconstruction after an interruption, we clear the reconstruction
        // volume.
        found_good_pose = false;
        pose->setIdentity();
      }
    }
  }

  if (track_state_ == TrackState::kStatic) {
    return dynslam::utils::Option<Eigen::Matrix4d>(new Eigen::Matrix4d(first_good_cam_pose));
  }

  if (found_good_pose) {
    Eigen::Matrix4d aux = first_good_cam_pose * *pose; //find the first camera frame that observe the pose.
    *pose = aux;
    return dynslam::utils::Option<Eigen::Matrix4d>(pose);
  }
  else {
    return dynslam::utils::Option<Eigen::Matrix4d>::Empty();
  }
}

Option<Pose>* Track::EstimateInstanceMotion(
    const vector<RawFlow, Eigen::aligned_allocator<RawFlow>> &instance_raw_flow,
    const SparseSFProvider &ssf_provider,
    const vector<double> &initial_estimate,
    vector<Eigen::Vector3d> normals
) {
  // This is a good conservative value, but we can definitely do better.
  // TODO(andrei): Try setting the minimum to 6-10, but threshold based on the final RMSE, flagging
  // pose estimates whose residual is above some value as invalid.
  // Note: 25 => OK results in most cases, but where cars are advancing from opposite direction
  //             in the hill sequence, this no longer works.
  uint32_t kMinFlowVectorsForPoseEst;
  if(normals.empty()){
    kMinFlowVectorsForPoseEst = 18; //For sparse
                                    //35 for testing 10 sequence
                                    //18
  }
  else{
      kMinFlowVectorsForPoseEst = 18;
  }
//  uint32_t kMinFlowVectorsForPoseEst = 18;
  // technically 3 should be enough (because they're stereo-and-time 4-way correspondences, but
  // we're being a little paranoid).
  size_t flow_count = instance_raw_flow.size();

  if (instance_raw_flow.size() >= kMinFlowVectorsForPoseEst) {
    //Get estimated instance motion.

     vector<double> instance_motion_delta;
    if(!normals.empty())
        instance_motion_delta= ssf_provider.ExtractMotion(instance_raw_flow, initial_estimate,normals);//initial_estimate for previous frame's relative pose
    else {
        instance_motion_delta = ssf_provider.ExtractMotion(instance_raw_flow, initial_estimate);//initial_estimate for previous frame's relative pose
    }
    if (instance_motion_delta.size() != 6) {
      // track information not available yet; idea: we could move this computation into the
      // track object, and use data from many more frames (if available).
      cerr << "Could not compute instance #" << GetId() << " delta motion from " << flow_count << " matches." << endl;
      return new Option<Pose>;
    } else {
      cout << "Successfully estimated the relative instance pose from " << flow_count
           << " matches." << endl;
      // This is a libviso2 matrix.
      Matrix delta_mx = VisualOdometry::transformationVectorToMatrix(instance_motion_delta);

      // We then invert it and convert it into an Eigen matrix.
      // TODO(andrei): Make this a utility.
      return new Option<Pose>(new Pose(
          instance_motion_delta,
          Eigen::Matrix4d((~delta_mx).val[0])
      ));
    }
  }
  else {
    cout << "Only " << flow_count << " scene flow points. Not estimating relative pose for track "
         << "#" << GetId() << "." << endl;
    return new Option<Pose>();
  }
}



void Track::Update(const Eigen::Matrix4f &egomotion,  const Eigen::Matrix4f &my_camera_pose,
                   const instreclib::SparseSFProvider &ssf_provider,
                   bool verbose,
                   vector<Eigen::Vector3d> normals) {




  long prev_frame_idx = static_cast<long>(frames_.size()) - 2;
  vector<double> initial_estimate = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  if (prev_frame_idx >= 0) {
    auto prev_pose = frames_[prev_frame_idx].relative_pose;
    if (prev_pose->IsPresent()) //not nullprt
    {
      // Perform warm start is previous relative pose is known.
      initial_estimate = prev_pose->Get().se3_form;
    }
  }


  // Vehicle motion INCLUDING camera egomotion.
  Option<Pose> *motion_delta = EstimateInstanceMotion(
      GetLastFrame().instance_view.GetFlow(),
      ssf_provider,
      initial_estimate
  );

  //did not get valid estimation on the motion
  if(!motion_delta->IsPresent() ){
{
      if(MyGreyFrames_.size()>1){

          //Get current bounding box
          auto box_cur = GetLastFrame().instance_view.GetInstanceDetection().GetCopyBoundingBox();
          //Get previous bounding box
          auto box_pre = frames_[frames_.size()-2].instance_view.GetInstanceDetection().GetCopyBoundingBox();



          //Get current image / previous images
          cv::Mat imgleft_cur = (MyGreyFrames_.back().first);
          cv::Mat imgright_cur = (MyGreyFrames_.back().second);
          cv::Mat imgleft_pre = MyGreyFrames_[MyGreyFrames_.size()-2].first;
          cv::Mat imgright_pre = MyGreyFrames_[MyGreyFrames_.size()-2].second;


//          //Does not need
//          cv::Mat imgleft_cur_hsv = (MyHsvFrames_.back().first);
//          cv::Mat imgright_cur_hsv = (MyHsvFrames_.back().second);
//          cv::Mat imgleft_pre_hsv = MyHsvFrames_[MyHsvFrames_.size()-2].first;
//          cv::Mat imgright_pre_hsv = MyHsvFrames_[MyHsvFrames_.size()-2].second;





          //**************************************************************************************************************************************
          //1. Now, time for optical flow method
          vector<RawFlow, Eigen::aligned_allocator<RawFlow>> flow_update = GetLastFrame().instance_view.GetFlow();
          //if(flow_update.empty()){
              cout<<"Start to estimate using semi-dense method for track# "<< id_ <<endl;
              while(1){
                  vector<cv::Point2f> pt_l_c;
                  vector<cv::Point2f> pt_l_c_more;
                  //Initialize points
                  //Shink the bounding box
                  //to do the pose estimation (no)
                  for(int i = box_cur.r.x0+1; i <= box_cur.r.x1-1; i= i+1){
                      for (int j = box_cur.r.y0+1; j <= box_cur.r.y1-1; j= j+1) {
                          //Only keep points where there is a salient gradient in pixel values
                          Eigen::Vector2d delta(
                                      imgleft_cur.ptr<uchar>(j)[i+1] - imgleft_cur.ptr<uchar>(j)[i-1],//gradient in x
                                  imgleft_cur.ptr<uchar>(j+1)[i] - imgleft_cur.ptr<uchar>(j-1)[i]);//gradient in y
                          if(delta.norm()<100)// -> lower gradient threshold //100 ok 60 ok
                              continue;
                          if(delta.norm()<=150){// -> higher threshold //150 //140
                              pt_l_c_more.push_back(cv::Point2f(i,j));
                              continue;
                          }


                          cv::Point2f pt(i,j);
                          pt_l_c.push_back(pt);
                      }
                  }




                  vector<cv::Point2f> pt_l_p, pt_r_c, pt_r_p, kp1_new;
                  //vector<uchar> status1,status2, status3,status4;
                  vector<bool> status1,status2, status3,status4;

                  vector<bool> ok(pt_l_c.size(),true);
                  //vector<float> error;
                  if(!pt_l_c.empty()){
                      //cv::calcOpticalFlowPyrLK(*imgleft_cur, *imgright_cur, pt_l_c, pt_r_c, status1, error, cv::Size(8, 8));
                      //cv::calcOpticalFlowPyrLK(*imgleft_cur, *imgleft_pre, pt_l_c, pt_l_p, status2, error, cv::Size(8, 8));
                      //cv::calcOpticalFlowPyrLK(*imgleft_pre, *imgright_pre, pt_l_p, pt_r_p, status3, error, cv::Size(8, 8));

                      //cv::calcOpticalFlowPyrLK(*imgleft_cur, *imgleft_pre, pt_l_c, pt_l_p, status, error, cv::Size(8, 8));
                      //cv::calcOpticalFlowPyrLK(*imgleft_pre, *imgright_pre, pt_l_p, pt_r_p, status, error, cv::Size(8, 8));
                      //cv::calcOpticalFlowPyrLK(*imgright_pre,  *imgright_cur, pt_r_p, pt_r_c, status, error, cv::Size(8, 8));
                      //cv::calcOpticalFlowPyrLK(*imgright_cur, *imgleft_cur, pt_r_c, kp1_new, status, error, cv::Size(8, 8));

                      //Use bounding box to help do the optical flow
                      float dx = (box_cur.r.x0-box_pre.r.x0);
                      float dy = (box_cur.r.y0-box_pre.r.y0);


                      //tricky
                      //This for condition that they only get part of segmentation of object.
                      //One vertex of bounding box seems to keep on its place
                      if(abs(dx)<=5){
                          float dx2 = (box_cur.r.x1-box_pre.r.x1);
                          if(abs(dx2)>=5){
                              dx = dx2;
                          }
                      }

                      //Test
                      //dynslam::utils::Tic("testtesttest!");
                      Opticalflow::OpticalFlowMultiLevel(imgleft_cur, imgleft_pre, pt_l_c, pt_l_p, status1,true,dx);
                      Opticalflow::OpticalFlowMultiLevel(imgleft_pre, imgright_pre, pt_l_p, pt_r_p, status2,true);
                      Opticalflow::OpticalFlowMultiLevel(imgright_pre,  imgright_cur, pt_r_p, pt_r_c, status3,true,-dx);
                      Opticalflow::OpticalFlowMultiLevel(imgright_cur, imgleft_cur, pt_r_c, kp1_new, status4,true);
                      //dynslam::utils::Toc();



                      //Search valid matches
                      int count = FindMatches(ok,pt_l_c, kp1_new,pt_l_p,dx,dy);

                      //constraints
                      //dy for left and right should be allmost the same
                      //dy should be almost 0
                      for(size_t i = 0; i < pt_l_c.size(); i++){
                          if(ok[i]){
                              if(abs(pt_r_c[i].y - pt_l_c[i].y)>=5){
                                  ok[i] = false;
                                  count = count - 1;
                              }
                              if(abs(pt_r_p[i].y - pt_l_p[i].y)>=5){
                                  ok[i] = false;
                                  count = count - 1;
                              }
                          }
                      }


                      if(count<=30){//30

                          //match points with lower gradient threshold
                          vector<cv::Point2f> kp1_new_more, pt_l_p_more,pt_r_p_more,pt_r_c_more;
                          vector<bool> status1_more,status2_more, status3_more,status4_more;
                          Opticalflow::OpticalFlowMultiLevel(imgleft_cur, imgleft_pre, pt_l_c_more, pt_l_p_more, status1,true,dx);
                          Opticalflow::OpticalFlowMultiLevel(imgleft_pre, imgright_pre, pt_l_p_more, pt_r_p_more, status2,true);
                          Opticalflow::OpticalFlowMultiLevel(imgright_pre, imgright_cur, pt_r_p_more, pt_r_c_more, status3,true,-dx);
                          Opticalflow::OpticalFlowMultiLevel(imgright_cur, imgleft_cur, pt_r_c_more, kp1_new_more, status4,true);


                          vector<bool> ok2(pt_l_c_more.size(),true);


                          //Search valid matches
                          int count2 = FindMatches(ok2,pt_l_c_more, kp1_new_more,pt_l_p_more,dx,dy,count);

                          //constraints
                          //dy for left and right should be allmost the same
                          //dy should be almost 0
                          for(size_t i = 0; i < pt_l_c_more.size(); i++){
                              if(ok2[i]){
                                  if(abs(pt_r_c_more[i].y - pt_l_c_more[i].y)>=5){
                                      ok2[i] = false;
                                      count2 = count2 -1;
                                  }
                                  if(abs(pt_r_p_more[i].y - pt_l_p_more[i].y)>=5){
                                      ok2[i] = false;
                                      count2 = count2 -1;
                                  }
                              }
                          }



                          if(count2>=35){
                              //Update
                              ok.insert(ok.end(),ok2.begin(),ok2.end());
                              pt_l_c.insert(pt_l_c.end(),pt_l_c_more.begin(),pt_l_c_more.end());
                              pt_l_p.insert(pt_l_p.end(),pt_l_p_more.begin(),pt_l_p_more.end());
                              pt_r_p.insert(pt_r_p.end(),pt_r_p_more.begin(),pt_r_p_more.end());
                              pt_r_c.insert(pt_r_c.end(),pt_r_c_more.begin(),pt_r_c_more.end());

                              //Debug
                              //status1.insert(status1.end(),status1_more.begin(),status1_more.end());
                              //status2.insert(status2.end(),status2_more.begin(),status2_more.end());
                              //status3.insert(status3.end(),status3_more.begin(),status3_more.end());
                              //status4.insert(status4.end(),status4_more.begin(),status4_more.end());


                              kp1_new.insert(kp1_new.end(),kp1_new_more.begin(),kp1_new_more.end());

                          }else {//wont get a pose estimation since no enough matches
                              cout<<"cant do semi dense optical flow because of insufficient matches. We only have "<<count2<<" flows"<<endl;
                              break;
                          }

                      }


//                      //Debug
//                      //Draw raw matches
//                      if(id_==0){
//                          cv::Mat img2_CV = MyGreyFrames_.back().first;
//                          cv::cvtColor(img2_CV, img2_CV, CV_GRAY2BGR);
//                          //cv::cvtColor(*imgleft_pre, img1_pre, CV_GRAY2BGR);
//                         // cv::rectangle(img2_CV,cv::Point2f(box_cur.r.x0,box_cur.r.y0),
//                                        //cv::Point2f(box_cur.r.x1, box_cur.r.y1),cv::Scalar(250,0,0),2);
//                          for (int i = 0; i < pt_l_c.size(); i++) {
//                              if ( ok[i]) {

//                                  // if (pt_l_c[i].y-pt_l_p[i].y>=5*dx || pt_l_c[i].y-pt_l_p[i].y <= 0.2*dx){
//                                        //cv::line(img2_CV, pt_l_c[i], pt_l_p[i], cv::Scalar(0, 250, 0));
//                                   //}else{
//                                       //cv::line(img2_CV, pt_l_c[i], kp1_new[i], cv::Scalar(0, 250, 0));
//                                       //cout<<dx<<" "<<pt_l_c[i].x-pt_l_p[i].x<<" "<<dy<<" "<<pt_l_c[i].y-pt_l_p[i].y<<endl;

//                                   //}
//                                  //cv::circle(img1_pre, pt_l_p[i], 1, cv::Scalar(250, 250, 0), 1);
//                                  //cv::circle(img2_CV, pt_l_c[i], 1, cv::Scalar(250, 250, 0), 1);
//                                  cv::line(img2_CV, pt_l_c[i], pt_l_p[i], cv::Scalar(250, 250, 0));
//                                  //cv::line(img1_pre, pt_l_p[i], kp1_new[i], cv::Scalar(0, 250, 0));
//                              }
//                          }
//                          cv::imwrite("semi-correspondences.png",img2_CV);
//                          cv::imshow("Raw_cur", img2_CV);
//                          //cv::imshow("Raw_pre", img1_pre);
//                          cv::waitKey(0);
//                          //End drawing
//                      }//End debug


                      //flow_update.clear();
                      for (int i = 0; i < pt_l_c.size(); i++) {
                          if (ok[i]){
                              flow_update.emplace_back(pt_l_c[i].x,pt_l_c[i].y, i,
                                                       pt_r_c[i].x,pt_r_c[i].y, i,
                                                       pt_l_p[i].x,pt_l_p[i].y, i,
                                                       pt_r_p[i].x,pt_r_p[i].y, i
                                                       );
                          }
                      }

                      GetLastFrame().instance_view.UpdateInstanceFlow(flow_update);

                      //Used for debug
                      //test = GetLastFrame().instance_view.GetFlow();


                      motion_delta = EstimateInstanceMotion(
                                  GetLastFrame().instance_view.GetFlow(),
                                  ssf_provider,
                                  initial_estimate,
                                  normals
                                  );

                  }


                  if(motion_delta->IsPresent()){
                      cout<<"Successfully estimate motion using semi-dense method for track# "<< id_ <<endl;
                      break;
                  }else {
                      cout<<"Still can not estimate motion using semi-dense method for track# "<< id_ <<endl;
                      break;

                  }


              }//end while

 //         }      //End of optical flow


//Template matching
{
          if(!motion_delta->IsPresent())
          {
              //******************************************************************************************
              //2. find matches using template matching
/*              //test
              cv::Mat templ = (imgleft_cur_hsv)(Range(max(box_cur.r.y0,0),min(box_cur.r.y1,imgleft_cur_hsv.rows)),
                                             Range(max(box_cur.r.x0,0),min(box_cur.r.x1,imgleft_cur_hsv.cols)));


              cout<<"Start to estimate using template matching method for track# "<< id_ <<endl;
              //Initialize result Mat
              int result_cols = imgleft_cur.cols;
              int result_rows = imgleft_cur.rows;
              cv::Mat result;
              result.create( result_rows, result_cols, CV_32FC1 );

              //Do template matching within a circle
              //Template matching
              matchTemplate( imgleft_pre_hsv , templ, result, 5);//match_method = TM COEFF NORMED
              //Find max and min
              double minVal; double maxVal; Point minLoc; Point maxLoc;
              Point matchLoc;
              minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
              matchLoc = maxLoc;

              ;
              cv::Mat temp2 = (imgleft_pre_hsv)(Range(max(matchLoc.y,0),min(matchLoc.y + templ.rows,imgleft_cur_hsv.rows)),
                                             Range(max(matchLoc.x,0),min(matchLoc.x + templ.cols,imgleft_cur_hsv.cols)));


              //int th = 10;
              //check
              //check whether template matching is successful? //Give a tolerance
              if(box_pre.r.x0 - 3 <= matchLoc.x && box_pre.r.x1 + 3 >= matchLoc.x + templ.cols&&
                 box_pre.r.y0 - 3 <= matchLoc.y){
                    template_ok = true;
              }



              flow_update.clear();
              //Do the following matching
              if(template_ok){
                  cout<<"Do the rest of template matching"<<endl;
                  flow_update = templateMatching(imgleft_cur_hsv,imgleft_pre_hsv,imgright_cur_hsv,imgright_pre_hsv,
                                                 matchLoc,box_cur,temp2);
              }
              else {
                  //In case the object get ocluded which will influence the result of template matching
                  //With the aid of intensity histogram to solve the problem
                  //Calculate histogram on template
                  //1.
                  int histSize = 256; //h-30 s-32
                  Mat hist,hist2;
                  float range[] = { 0, 256 }; //the upper boundary is exclusive  h-0,180 s-0,256
                  const float* histRange = { range };
                  bool uniform = true, accumulate = false;

                  cv::Mat backproj;

                  int th_ = 15;

                  if(box_pre.r.y0+th_>=box_pre.r.y1-th_||box_pre.r.x0+th_>=box_pre.r.x1-th_){
                      th_ = 0;
                  }
                  Mat templ2 = imgleft_pre_hsv(Range(max(box_pre.r.y0+th_,0),min(box_pre.r.y1-th_,imgleft_pre_hsv.rows)),
                                               Range(max(box_pre.r.x0+th_,0),min(box_pre.r.x1-th_,imgleft_pre_hsv.cols)));

                  calcHist( &templ2, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

//                  if(templ.rows>2*th_ && templ.cols>2*th_){
//                      Mat templ3 = templ(Range(th_,templ.rows-th_),
//                                      Range(th_,templ.cols-th_));

//                      calcHist( &templ3, 1, 0, Mat(), hist2, 1, &histSize, &histRange, uniform, accumulate);

//                      hist = hist + hist2;

//                  }
                  calcBackProject( &templ2, 1, 0, hist, backproj, &histRange, 1, true );
                  int th_intensity =80;
                  //x02 y02 x12 y12 in case there is occlusion
                  int x0 = backproj.cols, y0 = backproj.rows;
                  int x1 = 0, y1 = 0;
                  for(int i=1; i<backproj.rows-1; i++){
                      for(int j=1; j<backproj.cols-1; j++){
                          if(int(backproj.at<uchar>(i,j))<=th_intensity){

                             backproj.at<uchar>(i,j) = 0;

                          }else {
                              //Check the nearear neighbouts
                             //col
                              if(int(backproj.at<uchar>(i,max(j-1,0)))<=th_intensity &&
                                 int(backproj.at<uchar>(i,min(j+1,backproj.cols-1))<=th_intensity)){
                                  backproj.at<uchar>(i,j) = 0;
                              }
                              else if (int(backproj.at<uchar>(i,max(j-1,0)))>=th_intensity &&
                                       int(backproj.at<uchar>(i,min(j+1,backproj.cols-1))>=th_intensity)){

                                  if(j<x0){
                                      x0 = j;
                                  }
                                  if(j>x1){
                                      x1 = j;
                                  }
                              }
                          }
                      }
                  }

                  //imshow("back",backproj);


                  //If get valid record

                  if( x0!=backproj.cols && x1!=0){

                    Mat temp = backproj(Range(0,backproj.rows),Range(x0,x1));

                    vector<int> record(temp.cols,0);
                    for(int i=1; i<temp.rows-1; i++){
                        for(int j=1; j<temp.cols-1; j++){
                            if(int(temp.at<uchar>(i,j-5))!=0 &&
                               int(temp.at<uchar>(i,j))!=0     &&
                               int(temp.at<uchar>(i,j+5))!=0 ){

                               record[j]+=1;

                        }
                        }
                    }


                    //Get part of the object
                    int diff_x = distance(record.begin(), max_element(record.begin(),record.end()));

                    //cv::imshow("backproj",temp(Range(0,temp.rows),Range(0,distance(record.begin(), max_element(record.begin(),record.end())))));
                    //Use this to adjust template and get part of the object
                    //Compare diff in x0 and x1
                    float comp = abs(x0-th_) / abs(diff_x+x0-(templ2.cols-th_));
                    //comp << 1 chose the part from left to right
                    auto box_temp = box_cur;
                    auto box_temp_pre = box_pre;
                    if(comp<0.1&&diff_x!=0){
                        //templ = templ(Range(0,templ.rows),Range(0,diff_x));
                        templ = (imgleft_cur_hsv)(Range(max(box_cur.r.y0,0),min(box_cur.r.y1,imgleft_cur_hsv.rows)),
                                                  Range(max(box_cur.r.x0,0),min(max(box_cur.r.x0,0)+diff_x,imgleft_cur_hsv.cols)));

                        box_temp.r.x1 = box_cur.r.x0 + diff_x;
                        box_temp_pre.r.x1 = box_pre.r.x0 + diff_x;
                    }else if(diff_x!=0){
                        //templ = templ(Range(0,templ.rows),Range(templ.cols-diff_x,templ.cols));
                        templ = (imgleft_cur_hsv)(Range(max(box_cur.r.y0,0),min(box_cur.r.y1,imgleft_cur_hsv.rows)),
                                                  Range(max(min(box_cur.r.x1,imgleft_cur_hsv.cols)-diff_x,0),min(box_cur.r.x1,imgleft_cur_hsv.cols)));
                        box_temp.r.x0 = box_cur.r.x1 - diff_x;
                        box_temp_pre.r.x0 = box_pre.r.x1 - diff_x;
                    }



                    cout<<"Start to estimate using part of template matching method for track# "<< id_ <<endl;

                    //Do template matching within a circle
                    //Template matching
                    matchTemplate( imgleft_pre_hsv , templ, result, 5);//match_method = TM COEFF NORMED
                    //Find max and min
                    minMaxLoc( result, &minVal, 0, &minLoc, 0, Mat() );
                    matchLoc = maxLoc;
                    temp2 = (imgleft_pre_hsv)(Range(max(matchLoc.y,0),min(matchLoc.y + templ.rows,imgleft_cur_hsv.rows)),
                                              Range(max(matchLoc.x,0),min(matchLoc.x + templ.cols,imgleft_cur_hsv.cols)));
                    //cv::imshow("...2",temp2);

                    //check
                    //check whether template matching is successful? //Give a tolerance
                    if(box_temp_pre.r.x0 - 5 <= matchLoc.x && box_temp_pre.r.x1 + 5 >= matchLoc.x + templ.cols&&
                       box_temp_pre.r.y0 - 3 <= matchLoc.y) {
                        cout<<"rest of template matching with part of object"<<endl;
                        template_ok = true;
                    }



                    //Do the following matching
                    if(template_ok){
                        flow_update = templateMatching(imgleft_cur_hsv,imgleft_pre_hsv,imgright_cur_hsv,imgright_pre_hsv,
                                                       matchLoc,box_temp,temp2);
                    }

                  }

              }

              if(!flow_update.empty()){
                  GetLastFrame().instance_view.UpdateInstanceFlow(flow_update);

                  //Used for debug
                  //test = GetLastFrame().instance_view.GetFlow();

                  motion_delta = EstimateInstanceMotion(
                              GetLastFrame().instance_view.GetFlow(),
                              ssf_provider,
                              initial_estimate,
                              normals
                              );


                  if(motion_delta->IsPresent()){
                       cout<<"Successfully estimate motion using template matching method for track# "<< id_ <<endl;
                   }else {
                       cout<<"Still can not estimate motion using template matching method for track# "<< id_ <<endl;
                   }
              }else{
                  cout<<"Still can not estimate motion using template matching method for track# "<< id_ <<" because of insufficient flow"<<endl;
              }
                //End of template matching-1*/

              if(!motion_delta->IsPresent()){
                  vector<cv::Point2f> pt_l_c;
                  for(int i = box_cur.r.x0+1; i < box_cur.r.x1-1; i= i+1){
                      for (int j = box_cur.r.y0+1; j < box_cur.r.y1-1; j= j+1) {
                          //Only keep points where there is a salient gradient in pixel values
                          Eigen::Vector2d delta(
                                  imgleft_cur.ptr<uchar>(j)[i+1] - imgleft_cur.ptr<uchar>(j)[i-1],//gradient in x
                                  imgleft_cur.ptr<uchar>(j+1)[i] - imgleft_cur.ptr<uchar>(j-1)[i]);//gradient in y
                          if(delta.norm()<100)//80 //[100] //120
                              continue;

                          cv::Point2f pt(i,j);
                          pt_l_c.push_back(pt);
                      }
                  }

                  //********************************
                  //Debug
                  //Mat temp = imgleft_cur.clone();
                  //Mat temp2 = imgleft_pre.clone();
                  //int count = 0;
                  //********************************

                  //Initialize
                  vector<cv::Point2f> pt_l_p, pt_r_c, pt_r_p;
                  //Reserve size
                  pt_l_p.reserve(pt_l_c.size()); pt_r_c.reserve(pt_l_c.size()); pt_r_p.reserve(pt_l_c.size());
                  vector<bool> ok (pt_l_c.size(),false);
                  Mat result;
                  double minVal; double maxVal; Point minLoc; Point maxLoc;
                  Point matchLoc;
                  float intensity_diff_th = 1000;

                  int r = 10; //Define the area of patch for template matching //15//10
                  if(box_cur.r.y1-box_cur.r.y0<=2*r || box_cur.r.x1-box_cur.r.x0<=2*r){
                      r = min(box_cur.r.y1-box_cur.r.y0,box_cur.r.x1-box_cur.r.x0)/10.0-2;
                      if(r <= 0){
                          r = 2;
                      }else{
                          while(r<=5){
                              r = r*2.0;
                          }
                      }
                  }

                  cv::Mat l_c_box = imgleft_cur(Range(max(0,box_cur.r.y0),min(box_cur.r.y1,imgleft_cur.rows)),//y0, y1
                                                Range(max(0,box_cur.r.x0),min(box_cur.r.x1,imgleft_cur.cols)));//x0, x1
                  //cordinates of point(x,y) inside this area in original image = (x +max(0,box_cur.r.x0),y+max(0,box_cur.r.y0))
                  cv::Mat l_p_box = imgleft_pre(Range(max(0,box_pre.r.y0),min(box_pre.r.y1,imgleft_pre.rows)),//y0, y1
                                                Range(max(0,box_pre.r.x0),min(box_pre.r.x1,imgleft_pre.cols)));//x0, x1

                  float dx = (box_cur.r.x0-box_pre.r.x0);
                  float dy = (box_cur.r.y0-box_pre.r.y0);

                  //tricky
                  //This for condition that they only get part of segmentation of object.
                  //One vertex of bounding box seems to keep on its place
                  if(abs(dx)<=5){
                      float dx2 = (box_cur.r.x1-box_pre.r.x1);
                      if(abs(dx2)>=5){
                          dx = dx2;
                      }
                  }

                  cout<<"Estimate using template matching with each pixel."<<endl;

                  //Template matching between left_cur and left_pre
                  for(int i = 0; i<pt_l_c.size(); i++){

                      if(pt_l_c[i].y-r>=0 && pt_l_c[i].x-r>=0 &&
                         pt_l_c[i].y+r<=imgleft_cur.rows && pt_l_c[i].x+r<=imgleft_cur.cols){
                          //******************************************************
                          //Debug
                          //circle(imgleft_cur, pt_l_c[i],2,Scalar::all((0)));
                          //circle(l_c_box, (pt_l_c[i]-Point2f(max(0,box_cur.r.x0),max(0,box_cur.r.y0))),2,Scalar::all((250)));
                          //******************************************************
                          cv::Mat t = imgleft_cur(Range(pt_l_c[i].y-r,pt_l_c[i].y+r),Range(pt_l_c[i].x-r,pt_l_c[i].x+r));
                          matchTemplate(l_p_box, t, result, 5);
                          minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
                          matchLoc = maxLoc;
                          //Translate to the position in previous left images
                          Point2f pt = Point2f(max(0,box_pre.r.x0)+matchLoc.x + float(t.cols)/2.0,
                                               max(0,box_pre.r.y0)+matchLoc.y + float(t.rows)/2.0);

                          pt_l_p[i] = pt;
                          //*********************************************
                          //Point2f pt2 = Point2f(matchLoc.x + float(t.cols)/2.0,
                                                //matchLoc.y + float(t.rows)/2.0);
                          //Debug
                          //circle(l_p_box, pt2,2,Scalar::all((250)));
                          //circle(imgleft_pre, pt,2,Scalar::all((250)));
                          //circle(imgleft_cur, pt_l_c[i],2,Scalar::all((250)));
                          //**********************************************


                          //check intensity diff
                          if(pow((GetPixelValue(imgleft_cur,pt_l_c[i].x,pt_l_c[i].y) -
                                 GetPixelValue(imgleft_pre,pt.x,pt.y)),2) <= intensity_diff_th){

                              ok[i] = true;
                              //We need to solve the problem of occlusion and avoid outliers
                              //Method 2: Or try the color histogram
                              //Method 1:
                              //Use bounding box to help do the match
                              //Use bounding box to constrain the matches
                              if(abs(pt_l_c[i].y-pt_l_p[i].y)>10){ //use dy constrain
                                  if (abs(pt_l_c[i].x-pt_l_p[i].x)>abs(2*dx) || abs(pt_l_c[i].x-pt_l_p[i].x) < abs(0.5*dx)||
                                      abs(pt_l_c[i].y-pt_l_p[i].y)>abs(2*dy) || abs(pt_l_c[i].y-pt_l_p[i].y) < abs(0.5*dy) ||
                                          (pt_l_c[i].y-pt_l_p[i].y)*dy < -10){//wrong matches
                                        ok[i] = false;
                                  }
                              }else{ //Only use dx constrain
                                  if (abs(pt_l_c[i].x-pt_l_p[i].x)>abs(2*dx) || abs(pt_l_c[i].x-pt_l_p[i].x) < abs(0.5*dx)||
                                          (pt_l_c[i].y-pt_l_p[i].y)*dy < -10){
                                      ok[i] = false;
                                  }
                              }

                              if(ok[i]){
                                  //circle(temp2, pt,2,Scalar::all((250)));
                                  //circle(temp, pt_l_c[i],2,Scalar::all((250)));
                                  //line(temp,pt_l_c[i],pt_l_p[i],Scalar::all(250),1);
                              }    //End of if Debug
                          }
                      }
                  }

                  //Template matching between left_pre and right_pre
                  //We dont have bounding box for right images, so we need to expand the left bounding box
                  //Or we can do template matching using left bouding box with right images.
                  int expand_th = 20;
                  cv::Mat r_p_box = imgright_pre(Range(max(0,box_pre.r.y0),
                                                       min(box_pre.r.y1,imgright_pre.rows)),//y0, y1
                                                 Range(max(0,box_pre.r.x0-expand_th),
                                                       min(box_pre.r.x1,imgright_pre.cols)));//x0, x1

                  for(int i = 0; i<pt_l_c.size(); i++){
                      if(ok[i]){
                          if(pt_l_p[i].y-r>=0 && pt_l_p[i].x-r>=0 &&
                             pt_l_p[i].y+r<=imgleft_pre.rows && pt_l_p[i].x+r<=imgleft_pre.cols){

                              cv::Mat t = imgleft_pre(Range(pt_l_p[i].y-r,pt_l_p[i].y+r),
                                                      Range(pt_l_p[i].x-r,pt_l_p[i].x+r));

                              matchTemplate(r_p_box, t, result, 5);
                              minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
                              matchLoc = maxLoc;

                              //Translate to the position in previous right images
                              Point2f pt = Point2f(max(0,box_pre.r.x0-expand_th)+matchLoc.x + float(t.cols)/2.0,
                                                   max(0,box_pre.r.y0)+matchLoc.y + float(t.rows)/2.0);

                              pt_r_p[i] = pt;

                              //check intensity diff
                              if((pow((GetPixelValue(imgleft_pre,pt_l_p[i].x,pt_l_p[i].y) -
                                       GetPixelValue(imgright_pre,pt.x,pt.y)),2) > intensity_diff_th)){
                                  ok[i] = false;

                              }

                              //dy should be almost 0
                              if(abs(pt_r_p[i].y - pt_l_p[i].y)>=5){
                                  ok[i] = false;
                              }
                          }
                      }
                  }

                  //Template matching between right_pre and right_cur
                  //We dont have bounding box for right images, so we need to expand the left bounding box
                  //Or we can do template matching using left bouding box with right images.
                  cv::Mat r_c_box = imgright_cur(Range(max(0,box_cur.r.y0),
                                                       min(box_cur.r.y1,imgright_cur.rows)),//y0, y1
                                                 Range(max(0,box_cur.r.x0-expand_th),
                                                      min(box_cur.r.x1,imgright_cur.cols)));//x0, x1
                  //count=0;
                  for(int i = 0; i<pt_l_c.size(); i++){
                      if(ok[i]){
                          if(pt_r_p[i].y-r>=0 && pt_r_p[i].x-r>=0 &&
                             pt_r_p[i].y+r<=imgright_pre.rows && pt_r_p[i].x+r<=imgright_pre.cols){
                              cv::Mat t = imgright_pre(Range(pt_r_p[i].y-r,pt_r_p[i].y+r),
                                                       Range(pt_r_p[i].x-r,pt_r_p[i].x+r));

                              matchTemplate(r_c_box, t, result, 5);

                              minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
                              matchLoc = maxLoc;


                              //Translate to the position in current right images
                              Point2f pt = Point2f(max(0,box_cur.r.x0-expand_th)+matchLoc.x + float(t.cols)/2.0,
                                                   max(0,box_cur.r.y0)+matchLoc.y + float(t.rows)/2.0);

                              pt_r_c[i] = pt;
                              //check intensity diff
                              if((pow((GetPixelValue(imgright_pre,pt_r_p[i].x,pt_r_p[i].y) -
                                       GetPixelValue(imgright_cur,pt.x,pt.y)),2) > intensity_diff_th)){
                                  ok[i] = false;
                              }

                              //Use bounding box to constrain the matches
                              if(abs(pt_r_c[i].y-pt_r_p[i].y)>10){ //use dy constrain
                                  if (abs(pt_r_c[i].x-pt_r_p[i].x)>abs(2*dx) || abs(pt_r_c[i].x-pt_r_p[i].x) < abs(0.5*dx)||
                                      abs(pt_r_c[i].y-pt_r_p[i].y)>abs(2*dy) || abs(pt_r_c[i].y-pt_r_p[i].y) < abs(0.5*dy) ||
                                          (pt_r_c[i].y-pt_r_p[i].y)*dy < -10){//wrong matches
                                        ok[i] = false;
                                  }
                              }else{ //Only use dx constrain
                                  if (abs(pt_r_c[i].x-pt_r_p[i].x)>abs(2*dx) || abs(pt_r_c[i].x-pt_r_p[i].x) < abs(0.5*dx)||
                                          (pt_r_c[i].y-pt_r_p[i].y)*dy < -10){
                                      ok[i] = false;
                                  }
                              }

                          }
                      }
                  }


                  //Mat temp = imgleft_cur.clone();
                  //Mat pre = imgleft_pre.clone();//Debug
                 // Mat cur = imgleft_cur.clone();//Debug
                  //Template matching between right_cur and left_cur
                  float diff_coord = 5;
                  for(int i = 0; i<pt_l_c.size(); i++){
                      if(ok[i]){
                          if(pt_r_c[i].y-r>=0 && pt_r_c[i].x-r>=0 &&
                             pt_r_c[i].y+r<=imgright_cur.rows && pt_r_c[i].x+r<=imgright_cur.cols){

                              cv::Mat t = imgright_cur(Range(pt_r_c[i].y-r,pt_r_c[i].y+r),
                                                       Range(pt_r_c[i].x-r,pt_r_c[i].x+r));

                              matchTemplate(l_c_box, t, result, 5);
                              minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
                              matchLoc = maxLoc;

                              //Translate to the position in current left images
                              Point2f pt = Point2f(max(0,box_cur.r.x0)+matchLoc.x + float(t.cols)/2.0,
                                                   max(0,box_cur.r.y0)+matchLoc.y + float(t.rows)/2.0);

                              //check intensity diff
                              if((pow((GetPixelValue(imgright_cur,pt_r_c[i].x,pt_r_c[i].y) -
                                       GetPixelValue(imgleft_cur,pt.x,pt.y)),2) > intensity_diff_th)){
                                  ok[i] = false;

                              }
                              //check coincident
                              if(abs(pt.x - pt_l_c[i].x) > diff_coord || abs(pt.y - pt_l_c[i].y) > diff_coord){
                                  ok[i] = false;

                              }

                              //dy should be almost 0
                              if(abs(pt_r_c[i].y - pt.y)>=5){
                                  ok[i] = false;
                              }
//                              //*********************************************
//                              Point2f pt2 = Point2f(matchLoc.x + float(t.cols)/2.0,
//                                                    matchLoc.y + float(t.rows)/2.0);
//                              //Debug
//                              //circle(l_p_box, pt2,2,Scalar::all((250)));

//                              //circle(pre, pt,2,Scalar::all((250)));
//                              //circle(cur, pt_l_c[i],2,Scalar::all((250)));

//                              if(ok[i]){
//                                  //circle(imgleft_cur, pt,2,Scalar::all((250)));
//                                  //circle(imgleft_cur, pt_l_c[i],2,Scalar::all((0)));
//                                  //line(temp,pt_l_c[i],pt_l_p[i],Scalar::all(250),2);
//                                  if(id_==1){
//                                      circle(pre, pt,5,Scalar::all((250)));
//                                      circle(cur, pt_l_c[i],5,Scalar::all((250)));
//                                      line(cur,pt_l_c[i],pt_l_p[i],Scalar::all(0),5);
//                                  }
//                                  //count++;
//                              }

//                              //**********************************************
                          }
                      }
                  }

                  //*********************************************
                  //Debug
//                  if(id_==1){
//                      cv::imshow("pre",pre);
//                      cv::imshow("cur",cur);
//                      cout<<ok.size()<<endl;
//                      waitKey(1);
//                  }

                  //cout<<count<<"hi "<<endl;
                  //cout<<"Estimate using template matching."<<endl;
                  //*********************************************


                  //Update flows
//                  if(flow_update.size()>18){// We could use the insufficient flows in optical flow method
//                      flow_update.clear();
//                  }
                  for (int i = 0; i < pt_l_c.size(); i++) {
                      if (ok[i]){
                          flow_update.emplace_back(pt_l_c[i].x,pt_l_c[i].y, i,
                                                   pt_r_c[i].x,pt_r_c[i].y, i,
                                                   pt_l_p[i].x,pt_l_p[i].y, i,
                                                   pt_r_p[i].x,pt_r_p[i].y, i
                                                   );
                      }
                  }

                  GetLastFrame().instance_view.UpdateInstanceFlow(flow_update);

                  motion_delta = EstimateInstanceMotion(
                              GetLastFrame().instance_view.GetFlow(),
                              ssf_provider,
                              initial_estimate,
                              normals
                              );

//                  if(!motion_delta->IsPresent()){
//                      flow_update.clear();

//                      for (int i = 0; i < pt_l_c.size(); i++) {
//                          if (ok[i]){
//                              flow_update.emplace_back(pt_l_c[i].x,pt_l_c[i].y, i,
//                                                       pt_r_c[i].x,pt_r_c[i].y, i,
//                                                       pt_l_p[i].x,pt_l_p[i].y, i,
//                                                       pt_r_p[i].x,pt_r_p[i].y, i
//                                                       );
//                          }
//                      }

//                      motion_delta = EstimateInstanceMotion(
//                                  GetLastFrame().instance_view.GetFlow(),
//                                  ssf_provider,
//                                  initial_estimate,
//                                  normals
//                                  );

//                  }



                  if(motion_delta->IsPresent()){
                      cout<<"Successfully estimate motion using template matching method (pixel) for track# "<< id_ <<endl;
                  }else {
                      cout<<"Still can not estimate motion using template matching method (pixel) for track# "<< id_ <<endl;
                  }

                  //End of template matching-2

              }


          } //End of template matching
}//template matching

      }//end size>=2
}
  }//end ispresent





  GetLastFrame().relative_pose = motion_delta;
  if (motion_delta->IsPresent()) {
    GetLastFrame().relative_pose_world = new Option<Eigen::Matrix4f>(new Eigen::Matrix4f(
        egomotion * motion_delta->Get().matrix_form.cast<float>()));
  }
  else {
    GetLastFrame().relative_pose_world = new Option<Eigen::Matrix4f>();
  }


  int current_frame_idx = GetLastFrame().frame_idx;


  // TODO(andrei): Buffer region for rot/trans error within which the state of the track is still
  // left as uncertain.
  switch(track_state_) {// the state of the detected objects.
    case kUncertain://Has not determine which state it is, cuz initialize as uncertain.
      if (motion_delta->IsPresent()) {
        Eigen::Matrix4f error = egomotion * motion_delta->Get().matrix_form.cast<float>();
        // Computes a relative pose error using the metric from the KITTI odometry evaluation.
        float trans_error = TranslationError(error);
        float rot_error = RotationError(error);

        if (verbose) {
          cout << "Object " << id_ << " has " << setw(8) << setprecision(4) << trans_error
               << " translational error w.r.t. the egomotion." << endl;
          cout << "Rotation error: " << rot_error << "(currently unused)" << endl;
          cout << endl << "ME: " << endl << egomotion << endl;
          cout << endl << "Object: " << endl << motion_delta->Get().matrix_form << endl;
        }

        //I guess, the system assume a virtual camera for the estimation of the objects pose (For this situation, assume the objects are static).
        //If the estimation sucessful, then the 3D motion of the object is equal to the inverse of the virtual camera's motion.
        //For static object, the resulting 3D object motion will be (ideally) identical to the camera pose. -> object motion * camera pose = identical
        if (trans_error + rot_error > kTransErrorThresholdHigh - 0.1) {//error lager than the higher threshold -> dynamic //modified
          if (verbose) {
            cout << id_ << ": Uncertain -> Dynamic object!" << endl;
          }
          this->track_state_ = kDynamic;
        }
        else if (trans_error < kTransErrorThresholdLow) {//error below than the lower threshold -> static
          if (verbose) {
            cout << id_ << ": Uncertain -> Static object!" << endl;
          }
          // If the motion is below the threshold, meaning that the object is stationary, set it to
          // identity to make the result more accurate.
          motion_delta->Get().SetIdentity(); // This one only set when uncertain -> Static //Cause we set this frame as the original frame to this object
          this->track_state_ = kStatic;
        }
        else {//else, still be uncertain objects
          if (verbose) {
            cout << id_ << ": Uncertain -> Still uncertain because of ambiguous motion!" << endl;
          }
        }

        //Update the motion of the instance and the world motion
        this->last_known_motion_ = motion_delta->Get();// like constant velocity   //Note: this parameter has infulence on reconstruction
        this->last_known_motion_world_ = egomotion * motion_delta->Get().matrix_form.cast<float>();
        this->last_known_motion_time_ = current_frame_idx;



      }

      if (track_state_ != kUncertain) {
        // We just switched states

        if (HasReconstruction()) {//Reset the reconstruction.

          // Corner case: an instance which was static or dynamic, started being reconstructed, then
          // became uncertain again, and then was labeled as static or dynamic once again. In this
          // case, we have no way of registering our new measurements to the existing
          // reconstruction, so we discard it in order to start fresh.
          cout << "Uncertain -> Static/Dynamic BUT a reconstruction was already present. "
               << "Resetting reconstruction to avoid corruption." << endl;
          reconstruction_->Reset();
        }
      }

      break;

    case kStatic:
    case kDynamic:
      assert(last_known_motion_time_ >= 0);

      int frameThreshold = (track_state_ == kStatic) ? kMaxUncertainFramesStatic :
                           kMaxUncertainFramesDynamic;//static-> use the static threshold, otherwise use the dynamic threshold.

      if (motion_delta->IsPresent()) {
        if (track_state_ == kStatic) {//For static objects
          this->last_known_motion_.SetIdentity();
          this->last_known_motion_world_.setIdentity();// Not sure, use the frame in which the object shows as the coordinate.
                                                       // Therefore, identity relates to the first frame. (because static.

          GetLastFrame().relative_pose_world->Get().setIdentity();//Relative pose to the previous frame, in world coordinates. = indentity
        }
        else {//Dynamic
          this->last_known_motion_ = motion_delta->Get();
          this->last_known_motion_world_ = motion_delta->Get().matrix_form.cast<float>();
        }


        this->last_known_motion_time_ = current_frame_idx;
      }
      else {//dont get motion delta for this detected instance //Previous got state, but dont get motion delta for this frame
        int motion_age = current_frame_idx - last_known_motion_time_;
        if (motion_age > frameThreshold) {

          if (verbose) {
            cout << id_ << ": " << GetStateLabel() << " -> Uncertain because the relative motion "
                 << "could not be evaluated over the last " << frameThreshold << " frames."
                 << endl;
          }

          this->track_state_ = kUncertain;
        }
        else {
          // Assume constant motion for small gaps in the track.
          GetLastFrame().relative_pose = new Option<Pose>(new Pose(last_known_motion_)); //Has influence on instance reconstruction
          GetLastFrame().relative_pose_world = new Option<Eigen::Matrix4f>(new Eigen::Matrix4f(last_known_motion_world_));

        }
      }
      break;


}

  if(track_state_!=kUncertain){
      //Add //calculate visibility
      observation_++;
  }
}



void Track::ExtractSceneFlow(const SparseSceneFlow &scene_flow,
                                             vector<RawFlow, Eigen::aligned_allocator<RawFlow>> &out_instance_flow_vectors,
                                             const InstanceDetection &detection,
                                             const Eigen::Vector2i &frame_size,
                                             bool check_sf_start) {
  auto flow_mask = detection.delete_mask;
  const BoundingBox &flow_bbox = flow_mask->GetBoundingBox();
  map<pair<int, int>, RawFlow> coord_to_flow;
  int frame_width = frame_size(0);
  int frame_height = frame_size(1);

  for(const auto &match : scene_flow.matches) {
    int fx = static_cast<int>(match.curr_left(0));//u
    int fy = static_cast<int>(match.curr_left(1));//v
    int fx_prev = static_cast<int>(match.prev_left(0));
    int fy_prev = static_cast<int>(match.prev_left(1));

    if (flow_mask->ContainsPoint(fx, fy)) {
      // If checking the SF start, also ensure that keypoint position in the previous frame is also
      // inside the current mask. This limits the number of valid SF points, but also gets rid of
      // many noisy masks.
      if (!check_sf_start || detection.copy_mask->GetBoundingBox().ContainsPoint(fx_prev, fy_prev)) {
        coord_to_flow.emplace(pair<pair<int, int>, RawFlow>(pair<int, int>(fx, fy), match));
      }
    }
  }

  // TODO(andrei): This second half is NOT necessary and also slow... It should be removed
  // post-deadline.
  for (int cons_row = 0; cons_row < flow_bbox.GetHeight(); ++cons_row) {
    for (int cons_col = 0; cons_col < flow_bbox.GetWidth(); ++cons_col) {
      int cons_frame_row = cons_row + flow_bbox.r.y0;
      int const_frame_col = cons_col + flow_bbox.r.x0;

      if (cons_frame_row >= frame_height || const_frame_col >= frame_width) {
        continue;
      }

      u_char mask_val = flow_mask->GetData()->at<u_char>(cons_row, cons_col);
      if (mask_val == 1) {
        auto coord_pair = pair<int, int>(const_frame_col, cons_frame_row);
        if (coord_to_flow.find(coord_pair) != coord_to_flow.cend()) {
          out_instance_flow_vectors.push_back(coord_to_flow.find(coord_pair)->second);
        }
      }
    }
  }
}



//Added
//create predicted bounding bos
instreclib::utils::BoundingBox Track::PredictBoundingbox(const instreclib::utils::BoundingBox last_bbox,
                                                         int delta)const{
    //average changes
    float delta_u = 0;
    float delta_v = 0;


    //if(frames_.size()>3){
        auto end =  frames_.begin();

        if(frames_.size()>6){
            end = frames_.end()-5;
        }
        for(auto it = frames_.end()-1,itend = end;
            it != itend  ; it--){ //can changed to like only use previous 5 frames to do the prediction
            //Copy -> used for do instance reconstruction
            //delete -> used to delete related information from static scene
            delta_u += (*(it)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.x0
                    - (*(it-1)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.x0;
            //trick
            if(abs(delta_u)<=5){
                float dx2 = (*(it)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.x1
                        - (*(it-1)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.x1;
                if(abs(dx2)>=5){
                    delta_u = dx2;
                }
            }

            delta_v += (*(it)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.y0
                    - (*(it-1)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.y0;
        }

        //average:
        delta_u = float(delta)*delta_u/frames_.size();
        delta_v = float(delta)*delta_v/frames_.size();
    //}


//Method2
/*    for(auto it = frames_.begin(),itend = frames_.end();
        it != itend -1 ; ++it){
        //Copy -> used for do instance reconstruction
        //delete -> used to delete related information from static scene
        delta_u += (*(it+1)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.x0
                - (*(it)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.x0;

        delta_v += (*(it+1)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.y0
                - (*(it)).instance_view.GetInstanceDetection().GetCopyBoundingBox().r.y0;
    }

    //average:
    delta_u = float(delta)*delta_u/frames_.size();
    delta_v = float(delta)*delta_v/frames_.size();
*/


    //create a new bounding box which is predicted from the last box
    ///This provide a location where a bounding box should be in the current frame
    instreclib::utils::BoundingBox NewPredictBox(last_bbox.r.x0+delta_u,//x0
                                                       last_bbox.r.y0+delta_v,//y0
                                                       last_bbox.r.x0+delta_u+last_bbox.GetWidth(),//x1
                                                       last_bbox.r.y0+delta_v+last_bbox.GetHeight());//y1



    //End

    return NewPredictBox;

}


//Added
int Track::FindMatches(vector<bool> &out_ok,
                       vector<cv::Point2f> pt_l_c,
                       vector<cv::Point2f> kp1_new,
                       vector<cv::Point2f> pt_l_p,
                       float dx,
                       float dy,
                       int count_pre){
    int count = count_pre;
    vector<size_t> index;
    //check within a loop
    for (size_t in = 0; in<pt_l_c.size(); in++) {
        //lower threshold
        if(!(kp1_new[in].x>= pt_l_c[in].x-.8 && kp1_new[in].x<= pt_l_c[in].x+.8 &&
                kp1_new[in].y>= pt_l_c[in].y-.8 && kp1_new[in].y<= pt_l_c[in].y+.8 )){
            out_ok[in] = false;//0.8
        }
        else{
            //higher threshold
            if(!(kp1_new[in].x>= pt_l_c[in].x-0.5 && kp1_new[in].x<= pt_l_c[in].x+0.5 &&
                    kp1_new[in].y>= pt_l_c[in].y-0.5 && kp1_new[in].y<= pt_l_c[in].y+0.5)){
                out_ok[in] = false;//0.5

                //Record more matches
                //Use bounding box to constrain the matches//2  //0.5
                if (abs(pt_l_c[in].x-pt_l_p[in].x)<=abs(5*dx) && abs(pt_l_c[in].x-pt_l_p[in].x) >= abs(0.2*dx)){
                    if(abs(pt_l_c[in].y-pt_l_p[in].y)>10){ // if the object is not doing a horizontal movement -> use dy constrain
                        if(abs(pt_l_c[in].y-pt_l_p[in].y)<=abs(5*dy) && abs(pt_l_c[in].y-pt_l_p[in].y) >= abs(0.2*dy) &&
                                (pt_l_c[in].y-pt_l_p[in].y)*dy > -10){//suposed to be 0, but here have small tolerance//there will be samll 'shaking' of bounding box detection
                              index.push_back(in); //record
                        }
                    }else{//ignore dy

                        if ((pt_l_c[in].y-pt_l_p[in].y)*dy > -10)
                          //suposed to be 0
                          //but there will be samll 'shaking' of bounding box detection
                          index.push_back(in); //record

                    }
                }

            }
            else {
                //Use bounding box to constrain the matches
                if(abs(pt_l_c[in].y-pt_l_p[in].y)>10){ //use dy constrain //2 0.5
                    if (abs(pt_l_c[in].x-pt_l_p[in].x)>abs(2*dx) || abs(pt_l_c[in].x-pt_l_p[in].x) < abs(0.5*dx)||
                        abs(pt_l_c[in].y-pt_l_p[in].y)>abs(2*dy) || abs(pt_l_c[in].y-pt_l_p[in].y) < abs(0.5*dy) ||
                            (pt_l_c[in].y-pt_l_p[in].y)*dy < -10){//wrong matches
                          out_ok[in] = false;
                    }else{
                        count++;
                    }

                }else{ //Only use dx constrain
                    //2    0.5
                    if (abs(pt_l_c[in].x-pt_l_p[in].x)>abs(2*dx) || abs(pt_l_c[in].x-pt_l_p[in].x) < abs(0.5*dx)||
                            (pt_l_c[in].y-pt_l_p[in].y)*dy < -10){
                        out_ok[in] = false;
                    }else{
                        count++;
                    }
                }
            }
        }

    }
    //if not get enough matches, search in a wider area
    if(count<=35){//35
        for(auto &i: index){
            out_ok[i] = true;
            count++;

        }
    }
    return count;
}


//Not use
vector<RawFlow, Eigen::aligned_allocator<RawFlow>> Track::templateMatching(cv::Mat imgleft_cur,
                                                                           cv::Mat imgleft_pre,
                                                                           cv::Mat imgright_cur,
                                                                           cv::Mat imgright_pre,
                                                                           cv::Point matchLoc,
                                                                           instreclib::utils::BoundingBox  box_cur,
                                                                           cv::Mat temp2){

    //Record
    Point left_pre;
    left_pre = Point (matchLoc.x,matchLoc.y);




    //Initialize
    vector<RawFlow, Eigen::aligned_allocator<RawFlow>> flow_update;
    double minVal; double maxVal; Point minLoc; Point maxLoc;

    //Initialize result Mat
    int result_cols = imgleft_cur.cols;
    int result_rows = imgleft_cur.rows;
    cv::Mat result;
    result.create( result_rows, result_cols, CV_32FC1 );

    //Template matching2
    matchTemplate( imgright_pre , temp2, result, 5);//match_method = TM COEFF NORMED
    //Find max and min
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    matchLoc = maxLoc;
    //Update template
    temp2 = (imgright_pre)(Range(matchLoc.y,matchLoc.y + temp2.rows),
                            Range(matchLoc.x,min(matchLoc.x + temp2.cols,imgright_pre.cols)));




    //Record
    Point right_pre;
    right_pre = Point(matchLoc.x,matchLoc.y);




    //Template matching3
    matchTemplate( imgright_cur , temp2, result, 5);//match_method = TM COEFF NORMED
    //Find max and min
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    matchLoc = maxLoc;
    //Update template
    temp2 = (imgright_cur)(Range(matchLoc.y,matchLoc.y + temp2.rows),
                            Range(matchLoc.x,min(matchLoc.x + temp2.cols,imgright_cur.cols)));




    //Record
    Point right_cur;
    right_cur = Point(matchLoc.x,matchLoc.y);



    //Template matching4
    matchTemplate( imgleft_cur , temp2, result, 5);//match_method = TM COEFF NORMED
    //Find max and min
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    matchLoc = maxLoc;
    Point left_cur;
    left_cur =  Point (matchLoc.x,matchLoc.y);

   // temp2 = (imgleft_cur)(Range(matchLoc.y,matchLoc.y + temp2.rows),
                          // Range(matchLoc.x,min(matchLoc.x + temp2.cols,imgright_cur.cols)));
   // imshow("temp2",temp2);
   // waitKey(0);


    int th2 = 3;
    //check whether template matching is successful? //Give a tolerance

    if(box_cur.r.x0 - th2 <= matchLoc.x && box_cur.r.x1 + th2 >= matchLoc.x + temp2.cols&&
            box_cur.r.y0 - th2 <= matchLoc.y){
        int count = 0;
        for (int var = 5; var <= temp2.cols-5; ++var) {
            for(int var2 = 5; var2 <= temp2.rows-5; ++var2 ){
                flow_update.emplace_back(left_cur.x+var,left_cur.y+var2, count,
                                         right_cur.x+var,right_cur.y+var2, count,
                                         left_pre.x+var,left_pre.y+var2, count,
                                         right_pre.x+var,right_pre.y+var2, count
                                         );
                count++;


            }

        }
    }//End if





    //check of template matching


    return flow_update;
}




}  // namespace reconstruction
}  // namespace instreclib
