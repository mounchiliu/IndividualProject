

#include "InstanceTracker.h"



namespace instreclib {
namespace reconstruction {

using namespace std;
using namespace instreclib::segmentation;

void InstanceTracker::ProcessInstanceViews(int frame_idx,
                                           const vector<InstanceView, Eigen::aligned_allocator<InstanceView>> &new_views,
                                           const Eigen::Matrix4f current_camera_pose,
                                           cv::Mat input_grey_left,
                                           cv::Mat input_grey_right,
                                           cv::Mat input_hsv_left,
                                           cv::Mat input_hsv_right,
                                           VisualOdometryStereo::parameters my_sf_params,
                                           ITMLib::Objects::ITMView *main_view,
                                           const SparseSceneFlow &scene_flow
) {
  // 0. Convert the instance segmentation result (`new_views`) into track objects frame.
  list<TrackFrame, Eigen::aligned_allocator<TrackFrame>> new_track_frames;
  for (const InstanceView &view : new_views) {//For each instanceview of each instance,create ... for processing
      new_track_frames.emplace_back(frame_idx, view, current_camera_pose); //create new track frames and add it to the list
  }


  //Added
  //clear
  vector<int> trackedobjects_cur;

  // 1. Try to find a matching track for all the obtained trac frames //Data association
  this->AssignToTracks(new_track_frames, input_grey_left,input_grey_right,input_hsv_left,input_hsv_right,trackedobjects_cur,main_view,scene_flow,current_camera_pose);//Associate new detections with exisiting tracks by ranking them based on IoU score
                                         //between a new detection and the most recent frame in a track
                                         //This function has deleted tracked objects in new_track vector

  // 2. For leftover detections, put them into new, single-frame, tracks.
  for (const TrackFrame &track_frame : new_track_frames) {
    int track_id = track_count_;
    track_count_++;
    auto sparse_sf_provider = new instreclib::VisoSparseSFProvider(my_sf_params);
    Track new_track(track_id,sparse_sf_provider);//Initialize//added
    new_track.AddFrame(track_frame);
    new_track.AddGreyFrames(make_pair(input_grey_left,input_grey_right));
    new_track.AddHsvFrames(make_pair(input_hsv_left,input_hsv_right));

    this->id_to_active_track_.emplace(make_pair(track_id, new_track));

    //Added
    //Record
    trackedobjects_cur.push_back(track_id);
    //----------------------------------------------

  }

  //Added
  trackedobjects_pre_ =  trackedobjects_cur;
  //---------------------------------------------------------

  // 3. Iterate through existing tracks, find ``expired'' ones, and discard them.
  this->PruneTracks(frame_idx);
}

void InstanceTracker::PruneTracks(int current_frame_idx) {
  auto it = id_to_active_track_.begin();
  while (it != id_to_active_track_.end()) {
    int last_active = it->second.GetEndTime();
    int frame_delta = current_frame_idx - last_active;

    if (frame_delta > inactive_frame_threshold_) {
      if (it->second.HasReconstruction()) {
        // Before deallocating the track's instance reconstructor, we delete its reference to the
        // latest view in that track, since that view object is managed by the track object, and
        // shouldn't be deleted by InfiniTAM's destructor (not doing this leads to a double free
        // error).
        it->second.GetReconstruction()->SetView(nullptr);
      }

      cout << "Erasing track: #" << it->first << ", " << it->second.GetClassName() << " of "
           << it->second.GetSize() << " frames." << endl;
      it = id_to_active_track_.erase(it);
    } else {
      ++it;
    }
  }
}

pair<Track *, float> InstanceTracker::FindBestTrack(const TrackFrame &track_frame, float &areascore) {
                                                                      //new frame  //score
  if (id_to_active_track_.empty()) {
    return kNoBestTrack;
  }

  float best_score = -1.0f;
  Track *best_track = nullptr;


  for (auto &entry : id_to_active_track_) {
    const Track& track = entry.second;
    float areascore_;
    float score = track.ScoreMatch(track_frame,areascore_); // Score the overlap using the standard intersection-over-union (IoU) measure.
                                                 //Need some other robust data association?


    if (score > best_score) {
      best_score = score;
      best_track = &entry.second;
      areascore = areascore_;
    }
  }
  assert(best_score >= 0.0f);
  assert(best_track != nullptr);
  return std::pair<Track *, float>(best_track, best_score);
}

//Do the data association
void InstanceTracker::AssignToTracks(std::list<TrackFrame, Eigen::aligned_allocator<TrackFrame>> &new_detections,
                                     cv::Mat input_grey_left,//Added
                                     cv::Mat input_grey_right,//Added
                                     cv::Mat input_hsv_left,//Added
                                     cv::Mat input_hsv_right,//Added
                                     vector<int> &trackedobjects_cur,//Added
                                     ITMLib::Objects::ITMView *main_view,//Added
                                     const SparseSceneFlow &scene_flow,//Added
                                     const Eigen::Matrix4f current_camera_pose){//Added

    auto it = new_detections.begin();

  //Added
  //check whether all the instance in the last frame has got a match
  vector<bool> hastracked(trackedobjects_pre_.size(),false);
  vector<instreclib::utils::BoundingBox> box_new_detection;
  //--------------------------------------------------------------
  while (it != new_detections.end()) {

    //if((*it).instance_view.GetInstanceDetection().class_probability<0.6) //debug
          //continue;

    float areascore = 0;//Added
    pair<Track *, float> match = FindBestTrack(*it,areascore);
    Track *track = match.first;
    float score = match.second;

    if (score > kTrackScoreThreshold) {
//      cout << "Found a match based on overlap with score " << score << "." << endl;
//      cout << "Adding it to track #" << track->GetId() << " of length " << track->GetSize() << "."
//           << endl;




      //We use five previous frame to do the prediction
//        if(track->GetFrames().size()>5){
//            ofstream outputfile5("./record/IoUscore.txt",ios::app);

//            //reconstruction accuracy of static map
//            outputfile5<<areascore<<endl;
//            //End
//        }



      track->AddFrame(*it);


      //-------------------------------------------------
      //Added
      track->AddGreyFrames(make_pair(input_grey_left,input_grey_right));
      track->AddHsvFrames(make_pair(input_hsv_left,input_hsv_right));

/*      //Add
        if(areascore-0.485186<=0.00001){
            //Debug.
            auto last_bbox = track->GetFrames().at(track->GetFrames().size()-2).instance_view.GetInstanceDetection().GetCopyBoundingBox();
            auto new_bbox = track->GetFrames().back().instance_view.GetInstanceDetection().GetCopyBoundingBox();
            const instreclib::utils::BoundingBox NewPredictBox = track->PredictBoundingbox(last_bbox);
            if(track->GetmyGray().size()>5){
                cv::rectangle(track->GetmyGray().back().first,cv::Point(NewPredictBox.r.x0,NewPredictBox.r.y0),
                              cv::Point(NewPredictBox.r.x1,NewPredictBox.r.y1),cv::Scalar::all(0),4);

                cv::rectangle(track->GetmyGray().back().first,cv::Point(new_bbox.r.x0,new_bbox.r.y0),
                              cv::Point(new_bbox.r.x1,new_bbox.r.y1),cv::Scalar::all(250),2);

                cv::rectangle(track->GetmyGray()[track->GetmyGray().size()-2].first,cv::Point(last_bbox.r.x0,last_bbox.r.y0),
                              cv::Point(last_bbox.r.x1,last_bbox.r.y1),cv::Scalar(250,0,0),2);

                cv::imshow("GrayLeft",track->GetmyGray().back().first);
                cv::imshow("GrayLeft2",track->GetmyGray()[track->GetmyGray().size()-2].first);
                imwrite("GrayLeft_cur.png",track->GetmyGray().back().first);
                imwrite("GrayLeft_pre.png",track->GetmyGray()[track->GetmyGray().size()-2].first);

                cout<<float(new_bbox.IntersectWith(NewPredictBox).GetArea())/new_bbox.GetArea()<<" "<<areascore<<endl;
                cv::waitKey(0);
            }

        }
*/


      //Record
      trackedobjects_cur.push_back(track->GetId());
      box_new_detection.push_back((*it).instance_view.GetInstanceDetection().GetCopyBoundingBox());

      //check whether all the instance in the last frame has got a match
      int count = 0;
      for(auto id:trackedobjects_pre_){
          if(id == track->GetId()){
              hastracked[count] = true;
          }
          count++;
      }

      //-------------------------------------------------

      it = new_detections.erase(it);
    } else {
      ++it;
    }
  }


  //Added
  //Record
  if(!new_detections.empty()){
      for (auto new_detection:new_detections) {
          box_new_detection.push_back(new_detection.instance_view.GetInstanceDetection().GetCopyBoundingBox());
      }
  }


/*
  //find more association
  int box_threshold_radius_x = 200;
  int box_threshold_radius_y = 50;
  for(auto new_det : new_detections){
      auto box = new_det.instance_view.GetInstanceDetection().GetCopyBoundingBox();

      cv::Mat1b match_new (*input_grey_left,
                              cv::Range(box.r.y0,box.r.y1+1), //rowrange
                              cv::Range(box.r.x0,box.r.x1+1) //colrange
                              );

      //create searching area
      int x0 = max(0,box.r.x0 - box_threshold_radius_x);
      int x1 = min(box.r.x1 + box_threshold_radius_x,input_grey_left->cols);
      int y0 = max(box.r.y0 - box_threshold_radius_y,0);
      int y1 =  min(box.r.y1 + box_threshold_radius_y,input_grey_left->rows);


//-------------------------------------------------------drawing
      cv::Point pt1;
      //pt1.x = box.r.x0;
      //pt1.y = box.r.y0;
      pt1.x = x0;
      pt1.y = y0;
      cv::Point pt2;
      //pt2.x = box.r.x1;
     // pt2.y = box.r.y1;
      pt2.x = x1;
      pt2.y = y1;

      cv::rectangle(*input_grey_left,pt1,pt2,cv::Scalar(255,0,0),5);
      cv::imshow("deb2",*input_grey_left);
      cv::waitKey(0);
//-----------------------------------------------End debug


      //Check all the available tracked objects in the system
      //Search in previous frames
      for (auto track_id: id_to_active_track_) {
          auto Track_search = track_id.second;

        //Speed up search
          if(Track_search.GetLastFrame().frame_idx==new_det.frame_idx ||
             Track_search.GetLastFrame().frame_idx==(new_det.frame_idx -1)){
              continue;
          }
          auto recent_box = Track_search.GetLastFrame().instance_view.GetInstanceDetection().GetCopyBoundingBox();


          //Debug
          cv::Point pt31,pt41;
          pt31.x = recent_box.r.x0;
          pt31.y = recent_box.r.y0;
          pt41.x = recent_box.r.x1;
          pt41.y = recent_box.r.y1;
          cv::rectangle(*input_grey_left,pt31,pt41,cv::Scalar(255,0,0),5);
          cv::imshow("recent box",*input_grey_left);
          cv::waitKey(0);

          //--------------------------------------------------------------


          //check whether the recent one is in the searching area to speed up search
          if(recent_box.r.x0 >= x0 &&
             recent_box.r.x1 <= x1 &&
             recent_box.r.y0 >= y0 &&
             recent_box.r.y1 <= y1){


            //cv::Mat m0 = m.rowRange(i0, i1) -> row i0 ~ i1-1
            //Using the first detected frame to do ssd
            auto Last_left_grey = Track_search.GetmyGray().front().first;
            auto recent_box_first = Track_search.GetFrames().front().instance_view.GetInstanceDetection().GetCopyBoundingBox();
            //cv::Mat m0 = m.rowRange(i0, i1) -> row i0 ~ i1-1
             cv::Mat1b match_tracked_recent (*Last_left_grey,
                                     cv::Range(recent_box_first.r.y0,recent_box_first.r.y1+1), //rowrange
                                     cv::Range(recent_box_first.r.x0,recent_box_first.r.x1+1) //colrange
                                     );

             cv::imshow("patch",match_tracked_recent);
             //cv::imshow("patch_new",match_new);
             cv::waitKey(0);


             //Do ssd
             //find the best match
             //check label
             int diff = 0;
             auto new_box = new_det.instance_view.GetInstanceDetection().GetCopyBoundingBox();
             int direction = new_box.r.x0 - recent_box.r.x0;
             //dir>0 new one in the right
             //clip from back

             if(direction < 0){
                 for (size_t r = 0; r < min(match_new.rows, match_tracked_recent.rows); r++) {
                     for (size_t c = 0; c < min(match_new.cols, match_tracked_recent.cols); c++) {
                         diff += abs(match_tracked_recent[r][c] - match_new[r][c]);

                     }

                 }
             }
             else {
                 //clip
                 bool b = match_new.cols < match_tracked_recent.cols;
                 if(b){
                    cv::Mat match_tracked_recent2(match_tracked_recent,
                                                 cv::Range(0,match_tracked_recent.rows),
                                                 cv::Range(match_tracked_recent.cols-match_new.cols-1,match_tracked_recent.cols));
                    cv::imshow("patch",match_tracked_recent2);
                    cv::waitKey(0);

                 }


                 for (size_t r = 0; r < min(match_new.rows, match_tracked_recent.rows); r++) {
                     for (size_t c = 0; c < min(match_new.cols, match_tracked_recent.cols); c++) {
                         diff += abs(match_tracked_recent[r][c] - match_new[r][c]);

                     }

                 }
             }

             cout<<"test:"<<diff<<endl;



          }


      }


  }
  //---------------------------------------------------
  */

  //check whether all the instance in the last frame has got a match
  int count2 = 0;
  for(auto tracked_b:hastracked){
      if(tracked_b == false){
          //create the missed bounding box
          //Object occluded
          //Detect whether there has got a bounding box in the predicted location
          //trackedobjects_pre_[count2] -> object id
          Track &track_temp = id_to_active_track_.find(trackedobjects_pre_[count2])->second;
          auto last_box = track_temp.GetLastFrame().instance_view.GetInstanceDetection().GetCopyBoundingBox();
          auto predict_box = track_temp.PredictBoundingbox(last_box);
          //intersects with any box of all the bounding boxes in current frame?
          bool occluded = false;
          //used for prevent the condition of occluded objects.
          for(auto new_box:box_new_detection){

              int intersection = predict_box.IntersectWith(new_box).GetArea();

              if(float(intersection)/float(new_box.GetArea())>0.70){//intersection threshold
                  occluded = true;
              }
          }

          //not occluded -> create new bounding box // create new instance detections
          if(!occluded && predict_box.r.x0 >= 0 &&
                          predict_box.r.y0 >= 0 &&
                          predict_box.r.x1 <= input_grey_left.cols &&
                           predict_box.r.y1 <= input_grey_left.rows){
              //Associate information
              auto instance_detection_new = track_temp.GetLastFrame().instance_view.GetInstanceDetection();
              instance_detection_new.GetCopyBoundingBox() = track_temp.PredictBoundingbox(last_box);
              instance_detection_new.GetCopyBoundingBox() = track_temp.PredictBoundingbox(track_temp.GetLastFrame().instance_view.GetInstanceDetection().GetDeleteBoundingBox());
              //conservative_mask need update???

              //extract raw flow
              Vector2i frame_size_itm = main_view->rgb->noDims;
              Eigen::Vector2i frame_size(frame_size_itm.x, frame_size_itm.y);
              vector<RawFlow, Eigen::aligned_allocator<RawFlow>> instance_flow_vectors;
              ExtractSceneFlow(scene_flow,
                               instance_flow_vectors,
                               instance_detection_new,
                               frame_size);

              //instance view
              ITMRGBDCalib *calibration = new ITMRGBDCalib;
              *calibration = *main_view->calib;
              auto view = make_shared<ITMView>(calibration, frame_size_itm, frame_size_itm,1);

              //get instance flow using this bounding box
              const InstanceView new_instance_view(instance_detection_new,view,instance_flow_vectors);

              //create a track frame
              int frame_id_cur = track_temp.GetLastFrame().frame_idx+1;
              TrackFrame new_track_frame_new(frame_id_cur,new_instance_view,current_camera_pose);
              track_temp.AddFrame(new_track_frame_new);


      }

  }
      count2++;
  }


}






//Added
void InstanceTracker::ExtractSceneFlow(const SparseSceneFlow &scene_flow,
                                             vector<RawFlow, Eigen::aligned_allocator<RawFlow>> &out_instance_flow_vectors,
                                             const InstanceDetection &detection,
                                             const Eigen::Vector2i &frame_size,
                                             bool check_sf_start) {
  auto flow_mask = detection.delete_mask;
  const instreclib::utils::BoundingBox &flow_bbox = flow_mask->GetBoundingBox();


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




}  // namespace segmentation
}  // namespace instreclib
