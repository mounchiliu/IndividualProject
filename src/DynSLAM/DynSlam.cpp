

#include <chrono>
#include <thread>

#include "DynSlam.h"
#include "Evaluation/Evaluation.h"


#include <opencv2/opencv.hpp>

DEFINE_bool(dynamic_weights, false, "Whether to use depth-based weighting when performing fusion.");
DECLARE_bool(semantic_evaluation);
DECLARE_int32(evaluation_delay);

namespace dynslam {

using namespace instreclib::reconstruction;
using namespace dynslam::eval;

void DynSlam::ProcessFrame(Input *input) {
  // Read the images from the first part of the pipeline
  if (! input->HasMoreImages()) {
    cout << "No more frames left in image source." << endl;
    return;
  }

  bool first_frame = (current_frame_no_ == 0);

  utils::Tic("Read input and compute depth");
  if(!input->ReadNextFrame()) {
    throw runtime_error("Could not read input from the data source.");
  }
  utils::Toc();



  //Step 1
  //thread 1. Semantic segmentation
  future<shared_ptr<InstanceSegmentationResult>> seg_result_future = async(launch::async, [this] {
    if (dynamic_mode_ || FLAGS_semantic_evaluation) {
      utils::Timer timer("Semantic segmentation");
      timer.Start();
      auto segmentation_result = segmentation_provider_->SegmentFrame((*input_rgb_image_));
      timer.Stop();
      cout << timer.GetName() << " took " << timer.GetDuration() / 1000 << "ms" << endl;
      return segmentation_result;
    }
    else {
      return shared_ptr<InstanceSegmentationResult>(nullptr);
    }
  });




// Step 2
//  thread 2. Compute sparse Scene Flow from stereo pair
  future<void> ssf_and_vo = async(launch::async, [this, &input, &first_frame] {
    utils::Tic("Sparse Scene Flow");



    //Step 2.1. Convert to grayscale images

    // Whether to use input from the original cameras. Unavailable with the tracking dataset.
    // When original gray images are not used, the color ones are converted to grayscale and passed
    // to the visual odometry instead.
    bool original_gray = false;

    // TODO-LOW(andrei): Reuse these buffers for performance.
    cv::Mat1b *left_gray, *right_gray;
    if (original_gray) {
      input->GetCvStereoGray(&left_gray, &right_gray);//Get input data to the variables
    }
    else {
      cv::Mat3b *left_col, *right_col;
      input->GetCvStereoColor(&left_col, &right_col);

      left_gray = new cv::Mat1b(left_col->rows, left_col->cols);
      right_gray = new cv::Mat1b(right_col->rows, right_col->cols);

      cv::cvtColor(*left_col, *left_gray, cv::COLOR_RGB2GRAY);
      cv::cvtColor(*right_col, *right_gray, cv::COLOR_RGB2GRAY);





    }




    //Step 2.2. Compute Sparse Scene Flow and visual odometry
    // TODO(andrei): Idea: compute only matches here, the make the instance reconstructor process
    // the frame and remove clearly-dynamic SF vectors (e.g., from tracks which are clearly dynamic,
    // as marked from a prev frame), before computing the egomotion, and then processing the
    // reconstructions. This may improve VO accuracy, and it could give us an excuse to also
    // evaluate ATE and compare it with the results from e.g., StereoScan, woo!
    sparse_sf_provider_->ComputeSparseSF( //Compute matches
        make_pair((cv::Mat1b *) nullptr, (cv::Mat1b *) nullptr),
        make_pair(left_gray, right_gray),
        (*latest_seg_result_).instance_detections //.data returns a pointer of the vector
    );


    if (!sparse_sf_provider_->FlowAvailable() && !first_frame) {
      cerr << "Warning: could not compute scene flow." << endl;
    }
    utils::Toc("Sparse Scene Flow", false);


/*
    //For debug
    if(sparse_sf_provider_->FlowAvailable()){
        //Draw matches //raw
        cv::Point2f pt1,pt2;
        auto matches = sparse_sf_provider_->GetFlow().matches;
        for(int i=0;i<matches.size();i++){
           pt1.x=matches[i].curr_left[0];
           pt1.y=matches[i].curr_left[1];

           pt2.x=matches[i].prev_left[0];
           pt2.y=matches[i].prev_left[1];

           cv::circle(*left_gray,pt1,2,cv::Scalar(255,0,0),-1);
           cv::line(*left_gray,pt1,pt2,cv::Scalar(255,0,0)); //point 2 to point 1 (previous to current) //Draw the flow (Matching)
        }

        //Draw deleted matches
        cv::Point2f pt3,pt4;
        auto matches_d = sparse_sf_provider_->latest_flow_delete_.matches;
        for(int i=0;i<matches_d.size();i++){
            pt3.x=matches_d[i].curr_left[0];
            pt3.y=matches_d[i].curr_left[1];

            pt4.x=matches_d[i].prev_left[0];
            pt4.y=matches_d[i].prev_left[1];

            //cv::circle(*left_gray,pt3,2,cv:: Scalar(255,0,0),-1);
            //cv::line(*left_gray,pt3,pt4,cv::Scalar(255,0,0)); //Draw the flow (Matching) of potential dynamic objects
        }

        cv::imshow("current matches",
                   *left_gray);

        cv::waitKey(0);//Wait for a keystroke in the window

    }
     //END For debug //Drawing results
*/


    //Step 2.2.3. Get Visual Odometry
    utils::Tic("Visual Odometry");
    Eigen::Matrix4f delta = sparse_sf_provider_->GetLatestMotion();//Relative motion from sparse scene flow

    // TODO(andrei): Nicer way to do this switch.
    bool external_odo = true;
    if (external_odo) {
      Eigen::Matrix4f new_pose = delta * pose_history_[pose_history_.size() - 1]; //world to cam
      static_scene_->SetPose(new_pose.inverse());//to the world coordinate //Set pose
      pose_history_.push_back(new_pose);//to the current camera
    }
    else {
      //##################################
      // Used when we're *not* computing VO as part of the SF estimation process.
      static_scene_->Track();
      Eigen::Matrix4f new_pose = static_scene_->GetPose();
      pose_history_.push_back(new_pose);
      //##################################
    }
    if (! original_gray) {
        delete left_gray;
        delete right_gray;
    }


    utils::Toc("Visual Odometry", false);
  });


//  //main processing for DynSLAM
//  std::promise<shared_ptr<InstanceSegmentationResult>> promise;
//  auto future = promise.get_future();



  seg_result_future.wait();
  // 'get' ensures any exceptions are propagated (unlike 'wait').
  ssf_and_vo.get();

  utils::Tic("Input preprocessing");

  input->GetCvImages(&input_rgb_image_, &input_raw_depth_image_);//Get the input data to the variables. first one for left
  static_scene_->UpdateView(*input_rgb_image_, *input_raw_depth_image_);

  utils::Toc();

  // Separate input and reconstruction
  // Split the scene up into instances, and fuse each instance independently.

  cv::Mat3b *left_col, *right_col;
  input->GetCvStereoColor(&left_col, &right_col);


  //Added
  //try HSV color range to sovle shadow effect

  cv::Mat hsv_l[3],hsv_r[3];

  cv::Mat HSV_image;
  cvtColor(*left_col, HSV_image, cv::COLOR_BGR2HSV);
  split(HSV_image, hsv_l);
  cv::Mat1b left_hsv= hsv_l[1];
  cvtColor(*right_col, HSV_image, cv::COLOR_BGR2HSV);
  split(HSV_image, hsv_r);
  cv::Mat1b right_hsv = hsv_r[1];




  utils::Tic("Instance tracking and reconstruction");
  if (sparse_sf_provider_->FlowAvailable()) { //When the scene flow is available, split...,
                                              //For the first frame, not available.
    this->latest_seg_result_ = seg_result_future.get();
    // We need flow information in order to correctly determine which objects are moving, so we
    // can't do this when no scene flow is available (i.e., in the first frame).



    //Added
    //Calculate normal vectors
    auto matches_feature = sparse_sf_provider_->GetFlow().matches;
    int size = matches_feature.size();
    matches_feature.erase(matches_feature.begin()+size-1,matches_feature.end());
    vector<Eigen::Vector3d> features_for_normal;
    //No matches on dynamic objects
    for(auto &match:matches_feature){

        auto ul = match.curr_left.x();
        auto vl = match.curr_left.y();
        for(auto detection:latest_seg_result_->instance_detections){
            float x1 = detection.GetCopyBoundingBox().r.x0;
            float x2 = detection.GetCopyBoundingBox().r.x1;
            float y2 = detection.GetCopyBoundingBox().r.y1;

            int th = 25;//25 //Should add semantic segmentation to segment road, but I dont have time to do so, therefore use tricky way
            if(latest_seg_result_->instance_detections.size() <=2){
                th = 60;
            }

            if(ul>=x1-10 && ul<= x2+10 && vl>= y2+5 && vl <=y2+th){

                auto ur = match.curr_right.x();

                //Get three-D point

                double X,Y,Z;

                double d = ul - ur;
                if(d<0.001)
                    continue;
                X = (ul-my_sf_params_.calib.cu)*my_sf_params_.base/d;
                Y = (vl-my_sf_params_.calib.cv)*my_sf_params_.base/d;
                Z = my_sf_params_.calib.f*my_sf_params_.base/d;

                //drawing
                //cv::circle(*left_gray,cv::Point2f(ul,vl),5,cv:: Scalar(250,0,0),-1);

                features_for_normal.push_back(Eigen::Vector3d(X,Y,Z));
            }
        }
    }

//   //Debug
//   //cv::imshow("features on ground", *left_gray);
//   //cv::waitKey(0);

    vector<Eigen::Vector3d> normals;

    //For normal sequence normal vector similar to Eigen::Vector3d(0,1,0)
    //Get some normal vectors
    if(features_for_normal.size()>=5){
        for (size_t m = 0; m < features_for_normal.size(); m++) {
            for (size_t n = m+1; n < features_for_normal.size(); n++) {
                for (size_t w = n+1; w < features_for_normal.size(); w++) {
                    for (size_t t = w+1; t < features_for_normal.size(); t++) {
                        Eigen::Vector3d v1 = features_for_normal[m] - features_for_normal[w];
                        Eigen::Vector3d v2 = features_for_normal[n] - features_for_normal[t];
                        Eigen::Vector3d temp = v1.cross(v2);
                        temp.normalize();
                        if(temp[0] == 0 && temp[1] == 0 && temp[1] == 0)
                            continue;

                        if(temp[1]<0) //keep same direction
                            temp = v2.cross(v1);

                        normals.push_back(temp);

                        if(normals.size()>=1000)
                            break;
                    }
                }

            }
        }
    }


//    //Do RANSAC based framework to select m top hypotheses for the normals

    int itr = 100;
    int total_inliers_num = 0;
    int index1;
    srand(time(0));
    vector<Eigen::Vector3d> total_inliers;
    if(!normals.empty()){
        for (size_t k=0;k<itr;k++){
            //Get one random normal vectors
            index1 = rand()%normals.size();

            Eigen::Vector3d v2(0,1,0);  //For normal sequence normal vector similar to Eigen::Vector3d(0,1,0)
            //Find inliers
            int inliers_num = 0;
            vector<Eigen::Vector3d> inliers;


            if (v2.cross(normals[index1]).norm() <= 0.5){
                inliers_num++;
                inliers.push_back(normals[index1]);

                for(Eigen::Vector3d n: normals){
                    if (n.cross(normals[index1]).norm() < 1e-4 &&
                        n.cross(v2).norm()<=0.5    ){
                        inliers_num++;
                        inliers.push_back(n);

                    }
                }
            }


            if(inliers_num>total_inliers_num){
                total_inliers_num = inliers_num;
                total_inliers = inliers;
            }
        }

    }

    //Remove outliers
    normals = total_inliers;
    while(normals.size()<50){
        //normals.push_back(Eigen::Vector3d (0,1,0));
        if(!normals.empty()){
            normals.push_back(normals[rand()%normals.size()]); //We should make sure the cost function has equal effect of normal vectors
        }
        else {
            normals.push_back(Eigen::Vector3d (0,1,0));
        }
    }


    //Try
    //normals = vector<Eigen::Vector3d> (50,Eigen::Vector3d(0,1,0));


    cv::Mat1b left_hsv_temp;  left_hsv.copyTo(left_hsv_temp);
    cv::Mat1b right_hsv_temp; right_hsv.copyTo(right_hsv_temp);


    //Test:If we not using sparse flow, only use optical flow to estimate poses.
    //sparse_sf_provider_->GetFlow().matches.clear();
    if (dynamic_mode_ && current_frame_no_ % experimental_fusion_every_ == 0) { // What does this mean???  -> I suppose this for determining whether the scene flow is available?
      instance_reconstructor_->ProcessFrame( //do the segmentation, find the object motion based on all frames up to the current frame, also update reconstruction.
          this,
          static_scene_->GetView(),
          *latest_seg_result_,
         //  sparse_sf_provider_->latest_flow_delete_,//Flow of dynamic objects
          sparse_sf_provider_->GetFlow(), //scene flow of the two consecutive frames (static scene + dynamic objects) //We can use segmented results to only get the dynamic objects scene flow
          *sparse_sf_provider_,
          always_reconstruct_objects_,//Has set to do the force reconstruction for non-dynamic objects e.g. a parked car.
          left_hsv,
          right_hsv,
          left_hsv_temp,//Not use
          right_hsv_temp,//Not use
          my_sf_params_,
          normals);
    }
  }

 // delete left_gray;
  //delete right_gray;

  utils::Toc();

  //Step 4. Static map reconstruction
  // Perform the tracking after the segmentation, so that we may in the future leverage semantic
  // information to enhance tracking.
  if (! first_frame) {
    if (current_frame_no_ % experimental_fusion_every_ == 0) {
      utils::Tic("Static map fusion");
      static_scene_->Integrate();//Pass to infiniTam
      static_scene_->PrepareNextStep();
      utils::TocMicro();

      // Idea: trigger decay not based on frame gap, but using translation-based threshold.
      // Decay old, possibly noisy, voxels to improve map quality and reduce its memory footprint.
      utils::Tic("Map decay");
      static_scene_->Decay();
      utils::TocMicro();
    }
  }

  int evaluated_frame_idx = current_frame_no_ - FLAGS_evaluation_delay;
  if (!FLAGS_enable_evaluation && evaluated_frame_idx > 0) { //Evaluation Switch //Add //Modified
    utils::Tic("Evaluation");
    bool enable_compositing = (FLAGS_evaluation_delay == 0);
    evaluation_->EvaluateFrame(input, this, evaluated_frame_idx, enable_compositing);
    utils::Toc();
  }
  evaluation_->LogMemoryUse(this);

  // Final sanity check after the frame is processed: individual components should check for errors.
  // If something slips through and gets here, it's bad and we want to stop execution.
  ITMSafeCall(cudaDeviceSynchronize());
  cudaError_t last_error = cudaGetLastError();
  if (last_error != cudaSuccess) {
    cerr << "A CUDA error slipped by undetected from a component of DynSLAM!" << endl;

    // Trigger the regular error response.
    ITMSafeCall(last_error);
  }

  current_frame_no_++;
}

const unsigned char* DynSlam::GetObjectPreview(int object_idx) {
  ITMUChar4Image *preview = instance_reconstructor_->GetInstancePreviewRGB(object_idx);
  if (nullptr == preview) {
    // This happens when there's no instances to preview.
    out_image_->Clear();
  } else {
    out_image_->SetFrom(preview, ORUtils::MemoryBlock<Vector4u>::CPU_TO_CPU);
  }

  return out_image_->GetData(MemoryDeviceType::MEMORYDEVICE_CPU)->getValues();
}

void DynSlam::SaveStaticMap(const std::string &dataset_name, const std::string &depth_name) const {
  string target_folder = EnsureDumpFolderExists(dataset_name);
  string map_fpath = utils::Format("%s/static-%s-mesh-%06d-frames.obj",
                                   target_folder.c_str(),

                                   depth_name.c_str(),
                                   current_frame_no_);
  cout << "Saving full static map to: " << map_fpath << endl;
  static_scene_->SaveSceneToMesh(map_fpath.c_str());
}

void DynSlam::SaveDynamicObject(const std::string &dataset_name,
                                const std::string &depth_name,
                                int object_id) const {
  cout << "Saving mesh for object #" << object_id << "'s reconstruction..." << endl;
  string target_folder = EnsureDumpFolderExists(dataset_name);
  string instance_fpath = utils::Format("%s/instance-%s-%06d-mesh.obj",
                                        target_folder.c_str(),
                                        depth_name.c_str(),
                                        object_id);
  instance_reconstructor_->SaveObjectToMesh(object_id, instance_fpath);

  cout << "Done saving mesh for object #" << object_id << "'s reconstruction in file ["
       << instance_fpath << "]." << endl;
}

std::string DynSlam::EnsureDumpFolderExists(const string &dataset_name) const {
  // TODO-LOW(andrei): Make this more cross-platform and more secure.
  string today_folder = utils::GetDate();
  string target_folder = "mesh_out/" + dataset_name + "/" + today_folder;
  if(system(utils::Format("mkdir -p '%s'", target_folder.c_str()).c_str())) {
    throw runtime_error(utils::Format("Could not create directory: %s", target_folder.c_str()));
  }

  return target_folder;
}




}
