#include <dso_ros/dso_node.h>
#include <dso_ros/ros_output_wrapper.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "util/Undistort.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "dso_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::MultiThreadedSpinner spinner(2);

    dso_ros::DsoNode node(nh, nh_private);
    node.setting_loop_rate = 15; //Hz

    std::thread DsoThread(&dso_ros::DsoNode::RunDSO, &node);
    std::thread publishThread (&dso_ros::DsoNode::PublishMarginalizedStuffFromDSO, &node, node.setting_loop_rate);
    spinner.spin();
    DsoThread.join();
    publishThread.join();


  return 0;
}

dso_ros::DsoNode::DsoNode(ros::NodeHandle &n, ros::NodeHandle &n_private)
  : nh(n)
  , nh_private(n_private)
  , image_file("")
  , calib_file("")
  , vignette_file("")
  , gamma_file("")
  , rectified_file("")
  , stats_file("")
  , initial_timestamp(0)
  , display_GUI_for_my_system(false)
{
    initParams();

    /*****************************************************/
    /*   Setting the mode:                               */
    /*   0 = Live                                        */
    /*   1 = Run Our System                              */
    /*   2 = Generate rectified Full Resolution Images   */
    /*   3 = Generate rectified Half Resolution Images   */
    /*****************************************************/

    if (mode == 0 || mode == 1)
    {
        KF_ID_calib_pose_points_sub = n.subscribe(
              "KF_ID_calib_pose_points", 10, &dso_ros::DsoNode::KF_ID_calib_pose_points_callback, this);

        display_GUI_for_my_system_sub = n.subscribe(
                "display_GUI_for_my_system", 10, &dso_ros::DsoNode::Display_GUI_callback, this);

        marginalized_KF_time_image_calib_pose_points_pub = n.advertise<std_msgs::Float64MultiArray>("marginalized_KF_time_image_calib_pose_points", 10);

        reset_count_pub = n.advertise<std_msgs::Int64>("reset", 10);

        rectified_live_image_pub = n.advertise<sensor_msgs::Image>("image_rectified", 10);
    }
    else if (mode == 2 || mode ==3)
        rectified_live_image_pub = n.advertise<sensor_msgs::Image>("image_rectified", 10);

    dso::setting_desiredImmatureDensity = 600;
    dso::setting_desiredPointDensity = 800;
    dso::setting_minFrames = 4;
    dso::setting_maxFrames = 6;
    dso::setting_maxOptIterations = 4;
    dso::setting_minOptIterations = 1;
    dso::setting_logStuff = false;
    dso::setting_kfGlobalWeight = 1;

    dso::setting_pointLocalSmooth = false; // For the current version, keep it as false.
    dso::setting_patchRadius = 5; // For the current version, this is not used.

    dso::setting_onlyLogKFPoses = true;
    dso::setting_render_display3D = false;
    dso::setting_render_displayDepth = false;
    dso::setting_render_displayVideo = false;
    dso::setting_render_displayResidual = false;
    dso::setting_render_renderWindowFrames = false;
    dso::setting_render_plotTrackingFull = false;
    dso::setting_render_displayCoarseTrackingFull = false;

    original_setting_photometricCalibration = dso::setting_photometricCalibration;

}

dso_ros::DsoNode::~DsoNode()
{
}

void dso_ros::DsoNode::Display_GUI_callback(const std_msgs::Empty &msg)
{
    display_GUI_for_my_system = true;
}

void dso_ros::DsoNode::KF_ID_calib_pose_points_callback(const std_msgs::Float32MultiArray& id_calib_pose_points)
{
    std::vector<float> ID_calib_pose_points_received = id_calib_pose_points.data;

    // Get KF IDs
    int num_KFs = (int) ID_calib_pose_points_received.front();
    ID_calib_pose_points_received.erase(ID_calib_pose_points_received.begin());

    assert (num_KFs > 0);

    std::vector<int> KF_IDs_received;
    while (num_KFs > 0)
    {
        KF_IDs_received.push_back((int) ID_calib_pose_points_received.front());
        ID_calib_pose_points_received.erase(ID_calib_pose_points_received.begin());
        num_KFs--;
    }

    for (auto ID: KF_IDs_received)
    {
        auto it_KF_ID = std::find(
                KF_IDs_temp.begin(),
                KF_IDs_temp.end(),
                ID);
        if (it_KF_ID == KF_IDs_temp.end())
            KF_IDs_temp.push_back(ID);
    }

    // Get marginalized KF ID (or the most recent KF ID) and its calibration parameters and pose
    int marginalized_KF_ID = (int) ID_calib_pose_points_received.front();
    ID_calib_pose_points_received.erase(ID_calib_pose_points_received.begin());

    bool marginalized = (marginalized_KF_ID == -1 ? false : true);

    std::vector<float> calib(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+4);
    ID_calib_pose_points_received.erase(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+4);

    std::vector<float> pose(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+12);
    ID_calib_pose_points_received.erase(
            ID_calib_pose_points_received.begin(),
            ID_calib_pose_points_received.begin()+12);

    if (marginalized)
    {
        marginalized_KF_IDs_temp.push_back(marginalized_KF_ID);
        marginalized_KF_calib_temp.push_back(calib);
        marginalized_KF_poses_temp.push_back(pose);
    }

    // Get marginalized points
    std::vector<float> points = ID_calib_pose_points_received;
    ID_calib_pose_points_received.clear();
    for (auto p : points)
        marginalized_points_temp.push_back(p);

    if (marginalized)
    {
        marginalized_KF_points_temp.push_back(marginalized_points_temp);
        marginalized_points_temp.clear();
    }

}


void dso_ros::DsoNode::RunDSOLive() {
    cv::Mat frame;
    cv::Mat pose;
    cv::VideoCapture cap;

    Undistort* undistorter = Undistort::getUndistorterForFile(calib_file, gamma_file, vignette_file);

    bool open = cap.open(0);
    //cap.set(4,720);
    //cap.set(5,576);

    bool frame_captured = cap.read(frame);
    namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    imshow("Display window", frame);

    printf("Frame info: type: %i chan: %i width: %i height: %i\n",frame.type(), frame.channels(), frame.cols, frame.rows);
    //TODO: Need to convert to bw?
    MinimalImageB minImg((int)frame.cols, (int)frame.rows,(unsigned char*)frame.data);
    ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);

    image_h = undistImg->h;
    image_w = undistImg->w;
    image_h_half = image_h/2;
    image_w_half = image_w/2;

    fullSystem = new FullSystem();
    if(undistorter->photometricUndist != 0)
    {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }

    if (display_GUI)
    {
        if (half_resolution)
            fullSystem->outputWrapper.push_back(new dso::IOWrap::PangolinDSOViewer(image_w_half,image_h_half));
        else
            fullSystem->outputWrapper.push_back(new dso::IOWrap::PangolinDSOViewer(image_w,image_h));
    }

    dso::setting_photometricCalibration = 0;
    fullSystem->linearizeOperation = 0;

    fullSystem->outputWrapper.push_back(new dso_ros::ROSOutputWrapper(nh, nh_private));

    printf("LOADING COMPLETE!\n");

    int curFrameId = 0;
    double initTimestamp = 0;
    int nTotalFramesAfterInit=0;
    double frame_timestamp = 0.0;
    double track_start_timestamp = 0.0;
    double track_end_timestamp = 0.0;

    while(true) {
        frame_captured = cap.read(frame);
        cvtColor(frame, frame, CV_RGB2GRAY);
        frame_timestamp = ros::Time::now().toSec();
        MinimalImageB minFrame((int)frame.cols, (int)frame.rows,(unsigned char*)frame.data);

        //Undistort is basically just a copy when photometriccalibration=0
        dso::setting_photometricCalibration = 0;
        std::shared_ptr<dso::ImageAndExposure> frame_orig(undistorter->undistort<unsigned char>(&minFrame, 1, frame_timestamp, 1.0f));

        //Perform actual undistort if needed here
        dso::setting_photometricCalibration = original_setting_photometricCalibration;
        std::shared_ptr<dso::ImageAndExposure> frame_undist(undistorter->undistort<unsigned char>(&minFrame, 1, frame_timestamp, 1.0f));
        
        if (display_GUI_for_my_system) {
            sensor_msgs::ImageConstPtr rectified_live_image_msg;
            rectified_live_image_msg = ConvertImageAndExposure2ImagePtr(frame_undist, false);
            rectified_live_image_pub.publish(rectified_live_image_msg);
        }

        if(!fullSystem->initialized) {   // if not initialized: reset start time.
            printf("Waiting for system init..\n");
            usleep(1000);
            initTimestamp = ros::Time::now().toSec();
        }

        if (fullSystem->initFailed || dso::setting_fullResetRequested) {
            printf("Resetting\n");
            reset();
        }

        if (fullSystem->initialized) {
            nTotalFramesAfterInit++;
        }


        {
            // Send images without calibration to ORB_SLAM2
            std::unique_lock<std::mutex> lock(frame_sync_mtx);
            frame_ptrs_sync.push_back(frame_orig);
            frame_timestamps_sync.push_back(frame_timestamp);
            frame_IDs_sync.push_back(curFrameId);
            curFrameId++;
        }


        // Use images with photo calib in DSO
        track_start_timestamp = ros::Time::now().toSec();
        fullSystem->addActiveFrame(frame_undist.get(), curFrameId, half_resolution);
        track_end_timestamp = ros::Time::now().toSec();

        trackingTimes.push_back(track_end_timestamp - track_start_timestamp);

        if(fullSystem->isLost)
        {
            //dso::setting_fullResetRequested = true;
            printf("TRACKING LOST!!\n");
            //break;
        }
        else if (fullSystem->initialized)
        {
            printf("System Initialized\n"); 
        }

    }

    printf("TRACKING FINISHED!! \n");
    fullSystem->blockUntilMappingIsFinished();

    printf("Marginalize Everything By Force!! \n");
    for (int i = 0; i < 10; i++)
    {
        usleep(0.1*1000*1000);
        fullSystem->marginalizeEverythingByForce();
    }

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
        ow->publishFullStop();

    ///  Step 6: Print timing statistics 

    double trackingTimeMed, trackingTimeAvg, trackingTimeStd;
    if (trackingTimes.empty())
    {
        trackingTimeMed = 0;
        trackingTimeAvg = 0;
        trackingTimeStd = 0;
    }
    else
    {
        sort(trackingTimes.begin(), trackingTimes.end());
        trackingTimeMed = trackingTimes[trackingTimes.size()/2];
        trackingTimeAvg = accumulate(trackingTimes.begin(), trackingTimes.end(), 0.0)/trackingTimes.size();
        std::vector<double> diff(trackingTimes.size());
        std::transform(trackingTimes.begin(), trackingTimes.end(), diff.begin(), std::bind2nd(std::minus<double>(), trackingTimeAvg));
        trackingTimeStd = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0)/trackingTimes.size();
        trackingTimeStd = std::sqrt(trackingTimeStd);
    }

    while (ros::ok())
    {
        ros::Duration(1).sleep();
    }


    for (dso::IOWrap::Output3DWrapper *ow : fullSystem->outputWrapper) {
      ow->join();
      delete ow;
    }

    delete fullSystem;
}


void dso_ros::DsoNode::RunDSO() {
    this->RunDSOLive();
    exit(1);
    return;
}

void dso_ros::DsoNode::PublishMarginalizedStuffFromDSO(const float &publish_loop_rate)
{
    if (mode != 1)
        return;

    ros::Rate loop_rate(publish_loop_rate);
    int wait_counter = 0;

    while (ros::ok())
    {
        loop_rate.sleep();

        if (KF_IDs_temp.size() == 0) continue;


        bool new_KF_added = false;
        for (auto KF_ID : KF_IDs_temp)
        {
            if (KF_IDs_sync.empty())
            {
                KF_IDs_sync.push_back(KF_IDs_temp.back());
                new_KF_added = true;
                break;
            }
            else
            {
                auto it_KF_ID = std::find(
                                KF_IDs_sync.begin(),
                                KF_IDs_sync.end(),
                                KF_ID);
                if (it_KF_ID == KF_IDs_sync.end() && KF_ID > KF_IDs_sync.back())
                {
                    KF_IDs_sync.push_back(KF_ID);
                    new_KF_added = true;
                    break;
                }
            }
        }

        if (KF_IDs_sync.empty())
            continue;

        assert(KF_IDs_sync.back() > 0);

        {
            std::unique_lock<std::mutex> lock(frame_sync_mtx);

            if (frame_IDs_sync.size() != frame_ptrs_sync.size()
                    || frame_IDs_sync.size() != frame_timestamps_sync.size())
            {
                std::cout << "Error: frame_sync vectors size mismatch !!!" << std::endl;
                std::cout << "frame_IDs_sync.size() = " << frame_IDs_sync.size()  <<std::endl;
                std::cout << "frame_ptrs_sync.size() = " << frame_ptrs_sync.size()  <<std::endl;
                std::cout << "frame_timestamps_sync.size() = " << frame_timestamps_sync.size()  <<std::endl;
                continue;
            }

            assert(frame_IDs_sync.size() == frame_ptrs_sync.size());
            assert(frame_IDs_sync.size() == frame_timestamps_sync.size());
        }



        if (new_KF_added)
        {
            std::unique_lock<std::mutex> lock(frame_sync_mtx);

            auto it_frame_IDs_history = std::find(
                            frame_IDs_sync.begin(),
                            frame_IDs_sync.end(),
                            KF_IDs_sync.back());

            if (it_frame_IDs_history != frame_IDs_sync.end())
            {
                auto it_frame_images = frame_ptrs_sync.begin();
                advance(it_frame_images, std::distance(frame_IDs_sync.begin(), it_frame_IDs_history));
                KF_ptrs_sync.push_back(*it_frame_images);


                auto it_frame_timestamps = frame_timestamps_sync.begin();
                advance(it_frame_timestamps, std::distance(frame_IDs_sync.begin(), it_frame_IDs_history));
                KF_timestamps_sync.push_back(*it_frame_timestamps);

                // Erase frame IDs, timestamps and image ptrs that are older than the KF ID:
                frame_IDs_sync.erase(frame_IDs_sync.begin(), it_frame_IDs_history);
                frame_ptrs_sync.erase(frame_ptrs_sync.begin(), it_frame_images);

                frame_timestamps_sync.erase(frame_timestamps_sync.begin(), it_frame_timestamps);
            }
            else
            {
                std::cout << "Error: KF ID does not exist in frame ID list !!!" << std::endl;
                std::cout << "KF_IDs_sync.back() = " << KF_IDs_sync.back() << std::endl;
                std::cout << "frame_IDs_sync.front() = " << frame_IDs_sync.front() << std::endl;
                std::cout << "frame_IDs_sync.back() = " << frame_IDs_sync.back() << std::endl;
                assert(it_frame_IDs_history != frame_IDs_sync.end());
            }
        }


        if (!marginalized_KF_IDs_temp.empty()
                && !marginalized_KF_poses_temp.empty()
                && !marginalized_KF_calib_temp.empty()
                && !marginalized_KF_points_temp.empty())
        {
            marginalized_KF_IDs_sync.push_back(marginalized_KF_IDs_temp.front());
            marginalized_KF_IDs_temp.erase(marginalized_KF_IDs_temp.begin());

            marginalized_KF_calib_sync.push_back(marginalized_KF_calib_temp.front());
            marginalized_KF_calib_temp.erase(marginalized_KF_calib_temp.begin());

            marginalized_KF_poses_sync.push_back(marginalized_KF_poses_temp.front());
            marginalized_KF_poses_temp.erase(marginalized_KF_poses_temp.begin());

            marginalized_KF_points_sync.push_back(marginalized_KF_points_temp.front());
            marginalized_KF_points_temp.erase(marginalized_KF_points_temp.begin());
        }

//        assert(KF_IDs_sync.size() == KF_timestamps_sync.size());
//        assert(KF_IDs_sync.size() == KF_ptrs_sync.size());

        auto it_marginalized_ID = std::find(
                        marginalized_KF_IDs_sync.begin(),
                        marginalized_KF_IDs_sync.end(),
                        KF_IDs_sync.front());

        if (it_marginalized_ID != marginalized_KF_IDs_sync.end())
        {
            marginalized_KF_ID = KF_IDs_sync.front();

            double marginalized_timestamp = KF_timestamps_sync.front();
            std::vector<float> marginalized_img;

            std::vector<float> marginalized_img_full(KF_ptrs_sync.front()->image,
                            KF_ptrs_sync.front()->image + image_h*image_w);
            marginalized_img = marginalized_img_full;

            if ( marginalized_img.size() != image_h*image_w)
                assert(marginalized_img.size() == image_h*image_w);

            // Since we cannot use the same iterator for different vector:
            auto iterator_calib = marginalized_KF_calib_sync.begin();
            advance(iterator_calib, std::distance(marginalized_KF_IDs_sync.begin(), it_marginalized_ID));
            std::vector<float> marginalized_calib = *iterator_calib;

            auto iterator_pose = marginalized_KF_poses_sync.begin();
            advance(iterator_pose, std::distance(marginalized_KF_IDs_sync.begin(), it_marginalized_ID));
            std::vector<float> marginalized_pose = *iterator_pose;

            auto iterator_points = marginalized_KF_points_sync.begin();
            advance(iterator_points, std::distance(marginalized_KF_IDs_sync.begin(), it_marginalized_ID));
            std::vector<float> marginalized_points = *iterator_points;

            // Publish the marginalized keyframe info:
            // 1st element: timestamp
            // 2nd & 3rd element: h & w
            // Next elements: image
            // Next elements: calibration (fx,fy,cx,cy)
            // Next elements: pose
            // Next elements: points (x,y,z,idepth_var_relative)
            std_msgs::Float64MultiArray marginalized_KF_time_image_calib_pose_points_msg;
            marginalized_KF_time_image_calib_pose_points_msg.data.push_back(marginalized_timestamp);

            marginalized_KF_time_image_calib_pose_points_msg.data.push_back(image_h);
            marginalized_KF_time_image_calib_pose_points_msg.data.push_back(image_w);
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                    marginalized_img.begin(),
                    marginalized_img.end());
            assert(marginalized_KF_time_image_calib_pose_points_msg.data.size() == image_h*image_w + 3);
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                                marginalized_calib.begin(),
                                marginalized_calib.end());
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                    marginalized_pose.begin(),
                    marginalized_pose.end());
            marginalized_KF_time_image_calib_pose_points_msg.data.insert(
                    marginalized_KF_time_image_calib_pose_points_msg.data.end(),
                    marginalized_points.begin(),
                    marginalized_points.end());
            marginalized_KF_time_image_calib_pose_points_pub.publish(marginalized_KF_time_image_calib_pose_points_msg);

            // Erase:
            KF_IDs_sync.erase(KF_IDs_sync.begin());
            KF_timestamps_sync.erase(KF_timestamps_sync.begin());
            KF_ptrs_sync.erase(KF_ptrs_sync.begin());
            marginalized_KF_IDs_sync.erase(it_marginalized_ID);
            marginalized_KF_calib_sync.erase(iterator_calib);
            marginalized_KF_poses_sync.erase(iterator_pose);
            marginalized_KF_points_sync.erase(iterator_points);

            wait_counter = 0;
        }
    }
}


void dso_ros::DsoNode::initParams()
{
    bool debug = nh_private.param<bool>("debug", false);
    if (debug) {
     dso::setting_debugout_runquiet = false;
     dso::setting_logStuff = true;
    } else {
     dso::setting_debugout_runquiet = true;
     dso::setting_logStuff = false;
    }

    nh_private.param<bool>("display_GUI", display_GUI, false);
    nh_private.param<double>("playback_speed", playbackSpeed, 1);
    nh_private.param<std::string>("image_file_path", image_file, "");
    nh_private.param<std::string>("calib_file_path", calib_file, "");
    nh_private.param<std::string>("vignette_file_path", vignette_file, "");
    nh_private.param<std::string>("gamma_file_path", gamma_file, "");
    nh_private.param<std::string>("rectified_file_path", rectified_file, "");
    nh_private.param<std::string>("stats_file_path", stats_file, "");
    nh_private.param<int>("mode", mode, 1);
    nh_private.param<int>("dataset", dataset, 1);
    nh_private.param<int>("sequence", sequence, 1);

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    std::cout << "     mode = " << mode << std::endl;

    dso::setting_semiDirect = false;
    half_resolution = false;
    preRectification = false;

    if (mode == 1)
    {
       std::cout << "     Run my system" << std::endl;
       dso::setting_semiDirect = true;
       half_resolution = true;
    }
    else if (mode == 2)
    {
       std::cout << "     Gnerate Rectified full Resolution Images, sequence = " << sequence <<std::endl;
    }
    else if (mode == 3)
    {
       std::cout << "     Gnerate Rectified half Resolution Images, sequence = " << sequence << std::endl;
       half_resolution = true;
    }

    if (dataset == 2 && mode == 1)
    {
       std::cout << "     Do Photometric Calibration !" << std::endl;
       dso::setting_photometricCalibration = 2;
    }
    else
    {
       std::cout << "     Don't Do Photometric Calibration !" << std::endl;
       dso::setting_photometricCalibration = 0;
       dso::setting_affineOptModeA = 0;
       dso::setting_affineOptModeB = 0;
    }

    std::cout << "     playback speed = x" << playbackSpeed << std::endl;

    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


}

sensor_msgs::ImageConstPtr dso_ros::DsoNode::ConvertImageAndExposure2ImagePtr(std::shared_ptr<dso::ImageAndExposure> input, bool half_resolution)
{
    cv::Mat image_cv;

    if (half_resolution)
        image_cv = cv::Mat(input->h_half, input->w_half, CV_32FC1, input->image_half)*(1/254.0f);
    else
        image_cv = cv::Mat(input->h, input->w, CV_32FC1, input->image)*(1/254.0f);

    image_cv.convertTo(image_cv, CV_8UC1, 255.0f);
    std_msgs::Header header;

    header.stamp = ros::Time(input->timestamp);
    cv_bridge::CvImage bridge_img(header, "mono8", image_cv);



    return bridge_img.toImageMsg();
}


void dso_ros::DsoNode::reset()
{
    printf("RESETTING!\n");
    setting_fullResetRequested=false;

    std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
    delete fullSystem;

    for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

    fullSystem = new FullSystem();
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = (playbackSpeed==0);

    fullSystem->outputWrapper = wraps;

    nSkippedFrames = 0;
    nTotalFramesAfterInit = 0;
    trackingStartTimestamp = 0;
    trackingTimes.clear();

    KF_IDs_temp.clear();

    KF_IDs_sync.clear();
    KF_timestamps_sync.clear();
    KF_ptrs_sync.clear();

    marginalized_KF_IDs_temp.clear();
    marginalized_KF_IDs_sync.clear();

    marginalized_KF_points_temp.clear();
    marginalized_KF_points_sync.clear();
    marginalized_points_temp.clear();

    marginalized_KF_calib_temp.clear();
    marginalized_KF_calib_sync.clear();

    marginalized_KF_poses_temp.clear();
    marginalized_KF_poses_sync.clear();

    reset_count_msg.data ++;
    reset_count_pub.publish(reset_count_msg);

    total_pub_count = 0;

    {
        std::unique_lock<std::mutex> lock(frame_sync_mtx);
        frame_IDs_sync.clear();
        frame_timestamps_sync.clear();
        frame_ptrs_sync.clear();
    }

}



