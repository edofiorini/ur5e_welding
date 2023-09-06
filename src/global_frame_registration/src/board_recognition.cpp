#include <global_frame_registration/board_recognition.hpp>

BoardCalibration::BoardCalibration(ros::NodeHandle nh) : nh_(nh), private_nh_("~")
{
    private_nh_.param("color_camera", image_topic, std::string(""));
    private_nh_.param("depth_camera", depth_topic, std::string(""));
    private_nh_.param("pose_array", pose_array_topic, std::string(""));
    private_nh_.param("cam_info", cam_info_topic, std::string(""));
    private_nh_.param("tof_3d", tof_3d, false);
    private_nh_.param("radius_circle", radius, 40);

    image_sub_       = nh_.subscribe(image_topic, 1, &BoardCalibration::imageCb, this);
    image_depth_sub_ = nh_.subscribe(depth_topic, 1, &BoardCalibration::imageDepthCb, this);
    camera_info_sub_ = nh_.subscribe(cam_info_topic, 1, &BoardCalibration::camInfoCb, this);
    pose_array_pub   = nh_.advertise<geometry_msgs::PoseArray>(pose_array_topic, 1);

    image_transport::ImageTransport it_(nh);
    pub_rgb  = it_.advertise("board_recognition/points", 1);
    pub_mask = it_.advertise("board_recognition/mask", 1);

    ms = cv::MSER::create();
}

BoardCalibration::~BoardCalibration()
{
    delete ms;
}

void BoardCalibration::camInfoCb(const sensor_msgs::CameraInfoPtr& msg)
{
    camera_color_frame = msg->header.frame_id;
    cam_info.K         = msg->K;
}

void BoardCalibration::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr    = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        image_rgb = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void BoardCalibration::imageDepthCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        image_depth = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

std_msgs::Header BoardCalibration::makeHeader(std::string id)
{
    std_msgs::Header header;
    header.frame_id = id;
    header.seq      = 0;
    header.stamp    = ros::Time::now();
    return header;
}

void BoardCalibration::PublishRenderedImage(image_transport::Publisher& pubImage,
                                            cv::Mat&                    image,
                                            std::string                 encoding,
                                            std_msgs::Header            header)
{
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    pubImage.publish(rendered_image_msg);
}

void BoardCalibration::findCircles(cv::Mat& img, int minArea, std::vector<cv::Point2f>& corners)
{
    cv::SimpleBlobDetector::Params params;
    params.filterByConvexity                 = true;
    params.minConvexity                      = 0.5f;
    params.minArea                           = minArea; // 350 rgb
    params.blobColor                         = 0;       // 0 black - 255 white
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint>       keypoints;
    detector->detect(img, keypoints);

    // for (auto& kp : keypoints) {
    // cv::drawMarker(img, cv::Point(kp.pt.x, kp.pt.y), cv::Scalar(255, 255, 255));
    // }
    cv::KeyPoint::convert(keypoints, corners);
    std::reverse(corners.begin(), corners.end());
}

void BoardCalibration::poseFromImage(cv::Mat                   image_depth,
                                     std::vector<cv::Point2f>  corners,
                                     float*                    K,
                                     geometry_msgs::PoseArray& pose_array)
{
    float cx     = K[2];
    float cy     = K[5];
    float fx_inv = 1.0 / K[0];
    float fy_inv = 1.0 / K[4];

    for (int i = 0; i < corners.size(); i++) {
        uint16_t z_raw = image_depth.at<uint16_t>(cv::Point(corners.at(i).x, corners.at(i).y));

        if (z_raw != 0.0) {
            float               z_metric = z_raw * 0.001;
            geometry_msgs::Pose pose;
            pose.position.x    = z_metric * ((corners.at(i).x - cx) * fx_inv);
            pose.position.y    = z_metric * ((corners.at(i).y - cy) * fy_inv);
            pose.position.z    = z_metric;
            pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
            pose.orientation.w                                           = 1;
            pose_array.poses.push_back(pose);
        }
    }
}

void BoardCalibration::poseFromImageToF(cv::Mat                   image_depth,
                                        std::vector<cv::Point2f>  corners,
                                        float*                    K,
                                        geometry_msgs::PoseArray& pose_array)
{
    float cx     = K[2];
    float cy     = K[5];
    float fx_inv = 1.0 / K[0];
    float fy_inv = 1.0 / K[4];

    for (int i = 0; i < corners.size(); i++) {
        auto z_raw = image_depth.at<float>(cv::Point(corners.at(i).x, corners.at(i).y));

        if (z_raw != 0.0) {
            float               z_metric = z_raw;
            geometry_msgs::Pose pose;
            pose.position.x    = z_metric * ((corners.at(i).x - cx) * fx_inv);
            pose.position.y    = z_metric * ((corners.at(i).y - cy) * fy_inv);
            pose.position.z    = z_metric;
            pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
            pose.orientation.w                                           = 1;
            pose_array.poses.push_back(pose);
        }
    }
}

void BoardCalibration::computeVersor(geometry_msgs::PoseArray& pose_array)
{
    float x, y, z;
    for (size_t i = 0; i < pose_array.poses.size(); i++) {
        x += pose_array.poses[i].position.x;
        y += pose_array.poses[i].position.y;
        z += pose_array.poses[i].position.z;
    }
    geometry_msgs::Pose pose;
    pose.position.x    = x / pose_array.poses.size();
    pose.position.y    = y / pose_array.poses.size();
    pose.position.z    = z / pose_array.poses.size() - 0.05;
    pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
    pose.orientation.w                                           = 1;
    pose_array.poses.push_back(pose);
}

void BoardCalibration::getMatDepthValue(cv::Mat& src, cv::Mat& dst)
{
    src.convertTo(dst, CV_32F, 1.0 / 65535.0f);
}

cv::Mat BoardCalibration::hsv_black_mask(cv::Mat color)
{
    cv::Mat HSV, hueMask_black;
    cv::cvtColor(color, HSV, CV_BGR2HSV);
    cv::inRange(HSV, cv::Scalar(0, 0, 0), cv::Scalar(180, 100, 100), hueMask_black);
    return hueMask_black;
}

cv::Mat BoardCalibration::hsv_blue_mask(cv::Mat color)
{
    cv::Mat HSV, hueMask_blue;
    cv::cvtColor(color, HSV, CV_BGR2HSV);
    cv::inRange(HSV, cv::Scalar(100, 70, 30), cv::Scalar(120, 255, 255), hueMask_blue);
    return hueMask_blue;
}

void BoardCalibration::detectRegions(cv::Mat& img, cv::Rect& roi)
{
    std::vector<std::vector<cv::Point>> regions;
    std::vector<cv::Rect>               mser_bbox;
    ms->setMinArea(1000);
    ms->detectRegions(img, regions, mser_bbox);
    int maxArea     = 0;
    int idx_maxArea = 0;
    if (regions.size() > 0) {
        for (int i = 0; i < regions.size(); i++) {
            // cv::rectangle(img, mser_bbox[i], cv::Scalar(255, 255, 255));
            int tempArea = mser_bbox[i].height * mser_bbox[i].width;
            if (maxArea < tempArea) {
                maxArea     = tempArea;
                idx_maxArea = i;
            }
        }
        // cv::rectangle(img, mser_bbox[idx_maxArea], cv::Scalar(255, 255, 255));
        roi = mser_bbox[idx_maxArea];
    }
}

void BoardCalibration::morphOperation(cv::Mat img, cv::Mat& morphMask)
{
    int     morph_elem = 2; // Element:\n 0: Rect - 1: Cross - 2: Ellipse"
    int     morph_size = 2;
    cv::Mat element    = getStructuringElement(
        morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));

    morphologyEx(img, morphMask, 2, element); // Closing: 3,  It is useful in closing small holes
}

void BoardCalibration::filterContour(cv::Mat& img, cv::Mat& mask)
{
    int                                 largest_area          = 0;
    int                                 largest_contour_index = 0;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i>              hierarchy;
    cv::findContours(img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    for (int i = 0; i < contours.size(); i++) {
        double a = cv::contourArea(contours[i], false);
        if (a > largest_area) {
            largest_area          = a;
            largest_contour_index = i;
        }
    }
    cv::Scalar color(255, 255, 255);
    //cv::drawContours(mask, contours, largest_contour_index, color, CV_FILLED, 8, hierarchy);
}

void BoardCalibration::update()
{
    if (!image_depth.empty() && !image_rgb.empty()) {

        cv::Mat hueMask;
        cv::Mat hueMask_temp = hsv_blue_mask(image_rgb);

        cv::Mat mask = cv::Mat::zeros(image_rgb.size(), CV_8U);
        filterContour(hueMask_temp, mask);

        std_msgs::Header header_mask = makeHeader("mask");
        PublishRenderedImage(pub_mask, mask, "mono8", header_mask);

        std::vector<cv::Point2f> corners_rgb;
        findCircles(mask, radius, corners_rgb);

        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = camera_color_frame;

        float KK[6] = { cam_info.K[0], cam_info.K[1], cam_info.K[2], cam_info.K[3], cam_info.K[4], cam_info.K[5] };

        if (tof_3d) {
            getMatDepthValue(image_depth, image_depth);
            poseFromImageToF(image_depth, corners_rgb, KK, poseArray);
        } else {
            poseFromImage(image_depth, corners_rgb, KK, poseArray);
        }

        computeVersor(poseArray);
        pose_array_pub.publish(poseArray);

        std_msgs::Header header_rgb = makeHeader("rgb");
        cv::Mat          img2pub    = image_rgb.clone();

        for (int i = 0; i < corners_rgb.size(); i++) {
            cv::putText(
                img2pub, std::to_string(i), corners_rgb.at(i), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
            cv::drawMarker(img2pub, cv::Point(corners_rgb.at(i).x, corners_rgb.at(i).y), cv::Scalar(0, 255, 0));
        }

        PublishRenderedImage(pub_rgb, img2pub, "bgr8", header_rgb);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "board_calibration");
    ros::NodeHandle nh;

    BoardCalibration node(nh);

    while (ros::ok()) {
        node.update();
        ros::spinOnce();
    }

    return 0;
}
