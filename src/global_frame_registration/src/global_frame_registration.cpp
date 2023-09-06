#include <fstream>
#include <global_frame_registration/global_frame_registration.hpp>
#include <global_frame_registration/quat_averaging.hpp>

GlobalFrameRegistrator::GlobalFrameRegistrator(ros::NodeHandle& n) : nh(n)
{
    if (!nh.getParam("tool_subscribers", tool_subs_topic)) {
        ROS_ERROR_STREAM("Unable to load tool_subscribers param");
    }

    if (!nh.getParam("camera_subscribers", camera_subs_topic)) {
        ROS_ERROR_STREAM("Unable to load camera_subscribers");
    }

    if (!nh.getParam("rgbd_subscribers", rgbd_subs_topic)) {
        ROS_ERROR_STREAM("Unable to load rgbd_subscribers");
    }

    if (!nh.getParam("global_frame_name", global_frame_name)) {
        ROS_ERROR_STREAM("Unable to load global_frame_name");
    }

    if (!nh.getParam("num_points", num_points)) {
        ROS_ERROR_STREAM("Unable to load num_points");
    }

    if (!nh.getParam("use_existing", use_existing)) {
        ROS_ERROR_STREAM("Unable to load use_existing");
    }

    if (!nh.getParam("eye_in_hand", eye_in_hand)) {
        ROS_ERROR_STREAM("Unable to load eye_in_hand");
    }

    if (!nh.getParam("camera_color_frame", camera_color_frame)) {
        ROS_ERROR_STREAM("Unable to load camera_color_frame");
    }

    if (!nh.getParam("camera_ref_frame", camera_ref_frame)) {
        ROS_ERROR_STREAM("Unable to load camera_ref_frame");
    }

    if (!nh.getParam("camera_ref_base_frame", camera_ref_base_frame)) {
        ROS_ERROR_STREAM("Unable to load camera_ref_base_frame");
    }

    if (use_existing) {
        if (!nh.getParam("registrations", registration_data)) {
            ROS_ERROR_STREAM("Unable to load registrations");
        }
    } else {
        for (std::string& topic : tool_subs_topic) {
            ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>(
                topic.c_str(),
                1,
                std::bind(&GlobalFrameRegistrator::toolPoseUpdate, this, std::placeholders::_1, topic));
            pose_subs.push_back(sub);
        }
        for (std::string& topic : rgbd_subs_topic) {
            ros::Subscriber sub = n.subscribe<geometry_msgs::PoseArray>(
                topic.c_str(),
                1,
                std::bind(&GlobalFrameRegistrator::arrayPoseUpdate, this, std::placeholders::_1, topic));
            pose_subs.push_back(sub);
        }
        for (std::string& topic : camera_subs_topic) {
            ros::Subscriber sub = n.subscribe<aruco_msgs::MarkerArray>(
                topic.c_str(),
                1,
                std::bind(&GlobalFrameRegistrator::cameraPoseUpdate, this, std::placeholders::_1, topic));
            pose_subs.push_back(sub);
        }
    }
}

void GlobalFrameRegistrator::arrayPoseUpdate(const geometry_msgs::PoseArray::ConstPtr& msg, std::string topicname)
{
    mtx.lock();
    auto it = rgbd_poses.find(topicname);
    if (it != rgbd_poses.end()) {
        it->second.header = msg->header;
        it->second.poses  = msg->poses;
    }

    rgbd_poses[topicname].header = msg->header;
    rgbd_poses[topicname].poses  = msg->poses;
    mtx.unlock();
}

void GlobalFrameRegistrator::toolPoseUpdate(const geometry_msgs::PoseStamped::ConstPtr& msg, std::string topicname)
{
    mtx.lock();

    auto it = tool_poses.find(topicname);
    if (it != tool_poses.end()) {
        it->second.header = msg->header;
        it->second.pose   = msg->pose;
    }

    tool_poses[topicname].header = msg->header;
    tool_poses[topicname].pose   = msg->pose;

    mtx.unlock();
}

void GlobalFrameRegistrator::cameraPoseUpdate(const aruco_msgs::MarkerArray::ConstPtr& msg, std::string topicname)
{
    mtx.lock();

    aruco_msgs::MarkerArray m_arr;
    m_arr.markers = msg->markers;
    m_arr.header  = msg->header;

    // sorting the markers by identification number (ascending)
    std::sort(m_arr.markers.begin(), m_arr.markers.end(), [](const aruco_msgs::Marker& a, const aruco_msgs::Marker& b) {
        return a.id < b.id;
    });

    // if the topicname has already been added to the map
    // clear the old poses and update it with the new ones
    auto it = cam_poses.find(topicname);
    if (it != cam_poses.end()) {
        it->second.clear();

        for (size_t i = 0; i < m_arr.markers.size(); i++) {
            geometry_msgs::PoseStamped p;
            p.header = m_arr.header;
            p.pose   = m_arr.markers[i].pose.pose;
            it->second.push_back(p);
        }
    } else {
        // or create the entry ad fill it with the data
        for (size_t i = 0; i < m_arr.markers.size(); i++) {
            geometry_msgs::PoseStamped p;
            p.header = m_arr.header;
            p.pose   = m_arr.markers[i].pose.pose;
            cam_poses[topicname].push_back(p);
        }
    }

    mtx.unlock();
}

std::pair<tf::Vector3, tf::Vector3>
GlobalFrameRegistrator::bestPlaneFromPoints(const std::vector<geometry_msgs::PoseStamped>& c)
{
    // copy coordinates to matrix in Eigen format
    size_t                                                                 num_atoms = c.size();
    Eigen::Matrix<Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i) {
        coord.col(i)[0] = c[i].pose.position.x;
        coord.col(i)[1] = c[i].pose.position.y;
        coord.col(i)[2] = c[i].pose.position.z;
    }

    // calculate centroid
    Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    // subtract centroid
    coord.row(0).array() -= centroid(0);
    coord.row(1).array() -= centroid(1);
    coord.row(2).array() -= centroid(2);

    // we only need the left-singular matrix here
    // minimal ortho distance between point set and plane
    // http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
    auto            svd          = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3f plane_normal = svd.matrixU().rightCols<1>();

    tf::Vector3 center(centroid.x(), centroid.y(), centroid.z());
    tf::Vector3 normal(plane_normal.x(), plane_normal.y(), plane_normal.z());

    return std::make_pair(center, normal.normalize());
}

std::pair<tf::Vector3, tf::Quaternion>
GlobalFrameRegistrator::bestPlaneFromPlanes(const std::vector<geometry_msgs::PoseStamped>& c)
{
    // copy coordinates to  matrix in Eigen format
    size_t                                                                 num_atoms = c.size();
    Eigen::Matrix<Eigen::Vector3f::Scalar, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
    for (size_t i = 0; i < num_atoms; ++i) {
        coord.col(i)[0] = c[i].pose.position.x;
        coord.col(i)[1] = c[i].pose.position.y;
        coord.col(i)[2] = c[i].pose.position.z;
    }

    // calculate centroid
    Eigen::Vector3f centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

    std::vector<tf::Quaternion> q_list;
    for (auto& pose : c) {
        q_list.push_back(tf::Quaternion(
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w));
    }

    tf::Vector3 center(centroid.x(), centroid.y(), centroid.z());
    auto        orientation = quat_averaging::avg(q_list);

    return std::make_pair(center, quat_averaging::avg(q_list));
}

void GlobalFrameRegistrator::initialize()
{
    if (use_existing) {
        t = new std::thread([&]() {
            loadRegistration();
            broadcastTransform();
        });
    } else {
        t = new std::thread([&]() {
            std::unordered_map<std::string, std::vector<geometry_msgs::PoseStamped>> read_tool_poses, read_cam_poses;
            std::unordered_map<std::string, std::vector<geometry_msgs::PoseArray>>   read_rgbd_poses;
            std::unordered_map<std::string, geometry_msgs::PoseStamped>              read_versor;

            for (size_t i = 0; i < tool_subs_topic.size(); i++) {
                ROS_INFO("Reading position from topic: %s", tool_subs_topic[i].c_str());
                for (size_t j = 0; j < num_points; j++) {
                    ROS_INFO_STREAM("Position your arm to position " << j + 1 << " of " << num_points
                                                                     << " and press 'Enter'");
                    getchar();

                    // get values from callback and store it
                    mtx.lock();
                    read_tool_poses[tool_subs_topic[i]].push_back(tool_poses[tool_subs_topic[i]]);
                    ROS_INFO("Pose: [%d/%d]", (int) (j + 1), num_points);
                    ROS_INFO_STREAM(tool_poses[tool_subs_topic[i]]);
                    mtx.unlock();
                }

                ROS_INFO("Position your arm over the calibration board and press 'Enter'");
                getchar();

                // get values from callback anread_array_versord store it
                mtx.lock();
                read_versor[tool_subs_topic[i]] = tool_poses[tool_subs_topic[i]];
                ROS_INFO("Surface versor:");
                ROS_INFO_STREAM(tool_poses[tool_subs_topic[i]]);
                mtx.unlock();
            }

            for (size_t i = 0; i < rgbd_subs_topic.size(); i++) {

                ROS_INFO("Reading position from topic: %s. PRESS ENTER", rgbd_subs_topic[i].c_str());
                getchar();
                ROS_INFO_STREAM(" size " << rgbd_poses[rgbd_subs_topic[i].c_str()].poses.size());
                mtx.lock();
                if (rgbd_poses[rgbd_subs_topic[i]].poses.size() > 0) {
                    read_rgbd_poses[rgbd_subs_topic[i]].push_back(rgbd_poses[rgbd_subs_topic[i]]);

                    for (size_t j = 0; j < rgbd_poses[rgbd_subs_topic[i]].poses.size() - 1; j++) {
                        geometry_msgs::PoseStamped pose_stamped;
                        pose_stamped.header.frame_id = rgbd_poses[rgbd_subs_topic[i]].header.frame_id;
                        pose_stamped.pose            = rgbd_poses[rgbd_subs_topic[i]].poses.at(j);
                        read_tool_poses[rgbd_subs_topic[i]].push_back(pose_stamped);
                    }

                    geometry_msgs::PoseStamped versorPose;
                    versorPose.pose =
                        rgbd_poses[rgbd_subs_topic[i]].poses.at(rgbd_poses[rgbd_subs_topic[i]].poses.size() - 1);
                    read_versor[rgbd_subs_topic[i]] = versorPose;
                }
                mtx.unlock();
            }

            for (size_t i = 0; i < camera_subs_topic.size(); i++) {
                ROS_INFO("Reading marker position from topic: %s", camera_subs_topic[i].c_str());
                ROS_INFO("Position the camera over the markers and press 'Enter'");
                getchar();

                // get values from callback and store it
                mtx.lock();
                read_cam_poses[camera_subs_topic[i]] = cam_poses[camera_subs_topic[i]];
                mtx.unlock();

                while (read_cam_poses[camera_subs_topic[i]].size() != 4) {
                    ROS_ERROR("Only %d of 4 markers are visible. Check your calibration board is completly visible.",
                              (int) read_cam_poses[camera_subs_topic[i]].size());
                    ROS_INFO("Position the camera over the markers and press 'Enter'");
                    getchar();

                    // get values from callback and store it
                    mtx.lock();
                    read_cam_poses[camera_subs_topic[i]] = cam_poses[camera_subs_topic[i]];
                    ROS_INFO_STREAM(camera_subs_topic[i]);
                    mtx.unlock();
                }
            }

            for (size_t i = 0; i < tool_subs_topic.size(); i++) {
                std::string          name      = tool_subs_topic[i].c_str();
                std::string          ref_frame = read_tool_poses[name][0].header.frame_id;
                geometry_msgs::Point versor    = read_versor[name].pose.position;

                computeReferenceSystem(name, ref_frame, read_tool_poses[name], versor);
            }

            for (size_t i = 0; i < rgbd_subs_topic.size(); i++) {
                if (rgbd_poses[rgbd_subs_topic[i]].poses.size() > 0) {
                    std::string          name      = rgbd_subs_topic[i].c_str();
                    std::string          ref_frame = read_tool_poses[name][0].header.frame_id;
                    geometry_msgs::Point versor    = read_versor[name].pose.position;

                    computeReferenceSystem(name, ref_frame, read_tool_poses[name], versor);
                }
            }

            for (size_t i = 0; i < camera_subs_topic.size(); i++) {
                std::string name      = camera_subs_topic[i].c_str();
                std::string ref_frame = read_cam_poses[name][0].header.frame_id;

                ROS_INFO("Calibrating world frame for topic: %s [%s]", name.c_str(), ref_frame.c_str());

                tf::StampedTransform transform;

                ROS_INFO_STREAM("Markers");
                for (auto& p : read_cam_poses[name]) {
                    std::cout << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << std::endl;
                }
                std::cout << "----------------------------\n";

                // find the interpolated plane between the set of points
                auto        plane      = bestPlaneFromPoints(read_cam_poses[name]);
                tf::Vector3 center     = plane.first;
                tf::Vector3 normal_vec = plane.second;

                ROS_INFO_STREAM("Plane center" << std::endl);
                std::cout << center.x() << " " << center.y() << " " << center.z() << std::endl;
                ROS_INFO_STREAM("Plane normal" << std::endl);
                std::cout << normal_vec.x() << " " << normal_vec.y() << " " << normal_vec.z() << std::endl;

                // define the forward direction on the calibration board (as the vector pointing for the
                // lower left marker to the upper left one)
                tf::Vector3 forward_plane_vec(
                    read_cam_poses[name][0].pose.position.x - read_cam_poses[name][3].pose.position.x,
                    read_cam_poses[name][0].pose.position.y - read_cam_poses[name][3].pose.position.y,
                    read_cam_poses[name][0].pose.position.z - read_cam_poses[name][3].pose.position.z);

                forward_plane_vec = forward_plane_vec.normalized();

                ROS_INFO_STREAM("Marker forward" << std::endl);
                std::cout << forward_plane_vec.x() << " " << forward_plane_vec.y() << " " << forward_plane_vec.z()
                          << std::endl;

                // define the left direction on the calibration board (as the vector pointing for the
                // upper left marker to the upper right one)
                tf::Vector3 left_plane_vec(
                    read_cam_poses[name][2].pose.position.x - read_cam_poses[name][3].pose.position.x,
                    read_cam_poses[name][2].pose.position.y - read_cam_poses[name][3].pose.position.y,
                    read_cam_poses[name][2].pose.position.z - read_cam_poses[name][3].pose.position.z);

                left_plane_vec = left_plane_vec.normalized();

                ROS_INFO_STREAM("Marker left" << std::endl);
                std::cout << left_plane_vec.x() << " " << left_plane_vec.y() << " " << left_plane_vec.z() << std::endl;

                // getting the upper versor
                auto up_plane_vec = forward_plane_vec.cross(left_plane_vec).normalized();

                ROS_INFO_STREAM("Marker up" << std::endl);
                std::cout << up_plane_vec.x() << " " << up_plane_vec.y() << " " << up_plane_vec.z() << std::endl;

                // checking if the normal of plane points towards the same direction of the up vector
                // built with the plane directions defines with marker disposition
                if (normal_vec.dot(up_plane_vec) < 0) {
                    ROS_INFO("Flipping the normal");
                    normal_vec = -normal_vec;
                }

                // building the left-ish vector as cross product of the up vector (the normal) and the
                // forward one
                left_plane_vec = normal_vec.cross(forward_plane_vec).normalized();

                // recompute the real forward versor on the plane
                forward_plane_vec = left_plane_vec.cross(normal_vec).normalized();

                ROS_INFO_STREAM("Ortho forward" << std::endl);
                std::cout << forward_plane_vec.x() << " " << forward_plane_vec.y() << " " << forward_plane_vec.z()
                          << std::endl;

                ROS_INFO_STREAM("Ortho left" << std::endl);
                std::cout << left_plane_vec.x() << " " << left_plane_vec.y() << " " << left_plane_vec.z() << std::endl;

                // building the rotation matrix and traspose it to represent the trasformation between the
                // base and the calibration board. The new basis is expressed by the following rotation matrix
                // where columns are the new basis vector
                tf::Matrix3x3 rot_m(forward_plane_vec.x(),
                                    left_plane_vec.x(),
                                    normal_vec.x(),
                                    forward_plane_vec.y(),
                                    left_plane_vec.y(),
                                    normal_vec.y(),
                                    forward_plane_vec.z(),
                                    left_plane_vec.z(),
                                    normal_vec.z());

                tf::Quaternion rotation;
                rot_m.getRotation(rotation);

                // building transformation
                // setting the center of the transformation the center of the calibration board
                // and the rotation too
                transform.setOrigin(center);
                transform.setRotation(rotation);

                transform = tf::StampedTransform(transform.inverse(), ros::Time::now(), global_frame_name, ref_frame);
                calib_transform[ref_frame] = transform;

                // debug
                auto rot_mi = tf::Matrix3x3(transform.getRotation());
                ROS_INFO_STREAM("Transformation with regards to " << global_frame_name << std::endl);
                ROS_INFO_STREAM("Center" << std::endl);
                ROS_INFO_STREAM(transform.getOrigin().x()
                                << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl);
                ROS_INFO_STREAM("Forward vector [x]" << std::endl);
                ROS_INFO_STREAM(rot_mi.getRow(0).x()
                                << " " << rot_mi.getRow(0).y() << " " << rot_mi.getRow(0).z() << std::endl);
                ROS_INFO_STREAM("Left vector [y]" << std::endl);
                ROS_INFO_STREAM(rot_mi.getRow(1).x()
                                << " " << rot_mi.getRow(1).y() << " " << rot_mi.getRow(1).z() << std::endl);
                ROS_INFO_STREAM("Normal vector [z]" << std::endl);
                ROS_INFO_STREAM(rot_mi.getRow(2).x()
                                << " " << rot_mi.getRow(2).y() << " " << rot_mi.getRow(2).z() << std::endl);

                // incremental output (registration)
                // storeRegistration();
            }

            broadcastTransform();
        });
    }
}

void GlobalFrameRegistrator::broadcastTransform()
{
    static tf::TransformBroadcaster br;

    if (eye_in_hand && !use_existing) {
        tf::Transform         cam2world, ecmBase2world;
        tf::StampedTransform  ecm2ecmBase;
        tf::TransformListener tfListener;

        for (auto& t : calib_transform) {
            if (t.first == camera_color_frame) {
                cam2world = t.second;
            }

            if (t.first == camera_ref_base_frame) {
                ecmBase2world = t.second;
            }
        }
        try {
            tfListener.waitForTransform(camera_ref_base_frame, camera_ref_frame, ros::Time(0), ros::Duration(10));
            tfListener.lookupTransform(camera_ref_base_frame, camera_ref_frame, ros::Time(0), ecm2ecmBase);

            if (calib_transform.find(camera_color_frame) != calib_transform.end()) {
                calib_transform[camera_color_frame] =
                    tf::StampedTransform((cam2world.inverse() * ecmBase2world * ecm2ecmBase).inverse(),
                                         ros::Time::now(),
                                         camera_ref_frame,
                                         camera_color_frame);
            }
        }
        catch (std::exception& e) {
            ROS_ERROR("%s", e.what());
        }
    }

    storeRegistration(calib_transform);

    while (1) {
        auto current_time = ros::Time::now();

        for (auto& t : calib_transform) {
            t.second.stamp_          = current_time;
            t.second.child_frame_id_ = t.first;
            br.sendTransform(t.second);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void GlobalFrameRegistrator::storeRegistration(std::unordered_map<std::string, tf::StampedTransform> calib)
{
    XmlRpc::XmlRpcValue outdata;

    int i = 0;
    for (auto& t : calib) {
        outdata[i]["frame_id"] = t.first;
        if (t.first == camera_color_frame) {
            outdata[i]["parent_frame"] = camera_ref_frame;
        } else {
            outdata[i]["parent_frame"] = global_frame_name;
        }

        outdata[i]["position"][0] = t.second.getOrigin().getX();
        outdata[i]["position"][1] = t.second.getOrigin().getY();
        outdata[i]["position"][2] = t.second.getOrigin().getZ();

        outdata[i]["orientation"][0] = t.second.getRotation().getX();
        outdata[i]["orientation"][1] = t.second.getRotation().getY();
        outdata[i]["orientation"][2] = t.second.getRotation().getZ();
        outdata[i]["orientation"][3] = t.second.getRotation().getW();

        i++;
    }

    nh.setParam("registrations", outdata);
}

void GlobalFrameRegistrator::loadRegistration()
{
    int num_frames = tool_subs_topic.size() + camera_subs_topic.size() + rgbd_subs_topic.size();
    for (size_t i = 0; i < num_frames; i++) {
        auto r_elem = registration_data[i];

        std::string frame_id     = (std::string) r_elem["frame_id"];
        std::string parent_frame = (std::string) r_elem["parent_frame"];

        std::vector<double> position;
        position.push_back(static_cast<double>(r_elem["position"][0]));
        position.push_back(static_cast<double>(r_elem["position"][1]));
        position.push_back(static_cast<double>(r_elem["position"][2]));

        std::vector<double> orientation;
        orientation.push_back(static_cast<double>(r_elem["orientation"][0]));
        orientation.push_back(static_cast<double>(r_elem["orientation"][1]));
        orientation.push_back(static_cast<double>(r_elem["orientation"][2]));
        orientation.push_back(static_cast<double>(r_elem["orientation"][3]));

        tf::StampedTransform transform;
        transform.setOrigin(tf::Vector3(position[0], position[1], position[2]));
        transform.setRotation(tf::Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]));

        transform.frame_id_ = parent_frame;

        calib_transform[frame_id] = transform;
    }

    ROS_INFO_STREAM("Registration loaded correctly");
}

void GlobalFrameRegistrator::computeReferenceSystem(std::string                             name,
                                                    std::string                             ref_frame,
                                                    std::vector<geometry_msgs::PoseStamped> board_points,
                                                    geometry_msgs::Point                    versor)
{
    ROS_INFO("Calibrating world frame for topic: %s [%s]", name.c_str(), ref_frame.c_str());

    // find the interpolated plane between the set of points
    auto plane      = bestPlaneFromPoints(board_points);
    auto normal_vec = plane.second;
    auto center     = plane.first;

    // checking if the normal of plane points towards the same direction of the read versor
    auto up_vector = (tf::Vector3(versor.x, versor.y, versor.z) - center).normalized();

    if (normal_vec.dot(up_vector) < 0) {
        ROS_INFO("Flipping the normal");
        normal_vec = -normal_vec;
    }

    // setting the center of the transformation the center of the calibration board
    tf::StampedTransform transform;
    transform.setOrigin(center);

    // building the rotation matrix of the plane w.r.t the tool reference frame
    tf::Quaternion q;

    // the forward direction of the calibration board is set as the direction from the
    // center to the first read point
    tf::Vector3 forward_plane_vec(board_points[0].pose.position.x - center.x(),
                                  board_points[0].pose.position.y - center.y(),
                                  board_points[0].pose.position.z - center.z());
    forward_plane_vec = forward_plane_vec.normalized();

    // building the left-ish vector as cross product of the up vector (the normal) and the
    // forward one
    auto left_plane_vec = normal_vec.cross(forward_plane_vec).normalized();

    // recompute the real forward versor on the plane
    forward_plane_vec = left_plane_vec.cross(normal_vec).normalized();

    // building the rotation matrix and traspose it to represent the trasformation between the
    // base and the calibration board. The new basis is expressed by the following rotation matrix
    // where columns are the new basis vector
    tf::Matrix3x3 rot_m(forward_plane_vec.x(),
                        left_plane_vec.x(),
                        normal_vec.x(),
                        forward_plane_vec.y(),
                        left_plane_vec.y(),
                        normal_vec.y(),
                        forward_plane_vec.z(),
                        left_plane_vec.z(),
                        normal_vec.z());

    // setting up rotation matrix
    rot_m.getRotation(q);
    transform.setRotation(q);

    // compute the inverse transform (from world frame to the arm frame)
    transform = tf::StampedTransform(transform.inverse(), ros::Time::now(), global_frame_name, ref_frame);
    calib_transform[ref_frame] = transform;

    // debug
    tf::Matrix3x3 rot_mi(transform.getRotation());
    ROS_INFO_STREAM("Transformation with regards to " << global_frame_name << std::endl);
    ROS_INFO_STREAM("Center" << std::endl);
    ROS_INFO_STREAM(transform.getOrigin().x()
                    << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl);
    ROS_INFO_STREAM("Forward vector [x]" << std::endl);
    ROS_INFO_STREAM(rot_mi.getRow(0).x() << " " << rot_mi.getRow(0).y() << " " << rot_mi.getRow(0).z() << std::endl);
    ROS_INFO_STREAM("Left vector [y]" << std::endl);
    ROS_INFO_STREAM(rot_mi.getRow(1).x() << " " << rot_mi.getRow(1).y() << " " << rot_mi.getRow(1).z() << std::endl);
    ROS_INFO_STREAM("Normal vector [z]" << std::endl);
    ROS_INFO_STREAM(rot_mi.getRow(2).x() << " " << rot_mi.getRow(2).y() << " " << rot_mi.getRow(2).z() << std::endl);

    // incremental output (registration)
    // storeRegistration();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_frame_registration");
    ros::NodeHandle n("~");

    GlobalFrameRegistrator gfr(n);
    gfr.initialize();

    ros::spin();
    return 0;
}
