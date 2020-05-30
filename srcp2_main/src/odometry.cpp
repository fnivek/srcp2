#include "srcp2_main/odometry.h"

void SampleFilter_init(SampleFilter* f)
{
    int i;
    for (i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
        f->history[i] = 0;
    f->last_index = 0;
}

void SampleFilter_put(SampleFilter* f, double input)
{
    f->history[f->last_index++] = input;
    if (f->last_index == SAMPLEFILTER_TAP_NUM)
        f->last_index = 0;
}

double SampleFilter_get(SampleFilter* f)
{
    double acc = 0;
    int index = f->last_index, i;
    for (i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    {
        index = index != 0 ? index - 1 : SAMPLEFILTER_TAP_NUM - 1;
        acc += f->history[index] * filter_taps[i];
    };
    return acc;
}

/**
 * @brief      Constructs a new instance.
 *
 * @param[in]  robot_name  The robot name
 */
Odometry::Odometry(std::string robot_name)
  : robot_name_(robot_name)
  , pos_(0, 0, 0)
  , vel_(0, 0, 0)
  , head_(1, 0, 0, 0)
  , accel_bias_(0, 0, 0)
  , ang_vel_bias_(0, 0, 0)
  , grav_(0, 0, 1.62)
  , last_time_(0)
  , last_laser_tf_(Eigen::Affine3d::Identity())
{
    err_cov_ = Eigen::Matrix<float, 18, 18>::Zero();
    // Debug filters
    SampleFilter_init(&debug_filt_x_);
    SampleFilter_init(&debug_filt_y_);
    SampleFilter_init(&debug_filt_z_);
}

/**
 * @brief      Delayed initialization
 *
 *             Subscribes to the imu and joint states associated with robot_name_. Starts the tf listener, tf_listener_.
 *
 * @return     True if success false otherwise
 */
bool Odometry::setup()
{
    ros::NodeHandle nh;
    // Make publisher
    debug_pub_ = nh.advertise<geometry_msgs::PoseStamped>("odom_debug", 1);
    // Start tf
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    tf_buf_.reset(new tf2_ros::Buffer);
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buf_));
    // Connect to ros topics
    imu_sub_.reset(new ros::Subscriber);
    *imu_sub_ = nh.subscribe("/" + robot_name_ + "/imu", 10, &Odometry::imuCallback, this);
    joint_states_sub_.reset(new ros::Subscriber);
    *joint_states_sub_ = nh.subscribe("/" + robot_name_ + "/joint_states", 10, &Odometry::jointStatesCallback, this);
    laser_sub_.reset(new ros::Subscriber);
    *laser_sub_ = nh.subscribe("/" + robot_name_ + "/laser/scan", 10, &Odometry::laserCallback, this);
}

void Odometry::update()
{
    while (!imu_queue_.empty())
    {
        // Get oldest imu msg
        // TODO(Kevin): Avoid a copy by using a reference
        sensor_msgs::Imu msg;
        {
            std::lock_guard<std::mutex> lock(imu_lock_);
            msg = imu_queue_.front();
        }

        // Do math
        imuUpdate(msg);

        // Update tf
        Eigen::Affine3d tf;
        tf.linear() = head_.cast<double>().toRotationMatrix();  // Maybe change to message heading
        tf.translation() = pos_.cast<double>();
        tf = tf.inverse();
        publishTf(msg.header.frame_id, "/world", msg.header.stamp, tf);

        // Remove last element
        {
            std::lock_guard<std::mutex> lock(imu_lock_);
            imu_queue_.pop();
        }
    }

    while (!laser_queue_.empty())
    {
        // Get next laser scan
        sensor_msgs::LaserScan msg;
        {
            std::lock_guard<std::mutex> lock(laser_lock_);
            msg = laser_queue_.front();
        }

        // Process
        laserUpdate(msg);

        // Remove last laser scan
        {
            std::lock_guard<std::mutex> lock(laser_lock_);
            laser_queue_.pop();
        }
    }
}

void Odometry::publishTf(std::string frame_id, std::string child_frame_id, ros::Time stamp, const Eigen::Affine3d& tf)
{
    geometry_msgs::Pose pose = tf2::toMsg(tf);
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.transform.translation.x = pose.position.x;
    tf_msg.transform.translation.y = pose.position.y;
    tf_msg.transform.translation.z = pose.position.z;
    tf_msg.transform.rotation = pose.orientation;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = frame_id;
    tf_msg.child_frame_id = child_frame_id;
    tf_broadcaster_->sendTransform(tf_msg);
}

/**
 * @brief      Imu callback
 *
 * @param[in]  msg   The message
 */
void Odometry::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(imu_lock_);

    // Temporary fix to the /// instead of _tf/ in imu messages frame_id
    sensor_msgs::Imu fixed_msg = *msg;
    size_t quick_fix = fixed_msg.header.frame_id.find("///");
    std::cout << quick_fix << std::endl;
    if (quick_fix != std::string::npos)
        fixed_msg.header.frame_id.replace(quick_fix, 3, "_tf/");

    imu_queue_.emplace(fixed_msg);
}

void Odometry::imuUpdate(const sensor_msgs::Imu& msg)
{
    // Timing information
    float dt = msg.header.stamp.toSec() - last_time_;
    if (last_time_ == 0)
    {
        last_time_ = msg.header.stamp.toSec();
        return;
    }
    if (dt <= 0)
        return;

    // Convert message to Eigen
    Eigen::Vector3f mes_accel(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    mes_accel -= accel_bias_;
    Eigen::Vector3f mes_vel(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    mes_vel -= ang_vel_bias_;
    Eigen::Quaternionf mes_head(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    // Eigen::Matrix3f mes_rot(mes_head.toRotationMatrix());
    Eigen::Matrix3f rot(mes_head.toRotationMatrix());

    // Filter
    SampleFilter_put(&debug_filt_x_, mes_accel[0]);
    SampleFilter_put(&debug_filt_y_, mes_accel[1]);
    SampleFilter_put(&debug_filt_z_, mes_accel[2]);
    mes_accel = Eigen::Vector3f(SampleFilter_get(&debug_filt_x_), SampleFilter_get(&debug_filt_y_),
                                SampleFilter_get(&debug_filt_z_));

    // std::cout << "----------------------\n"
    //           << "mes_accel:\n"
    //           << mes_accel << std::endl
    //           << "mes_vel:\n"
    //           << mes_vel << std::endl
    //           << "mes_head:\n"
    //           << mes_head.toRotationMatrix() << std::endl
    //           << "mes_head.angularDistance(Eigen::Quaternionf(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()))):\n"
    //           << mes_head.angularDistance(Eigen::Quaternionf(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())))
    //           << std::endl
    //           << "rot:\n"
    //           << rot << std::endl
    //           << "diff:\n"
    //           << mes_head.angularDistance(head_) << std::endl
    //           << "mes_accel.norm():\n"
    //           << mes_accel.norm() << std::endl
    //           << "rot * mes_accel:\n"
    //           << rot * mes_accel << std::endl
    //           << "(rot * mes_accel - grav_):\n"
    //           << (rot * mes_accel - grav_) << std::endl
    //           << "(rot * mes_accel - grav_).norm():\n"
    //           << (rot * mes_accel - grav_).norm() << std::endl
    //           << "SampleFilter_get(&debug_filt_x_):\n"
    //           << SampleFilter_get(&debug_filt_x_) << std::endl
    //           << "SampleFilter_get(&debug_filt_y_):\n"
    //           << SampleFilter_get(&debug_filt_y_) << std::endl
    //           << "SampleFilter_get(&debug_filt_z_):\n"
    //           << SampleFilter_get(&debug_filt_z_) << std::endl;

    geometry_msgs::PoseStamped debug_head;
    debug_head.header = msg.header;
    debug_head.pose.position.x = 0;
    debug_head.pose.position.y = 0;
    debug_head.pose.position.z = 0;
    debug_head.pose.orientation.x = mes_head.x();
    debug_head.pose.orientation.y = mes_head.y();
    debug_head.pose.orientation.z = mes_head.z();
    debug_head.pose.orientation.w = mes_head.w();
    debug_pub_.publish(debug_head);

    // Nominal state update
    last_time_ = msg.header.stamp.toSec();
    // std::cout << "dt: " << dt << std::endl;
    pos_ += vel_ * dt + 0.5 * (rot * mes_accel - grav_) * dt * dt;
    vel_ += (rot * mes_accel - grav_) * dt;
    mes_vel *= dt;
    float phi = mes_vel.norm();
    Eigen::Vector3f u_sin_phi = std::sin(phi / 2) * mes_vel / phi;
    head_ *= Eigen::Quaternionf(std::cos(phi / 2), u_sin_phi.x(), u_sin_phi.y(), u_sin_phi.z());
    // accel_bias_ = accel_bias_;
    // ang_vel_bias_ = ang_vel_bias_;
    // grav_ = grav_;

    // TODO(Kevin): Put this in a utils file
    auto skew_symetric = [](const Eigen::Vector3f& v) {
        Eigen::Matrix3f m = Eigen::Matrix3f::Zero();
        m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
        return m;
    };

    // Calculate Jacobian
    Eigen::Matrix<float, 18, 18> Fx = Eigen::Matrix<float, 18, 18>::Identity();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * dt;
    Fx.block<3, 3>(3, 6) = -rot * skew_symetric(mes_accel) * dt;
    Fx.block<3, 3>(3, 9) = -rot * dt;
    Fx.block<3, 3>(3, 15) = Fx.block<3, 3>(0, 3);
    // Fx.block<3, 3>(6, 6) = rot.transpose() * mes_vel * dt;  // TODO(Kevin): Figure out what the notation R{x}
    // means
    Fx.block<3, 3>(6, 12) = -Fx.block<3, 3>(0, 3);

    Eigen::Matrix<float, 18, 12> Fi = Eigen::Matrix<float, 18, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<float, 12, 12>::Identity();

    // TODO(Kevin): Define this elsewhere and find real parameters
    Eigen::Matrix<float, 12, 12> Q = Eigen::Matrix<float, 12, 12>::Identity() * 0.01;

    // Error mean always 0
    err_cov_ = Fx * err_cov_ * Fx.transpose() + Fi * Q * Fi.transpose();

    // std::cout << "----------------------\n"
    //           << "pos_:\n"
    //           << pos_ << std::endl
    //           << "vel_:\n"
    //           << vel_ << std::endl
    //           << "head_:\n"
    //           << head_.matrix() << std::endl
    //           << "accel_bias_:\n"
    //           << accel_bias_ << std::endl
    //           << "ang_vel_bias_:\n"
    //           << ang_vel_bias_ << std::endl
    //           << "grav_:\n"
    //           << grav_ << std::endl;
    // << "err_cov_:\n" << err_cov_ << std::endl;
}

/**
 * @brief      Joint state callback
 *
 * @param[in]  msg   The message
 */
void Odometry::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
}

void Odometry::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(laser_lock_);
    laser_queue_.emplace(*msg);
}

void Odometry::laserUpdate(const sensor_msgs::LaserScan& msg)
{
    // Convert to pcl point cloud xyz
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 ros_pc;
    projector.projectLaser(msg, ros_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(ros_pc, *pcl_pc);

    // If this is the first time just save the pc
    if (last_pc_ == NULL)
    {
        last_pc_ = pcl_pc;
        return;
    }

    // Do ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pcl_pc);
    icp.setInputTarget(last_pc_);
    pcl::PointCloud<pcl::PointXYZ> tfed_pc;
    icp.align(tfed_pc);
    // std::cout << icp.getFinalTransformation() << std::endl;
    last_laser_tf_ = Eigen::Affine3d(icp.getFinalTransformation().cast<double>()) * last_laser_tf_;
    publishTf(msg.header.frame_id, "/world_laser", msg.header.stamp, last_laser_tf_);

    // Update
    last_pc_ = pcl_pc;
}
