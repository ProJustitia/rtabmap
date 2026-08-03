import re

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "r") as f:
    content = f.read()

# 1. Intercept IMU packets
old_check = """    // Check if we have valid image data
    if(data.imageRaw().empty() || data.rightRaw().empty())
    {
        UERROR("cuVSLAM odometry only works with stereo cameras! It requires both left and right images! Left: %s, Right: %s", 
               data.imageRaw().empty() ? "empty" : "ok", 
               data.rightRaw().empty() ? "empty" : "ok");       
        return Transform();
    }"""

new_check = """    // Process IMU data if available
    if(!data.imu().empty())
    {
        CUVSLAM_ImuMeasurement imu;
        imu.angular_velocities[0] = data.imu().angularVelocity().val[0];
        imu.angular_velocities[1] = data.imu().angularVelocity().val[1];
        imu.angular_velocities[2] = data.imu().angularVelocity().val[2];
        imu.linear_accelerations[0] = data.imu().linearAcceleration().val[0];
        imu.linear_accelerations[1] = data.imu().linearAcceleration().val[1];
        imu.linear_accelerations[2] = data.imu().linearAcceleration().val[2];
        
        // Always buffer IMU measurements to ensure strict order when images arrive
        imu_buffer_.insert(std::make_pair(data.stamp(), imu));
        
        // If it's an IMU-only packet, return success early so it doesn't fail on missing images
        if(data.imageRaw().empty()) {
            if(cuvslam_handle_ && initialized_) {
                // If requested, we could try to get pose here, but returning previous pose is enough
                // as RTAB-Map's OdometryROS handles tf publishing.
                if(info) {
                    info->lost = false;
                    info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.001; // small covariance for IMU propagation
                }
            }
            return previous_pose_;
        }
    }

    // Check if we have valid image data
    if(data.imageRaw().empty() || data.rightRaw().empty())
    {
        UERROR("cuVSLAM odometry only works with stereo cameras! It requires both left and right images! Left: %s, Right: %s", 
               data.imageRaw().empty() ? "empty" : "ok", 
               data.rightRaw().empty() ? "empty" : "ok");       
        return Transform();
    }"""

content = content.replace(old_check, new_check)

# 2. Flush IMU buffer before prepareImages
old_init = """    // Initialize cuVSLAM tracker on first frame
    if(!initialized_)
    {   
        if(!initializeCuVSLAM(
            data, 
            cuvslam_handle_,
            ground_constraint_handle_,
            planar_constraints_,
            multicam_mode_,
            gpu_left_image_data_,
            gpu_right_image_data_,
            gpu_left_image_sizes_,
            gpu_right_image_sizes_,
            cuvslam_cameras_,
	        intrinsics_,
            cuda_stream_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return Transform();
        }
    }
        
    // Prepare images for cuVSLAM"""

new_init = """    // Initialize cuVSLAM tracker on first frame
    if(!initialized_)
    {   
        if(!initializeCuVSLAM(
            data, 
            cuvslam_handle_,
            ground_constraint_handle_,
            planar_constraints_,
            multicam_mode_,
            gpu_left_image_data_,
            gpu_right_image_data_,
            gpu_left_image_sizes_,
            gpu_right_image_sizes_,
            cuvslam_cameras_,
	        intrinsics_,
            cuda_stream_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return Transform();
        }
    }
    
    // Flush IMU buffer for timestamps <= current image timestamp
    // This strictly ensures that CUVSLAM_TrackGpuMem is called with a monotonically increasing timestamp
    if(cuvslam_handle_) {
        auto it = imu_buffer_.begin();
        while(it != imu_buffer_.end() && it->first <= data.stamp()) {
            int64_t timestamp_ns = static_cast<int64_t>(it->first * 1000000000.0);
            CUVSLAM_RegisterImuMeasurement(cuvslam_handle_, timestamp_ns, &it->second);
            it = imu_buffer_.erase(it);
        }
    }
        
    // Prepare images for cuVSLAM"""

content = content.replace(old_init, new_init)

# 3. Add imu_buffer_.clear() to cleanup
old_cleanup = """    gpu_right_image_sizes_.clear();
    cuvslam_cameras_.clear();
    intrinsics_.clear();
    initialized_ = false;"""

new_cleanup = """    gpu_right_image_sizes_.clear();
    cuvslam_cameras_.clear();
    intrinsics_.clear();
    imu_buffer_.clear();
    initialized_ = false;"""

content = content.replace(old_cleanup, new_cleanup)

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "w") as f:
    f.write(content)

print("Patched OdometryCuVSLAM.cpp")
