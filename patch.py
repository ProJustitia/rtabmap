import re

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "r") as f:
    content = f.read()

# 1. Clear imu_buffer_
content = content.replace("""    gpu_right_image_sizes_.clear();
    cuvslam_cameras_.clear();
    intrinsics_.clear();
    initialized_ = false;""", """    gpu_right_image_sizes_.clear();
    cuvslam_cameras_.clear();
    intrinsics_.clear();
    imu_buffer_.clear();
    initialized_ = false;""")

# 2. Buffer IMU instead of pushing
old_imu = """        imu.linear_accelerations[2] = acc.z;
        imu.angular_velocities[0] = gyr.x;
        imu.angular_velocities[1] = gyr.y;
        imu.angular_velocities[2] = gyr.z;
        
        if(cuvslam_handle_) {
            int64_t timestamp_ns = static_cast<int64_t>(data.stamp() * 1000000000.0);
            CUVSLAM_RegisterImuMeasurement(cuvslam_handle_, timestamp_ns, &imu);
        } else {
            imu_buffer_.insert(std::make_pair(data.stamp(), imu));
        }
        
        if(data.imageRaw().empty()) {"""

new_imu = """        imu.linear_accelerations[2] = acc.z;
        imu.angular_velocities[0] = gyr.x;
        imu.angular_velocities[1] = gyr.y;
        imu.angular_velocities[2] = gyr.z;
        
        // Always buffer IMU measurements to ensure strict order when images arrive
        imu_buffer_.insert(std::make_pair(data.stamp(), imu));
        
        if(data.imageRaw().empty()) {"""

content = content.replace(old_imu, new_imu)

# 3. Flush IMU up to current image
old_init = """    if(!cuvslam_handle_) {
        // Initialize cuVSLAM on first frame
        if(!data.imageRaw().empty()) {
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
    }
    
    // Synchronize CUDA stream before calling cuVSLAM APIs
    if(cuvslam_handle_) {
        if(!synchronizeGpuOperations(
            cuda_stream_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return Transform();
        }
        
        // Flush IMU buffer
        for(auto & pair : imu_buffer_) {
            int64_t timestamp_ns = static_cast<int64_t>(pair.first * 1000000000.0);
            CUVSLAM_RegisterImuMeasurement(cuvslam_handle_, timestamp_ns, &pair.second);
        }
        imu_buffer_.clear();
    }
        
    // Prepare images for cuVSLAM"""

new_init = """    if(!cuvslam_handle_) {
        // Initialize cuVSLAM on first frame
        if(!data.imageRaw().empty()) {
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
    }
    
    // Synchronize CUDA stream before calling cuVSLAM APIs
    if(cuvslam_handle_) {
        if(!synchronizeGpuOperations(
            cuda_stream_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return Transform();
        }
    }
        
    if(!cuvslam_handle_) {
        UERROR("cuVSLAM tracker is null! initialized_: %s", initialized_ ? "true" : "false");
        return Transform();
    }

    // Flush IMU buffer for timestamps <= current image timestamp
    // This strictly ensures that CUVSLAM_TrackGpuMem is called with a monotonically increasing timestamp
    auto it = imu_buffer_.begin();
    while(it != imu_buffer_.end() && it->first <= data.stamp()) {
        int64_t timestamp_ns = static_cast<int64_t>(it->first * 1000000000.0);
        CUVSLAM_RegisterImuMeasurement(cuvslam_handle_, timestamp_ns, &it->second);
        it = imu_buffer_.erase(it);
    }
    
    // Prepare images for cuVSLAM"""

content = content.replace(old_init, new_init)

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "w") as f:
    f.write(content)

print("Patched.")
