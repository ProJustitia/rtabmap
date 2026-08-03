import re

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "r") as f:
    content = f.read()

# 1. Update CreateConfiguration signature
content = content.replace(
    "CUVSLAM_Configuration CreateConfiguration(const SensorData & data, int multicam_mode);",
    "CUVSLAM_Configuration CreateConfiguration(const SensorData & data, int multicam_mode, const Transform & rig_from_imu);"
)

content = content.replace(
    "CUVSLAM_Configuration CreateConfiguration(const SensorData & data, int multicam_mode)",
    "CUVSLAM_Configuration CreateConfiguration(const SensorData & data, int multicam_mode, const Transform & rig_from_imu)"
)

content = content.replace(
    "    const CUVSLAM_Configuration configuration = CreateConfiguration(data, multicam_mode);",
    "    const CUVSLAM_Configuration configuration = CreateConfiguration(data, multicam_mode, rig_from_imu_);"
)

# 2. Revert the manual IMU rotation
old_block = """    // Process IMU data if available
    if(!data.imu().empty())
    {
        Transform rig_from_imu = cuvslam_pose_canonical * data.imu().localTransform();
        cv::Point3f acc(data.imu().linearAcceleration().val[0], data.imu().linearAcceleration().val[1], data.imu().linearAcceleration().val[2]);
        cv::Point3f gyr(data.imu().angularVelocity().val[0], data.imu().angularVelocity().val[1], data.imu().angularVelocity().val[2]);
        
        // ONLY apply rotation, do not add translation!
        acc = cv::Point3f(
            rig_from_imu.r11()*acc.x + rig_from_imu.r12()*acc.y + rig_from_imu.r13()*acc.z,
            rig_from_imu.r21()*acc.x + rig_from_imu.r22()*acc.y + rig_from_imu.r23()*acc.z,
            rig_from_imu.r31()*acc.x + rig_from_imu.r32()*acc.y + rig_from_imu.r33()*acc.z
        );
        gyr = cv::Point3f(
            rig_from_imu.r11()*gyr.x + rig_from_imu.r12()*gyr.y + rig_from_imu.r13()*gyr.z,
            rig_from_imu.r21()*gyr.x + rig_from_imu.r22()*gyr.y + rig_from_imu.r23()*gyr.z,
            rig_from_imu.r31()*gyr.x + rig_from_imu.r32()*gyr.y + rig_from_imu.r33()*gyr.z
        );
        
        CUVSLAM_ImuMeasurement imu;
        imu.angular_velocities[0] = gyr.x;
        imu.angular_velocities[1] = gyr.y;
        imu.angular_velocities[2] = gyr.z;
        imu.linear_accelerations[0] = acc.x;
        imu.linear_accelerations[1] = acc.y;
        imu.linear_accelerations[2] = acc.z;"""

new_block = """    // Process IMU data if available
    if(!data.imu().empty())
    {
        if(rig_from_imu_.isNull()) {
            rig_from_imu_ = cuvslam_pose_canonical * data.imu().localTransform();
        }
        
        CUVSLAM_ImuMeasurement imu;
        imu.angular_velocities[0] = data.imu().angularVelocity().val[0];
        imu.angular_velocities[1] = data.imu().angularVelocity().val[1];
        imu.angular_velocities[2] = data.imu().angularVelocity().val[2];
        imu.linear_accelerations[0] = data.imu().linearAcceleration().val[0];
        imu.linear_accelerations[1] = data.imu().linearAcceleration().val[1];
        imu.linear_accelerations[2] = data.imu().linearAcceleration().val[2];"""

content = content.replace(old_block, new_block)

# 3. Update CreateConfiguration body
old_config = """    // Odometry configuration
    configuration.odometry_mode = CUVSLAM_OdometryMode::Multicamera;
    configuration.multicam_mode = multicam_mode;    
    configuration.debug_imu_mode = 0;"""

new_config = """    // Odometry configuration
    if (!rig_from_imu.isNull()) {
        configuration.odometry_mode = CUVSLAM_OdometryMode::Inertial;
        configuration.imu_calibration.rig_from_imu = TocuVSLAMPose(rig_from_imu);
        configuration.imu_calibration.gyroscope_noise_density = 0.00024f;
        configuration.imu_calibration.gyroscope_random_walk = 0.00001f;
        configuration.imu_calibration.accelerometer_noise_density = 0.004f;
        configuration.imu_calibration.accelerometer_random_walk = 0.0001f;
        configuration.imu_calibration.frequency = 200.0f;
    } else {
        configuration.odometry_mode = CUVSLAM_OdometryMode::Multicamera;
    }
    configuration.multicam_mode = multicam_mode;    
    configuration.debug_imu_mode = 0;"""

content = content.replace(old_config, new_config)

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "w") as f:
    f.write(content)

print("Patched cpp.")
