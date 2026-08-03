import re

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "r") as f:
    content = f.read()

old_block = """    // Process IMU data if available
    if(!data.imu().empty())
    {
        CUVSLAM_ImuMeasurement imu;
        imu.angular_velocities[0] = data.imu().angularVelocity().val[0];
        imu.angular_velocities[1] = data.imu().angularVelocity().val[1];
        imu.angular_velocities[2] = data.imu().angularVelocity().val[2];
        imu.linear_accelerations[0] = data.imu().linearAcceleration().val[0];
        imu.linear_accelerations[1] = data.imu().linearAcceleration().val[1];
        imu.linear_accelerations[2] = data.imu().linearAcceleration().val[2];"""

new_block = """    // Process IMU data if available
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

content = content.replace(old_block, new_block)

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "w") as f:
    f.write(content)

print("Patched.")
