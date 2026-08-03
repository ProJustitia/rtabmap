import re

with open("corelib/include/rtabmap/core/odometry/OdometryCuVSLAM.h", "r") as f:
    content = f.read()

old_block = """	// IMU buffer for early measurements
	std::map<double, CUVSLAM_ImuMeasurement> imu_buffer_;

	// State tracking"""

new_block = """	// IMU buffer for early measurements
	std::map<double, CUVSLAM_ImuMeasurement> imu_buffer_;
	Transform rig_from_imu_;

	// State tracking"""

content = content.replace(old_block, new_block)

with open("corelib/include/rtabmap/core/odometry/OdometryCuVSLAM.h", "w") as f:
    f.write(content)

print("Patched header.")
