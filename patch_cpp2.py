import re

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "r") as f:
    content = f.read()

content = content.replace(
"""                       std::vector<CUVSLAM_Camera> & cuvslam_cameras,
	                   std::vector<std::array<float, 12>> & intrinsics,
                       cudaStream_t & cuda_stream);""",
"""                       std::vector<CUVSLAM_Camera> & cuvslam_cameras,
	                   std::vector<std::array<float, 12>> & intrinsics,
                       cudaStream_t & cuda_stream,
                       const Transform & rig_from_imu);"""
)

content = content.replace(
"""                       std::vector<CUVSLAM_Camera> & cuvslam_cameras,
	                   std::vector<std::array<float, 12>> & intrinsics,
                       cudaStream_t & cuda_stream)""",
"""                       std::vector<CUVSLAM_Camera> & cuvslam_cameras,
	                   std::vector<std::array<float, 12>> & intrinsics,
                       cudaStream_t & cuda_stream,
                       const Transform & rig_from_imu)"""
)

content = content.replace(
"""            if(!initializeCuVSLAM(
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
                cuda_stream_))""",
"""            if(!initializeCuVSLAM(
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
                cuda_stream_,
                rig_from_imu_))"""
)

content = content.replace(
"""    const CUVSLAM_Configuration configuration = CreateConfiguration(data, multicam_mode, rig_from_imu_);""",
"""    const CUVSLAM_Configuration configuration = CreateConfiguration(data, multicam_mode, rig_from_imu);"""
)

with open("corelib/src/odometry/OdometryCuVSLAM.cpp", "w") as f:
    f.write(content)

print("Patched.")
