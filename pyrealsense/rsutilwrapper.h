#include "rs.h"

void _apply_depth_control_preset(rs_device * device, int preset);
void _apply_ivcam_preset(rs_device * device, rs_ivcam_preset preset);
void _project_point_to_pixel(float pixel[],
                             const rs_intrinsics * intrin,
                             const float point[]);
void _deproject_pixel_to_point(float point[],
                               const struct rs_intrinsics * intrin,
                               const float pixel[],
                               float depth);
void _deproject_depth(float poincloud[],
                      const struct rs_intrinsics * intrin,
                      const uint16_t depth_image[],
                      const float depth_scale);

void _transform_point_to_point(float* to_point, 
							   const struct rs_extrinsics * extrin, 
	                           const float* from_point);

void _project_pointcloud_to_pixel(uint8_t *image,
	const struct rs_intrinsics *depth_intrin,
	const struct rs_intrinsics *color_intrin,
	const struct rs_extrinsics *depth_to_color_extrin,
	float *pointcloud,
	uint8_t *color_image);
