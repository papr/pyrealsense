#include <stdint.h>

#include "rs.h"
#include "rsutil.h"
#include <cmath>
#include <stdio.h>

void _apply_depth_control_preset(rs_device * device, int preset)
{
    rs_apply_depth_control_preset(device, preset);
}


void _apply_ivcam_preset(rs_device * device, rs_ivcam_preset preset)
{
    rs_apply_ivcam_preset(device, preset);
}


void _project_point_to_pixel(float pixel[],
                             const struct rs_intrinsics * intrin,
                             const float point[])
{
    rs_project_point_to_pixel(pixel, intrin, point);
}


void _deproject_pixel_to_point(float point[],
                               const struct rs_intrinsics * intrin,
                               const float pixel[],
                               float depth)
{
    rs_deproject_pixel_to_point(point, intrin, pixel, depth);
}



void _project_pointcloud_to_pixel(uint8_t *image,
								  const struct rs_intrinsics *depth_intrin,
								  const struct rs_intrinsics *color_intrin,
								  const struct rs_extrinsics *depth_to_color_extrin,
	                              float *pointcloud,
	                              uint8_t *color_image)
{
    //printf("image = %p, di = %p, ci = %p, dtce = %p, pc = %p, coli = %p\n", image, depth_intrin, color_intrin, depth_to_color_extrin, pointcloud, color_image);
    //printf("depth_intrin->width = %d, depth_intrin->height = %d\n", depth_intrin->width, depth_intrin->height);
    //printf("color_intrin->width = %d, color_intrin->height = %d\n", color_intrin->width, color_intrin->height);
    
    //for (int i = 0; i < 9; i++) {
    //    printf("rot[%d] = %f\n", i, depth_to_color_extrin->rotation[i]);
    //}
    //for (int i = 0; i < 3; i++) {
    //    printf("trans[%d] = %f\n", i, depth_to_color_extrin->translation[i]);
    //}
    for (int dy = 0; dy < depth_intrin->height; ++dy)
	{
		for (int dx = 0; dx < depth_intrin->width; ++dx)
		{
			float color_point[3];
			float pixel[2];

			rs_transform_point_to_point(color_point, depth_to_color_extrin, &pointcloud[(depth_intrin->width * dy + dx) * 3]);
			rs_project_point_to_pixel(pixel, color_intrin, color_point);

			// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
			const int cx = (int)std::round(pixel[0]), cy = (int)std::round(pixel[1]);
            if (cx < 0 || cy < 0 || cx >= color_intrin->width || cy >= color_intrin->height)
			{
				*image++ = 255;
				*image++ = 255;
				*image++ = 255;
			}
			else
			{
				uint8_t *src = color_image + (cy * color_intrin->width + cx) * 3;
				*image++ = *(src+2);
				*image++ = *(src+1);
				*image++ = *src;
			}

		}
	}
}

void _deproject_depth(float pointcloud[],
                      const struct rs_intrinsics * intrin,
                      const uint16_t depth_image[],
                      const float depth_scale)
{
    int dx, dy;
    for(dy=0; dy<intrin->height; ++dy)
    {
        for(dx=0; dx<intrin->width; ++dx)
        {
            /* Retrieve the 16-bit depth value and map it into a depth in meters */
            uint16_t depth_value = ((uint16_t*)depth_image)[dy * intrin->width + dx];
            float depth_in_meters = depth_value * depth_scale;
            /* Skip over pixels with a depth value of zero, which is used to indicate no data */
            if(depth_value == 0) continue;
            /* Map from pixel coordinates in the depth image to pixel coordinates in the color image */
            float depth_pixel[2] = {(float)dx, (float)dy};
            float depth_point[3];
            rs_deproject_pixel_to_point(depth_point, intrin, depth_pixel, depth_in_meters);
            /* store a vertex at the 3D location of this depth pixel */
            pointcloud[dy*intrin->width*3 + dx*3 + 0] = depth_point[0];
            pointcloud[dy*intrin->width*3 + dx*3 + 1] = depth_point[1];
            pointcloud[dy*intrin->width*3 + dx*3 + 2] = depth_point[2];
        }
    }
}

void _transform_point_to_point(float* to_point,
    const rs_extrinsics * extrin,
    const float* from_point)
{
    rs_transform_point_to_point(to_point, extrin, from_point);
}