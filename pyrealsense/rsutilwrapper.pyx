# -*- coding: utf-8 -*-
# Licensed under the Apache-2.0 License, see LICENSE for details.

import numpy as np
cimport numpy as np

import ctypes
from libc.stdint cimport uintptr_t, uint16_t, uint8_t


cdef extern from "rs.h":
    cdef struct rs_device:
        pass
    cdef struct rs_error:
        pass
    cdef enum rs_ivcam_preset:
        pass
    cdef struct rs_intrinsics:
        pass
    cdef struct rs_extrinsics:
        pass


cdef extern from "rsutilwrapper.h":
    void _apply_depth_control_preset(rs_device* device, int preset)
    void _apply_ivcam_preset(rs_device* device, rs_ivcam_preset preset)
    void _project_point_to_pixel(float* pixel,
                                 const rs_intrinsics* intrin,
                                 const float* point)
    void _deproject_pixel_to_point(float* point,
                                   const rs_intrinsics* intrin,
                                   const float* pixel,
                                   const float depth)
    void _deproject_depth(float* poincloud,
                          const rs_intrinsics* intrin,
                          const uint16_t* depth_image,
                          const float depth_scale)
    void _transform_point_to_point(float* to_point, 
                                   const rs_extrinsics * extrin, 
                                   const float* from_point)

    void _project_pointcloud_to_pixel(uint8_t *image,
	                                  const rs_intrinsics *depth_intrin,
	                                  const rs_intrinsics *color_intrin,
	                                  const rs_extrinsics *depth_to_color_extrin,
	                                  float *pointcloud,
	                                  uint8_t *color_image)

def apply_depth_control_preset(device, preset):
    cdef uintptr_t adr = <uintptr_t>ctypes.addressof(device.contents)
    _apply_ivcam_preset(<rs_device*>adr, preset)

def apply_ivcam_preset(device, preset):
    cdef uintptr_t adr = <uintptr_t>ctypes.addressof(device.contents)
    _apply_ivcam_preset(<rs_device*>adr, <rs_ivcam_preset>preset)

def project_point_to_pixel(np.ndarray pixel, intrin, np.ndarray point):
    cdef uintptr_t adr = <uintptr_t>ctypes.addressof(intrin)
    _project_point_to_pixel(<float*> pixel.data, <rs_intrinsics*>adr, <float*> point.data)

def deproject_pixel_to_point(np.ndarray point, intrin, np.ndarray pixel, depth):
    cdef uintptr_t adr = <uintptr_t>ctypes.addressof(intrin)
    _deproject_pixel_to_point(<float*> point.data, <rs_intrinsics*>adr, <float*> pixel.data, depth)

def deproject_depth(np.ndarray pointcloud, intrin, np.ndarray depth_image, depth_scale):
    cdef uintptr_t adr = <uintptr_t>ctypes.addressof(intrin)
    _deproject_depth(<float*> pointcloud.data, <rs_intrinsics*>adr, <uint16_t*> depth_image.data, depth_scale)

def transform_point_to_point(np.ndarray to_point, extrin, np.ndarray from_point):
    cdef uintptr_t adr = <uintptr_t>ctypes.addressof(extrin)
    _transform_point_to_point(<float*> to_point.data,<rs_extrinsics*> adr,<float*> from_point.data)

def project_pointcloud_to_pixel(np.ndarray image, depth_intrin, col_intrin, extrin, np.ndarray pointcloud, np.ndarray col_image):
    cdef uintptr_t cint_adr = <uintptr_t>ctypes.addressof(col_intrin)
    cdef uintptr_t ext_adr = <uintptr_t>ctypes.addressof(extrin)
    cdef uintptr_t dint_adr = <uintptr_t>ctypes.addressof(depth_intrin)
    _project_pointcloud_to_pixel(<uint8_t*> image.data, <rs_intrinsics*> dint_adr, <rs_intrinsics*> cint_adr, <rs_extrinsics*> ext_adr, <float*> pointcloud.data, <uint8_t*> col_image.data)

