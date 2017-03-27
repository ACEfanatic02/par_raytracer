#pragma once

struct DebugCounters {
    u64 ray_count;
    u64 sphere_check_count;
    u64 mesh_check_count;
};

struct {
    float ray_bias;
    u32 reflection_samples;
    u32 spec_samples;
    u32 bounce_depth;
    Vector4 background_color;
    float camera_fov;
    Vector3 camera_position;
    Vector3 camera_facing;
    char * image_output_filename;
    char * data_dirname;
    u32 image_width;
    u32 image_height;
} gParams;

