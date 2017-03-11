#pragma once

static u64 gRayCount;
static u64 gSpheresChecked;
static u64 gMeshesChecked;

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
} gParams;

