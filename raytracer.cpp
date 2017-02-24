// raytracer.cpp - Simple Raytracer
// Bryan Taylor 2/18/2017
#include "brt.h"
#include <stdlib.h>
#include <float.h>
#include "mathlib.h"
#include "color.h"
#include "random.h"
#include "mesh.h"
#include "geometry.h"
#include "timing.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "lib/stb_image_write.h"

#include "obj_parser.cpp"
#include "bsphere.cpp"

static u32 gRayCount;

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
    u32 use_system_rand;
} gParams;

// newmtl leaf
//     Ns 10.0000
//     Ni 1.5000
//     d 1.0000
//     Tr 0.0000
//     Tf 1.0000 1.0000 1.0000 
//     illum 2
//     Ka 0.5880 0.5880 0.5880
//     Kd 0.5880 0.5880 0.5880
//     Ks 0.0000 0.0000 0.0000
//     Ke 0.0000 0.0000 0.0000
//     map_Ka textures\sponza_thorn_diff.tga
//     map_Kd textures\sponza_thorn_diff.tga
//     map_d textures\sponza_thorn_mask.tga
//     map_bump textures\sponza_thorn_bump.png
//     bump textures\sponza_thorn_bump.png
struct Material {
    float specular_intensity;
    float index_of_refraction;
    float alpha;

    Vector4 ambient_color;
    Vector4 diffuse_color;
    Vector4 specular_color;
    Vector4 emissive_color;

    // TODO(bryan): Texture maps
};

enum LightSourceType {
    Light_Directional,
    // Light_Spot,
    Light_Point,
};

struct LightSource {
    LightSourceType type;
    Vector4 color;
    Vector3 position;
    Vector3 facing;
    float falloff;
};

enum ObjectType {
    ObjectType_Sphere,
    ObjectType_MeshGroup,
};

struct SceneObject {
    struct {
        Sphere sphere;
        MeshGroup * mesh_group;
        Mesh * mesh;
    };
    ObjectType type;
    Material * material;
};

struct Scene {
    std::vector<SceneObject *> objects;
    BoundingHierarchy * hierarchy;
    LightSource * lights;
    u32 light_count;
    Material * default_mat;
};

// struct TriangleHitParams {
//     float t;
//     // Barycentric coordinates (used for vertex attribute interpolation):
//     float u;
//     float v;
//     float w;

//     Vector3 normal;
// };

struct RaycastHit {
    float t;
    // Barycenteric coordinates (for triangles)
    Vector3 bw;
    // Index into index buffer for triangle's first vertex.
    u32 vertex0;

    Vector3 position;
    Vector3 normal;
    SceneObject * object;
};

static bool 
IntersectRaySphere(Ray ray, Sphere sphere, RaycastHit * out_hit) {
    // Adapted from Ericson, Real-Time Collision Detection, pg 178
    Vector3 m = ray.origin - sphere.center;
    // Are we pointing at the sphere?
    float b = Dot(m, ray.direction);
    // Is the origin of the ray inside the sphere?
    float c = Dot(m, m) - sphere.radius * sphere.radius;

    // Ray starts outside sphere and points away from it.
    if (c > 0.0f && b > 0.0f) {
        return false;
    }

    float discriminant = b*b - c;
    // Ray misses the sphere.
    if (discriminant < 0.0f) {
        return false;
    }

    float t = -b - sqrtf(discriminant);
    if (t < 0.0f) t = 0.0f;
    Vector3 p = ray.direction * t;
    out_hit->t = t;
    out_hit->position = p;
    out_hit->normal = Normalize(p - sphere.center);
    return true;
}

static bool
TestRaySphere(Ray ray, Sphere sphere) {
    static const float epsilon = 1e-4;
    Vector3 m = ray.origin - sphere.center;
    // Is the origin of the ray inside the sphere?
    float c = Dot(m, m) - sphere.radius * sphere.radius;

    if (c <= 0.0f) return true;
    // Are we pointing at the sphere?
    float b = Dot(m, ray.direction);

    // This is COMPLETELY WRONG, yet it fixes the raycast used
    // for object pruning.  What.
    if (b < 0.0f) return false;

    float discriminant = b*b - c;
    // Ray misses the sphere.
    if (discriminant < 0.0f) return false;

    return true;
}

static bool
IntersectRayTriangle(Ray ray, Vector3 a, Vector3 b, Vector3 c, RaycastHit * out_hit) {
    // Adapted from Ericson, Real-Time Collision Detection, pg 191
    Vector3 ab = b - a;
    Vector3 ac = c - a;

    // TODO(bryan):  We don't need to recalculate this every time.
    Vector3 normal = Cross(ab, ac);

    float d = Dot(ray.direction, normal);
    if (d <= 0.0f) {
        return false;
    }

    Vector3 ap = ray.origin - a;
    float t = Dot(ap, normal);

    if (t < 0.0f) {
        return false;
    }

    // At this point, we know we hit the plane of the triangle, now check if
    // we're actually inside it.
    Vector3 e = Cross(ray.direction, ap);
    float v = Dot(ac, e);
    if (v < 0.0f || v > d) return false;
    float w = -Dot(ab, e);
    if (w < 0.0f || (v + w) > d) return false;

    float ood = 1.0f / d;
    out_hit->t = t * ood;
    out_hit->bw.y = v * ood;
    out_hit->bw.z = w * ood;
    out_hit->bw.x = 1.0f - out_hit->bw.y - out_hit->bw.z;
    out_hit->position = ray.direction * out_hit->t;
    out_hit->normal = Normalize(normal);

    return true;
}

static bool
IntersectRayMesh(Ray ray, SceneObject * obj, RaycastHit * out_hit) {
    assert(obj->type == ObjectType_MeshGroup);
    MeshGroup * mesh_group = obj->mesh_group;
    Mesh * mesh = obj->mesh;

    bool hit = false;
    RaycastHit best_hit = { FLT_MAX };
    for (u32 i = 0; i < mesh_group->idx_positions.size(); i += 3) {
        u32 idx_a = mesh_group->idx_positions[i + 0];
        u32 idx_b = mesh_group->idx_positions[i + 1];
        u32 idx_c = mesh_group->idx_positions[i + 2];

        Vector3 pa = mesh->positions[idx_a];
        Vector3 pb = mesh->positions[idx_b];
        Vector3 pc = mesh->positions[idx_c];

        RaycastHit current_hit;
        if (IntersectRayTriangle(ray, pa, pb, pc, &current_hit)) {
            current_hit.vertex0 = i;
            current_hit.object = obj;
            if (current_hit.t < best_hit.t) {
                best_hit = current_hit;
                hit = true;
            }
        }
    }
    if (out_hit) *out_hit = best_hit;
    return hit;
}

static bool
TraceRay(Ray ray, Scene * scene, RaycastHit * out_hit) {
    gRayCount++;
    // Bias ray
    ray.origin += ray.direction * gParams.ray_bias;

    bool hit = false;
    RaycastHit best_hit = { FLT_MAX };
#if 0
    for (u32 i = 0; i < scene->object_count; ++i) {
        SceneObject * o = scene->objects + i;
        float t;
        if (o->type == ObjectType_Sphere) {
            if (IntersectRaySphere(ray, o->sphere, &t)) {
                if (t < best_t) {
                    best_object = o;
                    best_t = t;
                    hit = true;
                }
            }
        }
    }
#endif

    std::vector<u32> check_spheres;
    check_spheres.push_back(0);
    std::vector<u32> check_meshes;
    while (check_spheres.size() > 0) {
        u32 i = check_spheres.back();
        check_spheres.pop_back();
        BoundingSphere s = scene->hierarchy->spheres[i];
        RaycastHit sphere_test_hit;
        // if (IntersectRaySphere(ray, s.s, &sphere_test_hit)) {
        if (TestRaySphere(ray, s.s)) 
        {
            if (s.c0 && s.c1) {
                // TODO(bryan):  Insert closest sphere to ray origin first.
                check_spheres.push_back(s.c0);
                check_spheres.push_back(s.c1);
            }
            else {
                assert(s.c0 == 0 && s.c1 == 0);
                check_meshes.push_back(i);
            }
        }
    }

    for (u32 i = 0; i < check_meshes.size(); ++i) {
        RaycastHit current_hit;
        SceneObject * obj = scene->objects[check_meshes[i]];
        if (IntersectRayMesh(ray, obj, &current_hit)) {
            if (current_hit.t < best_hit.t) {
                best_hit = current_hit;
                hit = true;
            }
        }
    }

    if (out_hit) *out_hit = best_hit;
    return hit;
}

static Ray
GetShadowRayForLight(Vector3 origin, LightSource * light) {
    Ray ray;
    ray.origin = origin;
    switch (light->type) {
        case Light_Directional: {
            ray.direction = light->facing * -1.0f;
        } break;
        case Light_Point: {
            ray.direction = Normalize(light->position - ray.origin);
        } break;
        default: {
            Assert(false, "Unrecognized light type!");
        } break;
    }
    return ray;
}

static RandomState rng;
static bool rng_init = false;

static float
GetRandFloat01() {
    if (!gParams.use_system_rand) {
        if (!rng_init) {
            Random_Seed(&rng, 0xdeadbeef01234567ull);
            rng_init = true;
        }
        return Random_NextFloat01(&rng);      
    }
    return (float)rand() / (float)RAND_MAX;
}

static float
GetRandFloat11() {
    if (!gParams.use_system_rand) {
        if (!rng_init) {
            Random_Seed(&rng, 0xdeadbeef01234567ull);
            rng_init = true;
        }
        return Random_NextFloat11(&rng);
    }
    return GetRandFloat01() * 2.0f - 1.0f;
}

static Ray
GetRayInCone(Vector3 origin, Vector3 normal, float * cos_theta, float min_cos) {
    Matrix33 normal_to_world = Matrix33_FromToRotation(normal, Vector3(0.0f, 1.0f, 0.0f));
    Ray ray;
    ray.origin = origin;

    // Pick a random angle (this will be rotation around normal)
    float angle = GetRandFloat01() * PI32 * 2.0f;
    // Pick random height along normal
    float z = Lerp(min_cos, 1.0f, GetRandFloat01());
    // Project that height onto the tangent plane
    float xy = (1.0f - z*z);
    // Calculate the tangent components
    float ct = cosf(angle);
    float st = sinf(angle);
    float x = xy * ct;
    float y = xy * st;
    // Renormalize just to be sure.
    ray.direction = normal_to_world * Normalize(Vector3(x, y, z));

    *cos_theta = ct;
    return ray;
}

static Ray
GetDiffuseReflectionRay(Vector3 origin, Vector3 normal, float * cos_theta) {
    return GetRayInCone(origin, normal, cos_theta, 0.0f);
}

inline Vector3
Reflect(Vector3 v, Vector3 normal) {
    return normal * 2.0f*Dot(v, normal) - v;
}

inline float 
FresnelAmount(float ior_exit, float ior_enter, Vector3 normal, Vector3 incident) {
    float r0 = (ior_exit - ior_enter) / (ior_exit + ior_enter);
    r0 *= r0;
    float ct = -Dot(normal, incident);
    if (ior_exit > ior_enter) {
        float n = ior_exit / ior_enter;
        float st_sq = n*n*(1.0f - ct*ct);
        if (st_sq > 1.0f) {
            return 1.0f;
        }
        ct = sqrtf(1.0f - st_sq);
    }
    float x = 1.0f - ct;
    // Yes, really, we need to do this.  Compiler cannot reorder float 
    // multiplication because FP math is (ノ｀Д´)ノ彡┻━┻
    float x2 = x*x;
    float x3 = x*x2;
    return r0 + (1.0f - r0)*x2*x3;
}

struct LightingResult {
    Vector4 direct_diffuse;
    Vector4 direct_specular;
};

static LightingResult
ShadeLight(Scene * scene, LightSource * light, Ray view_ray, Vector3 normal, Vector3 position, float specular_intensity) {
    LightingResult result = {};
    Ray shadow_ray = GetShadowRayForLight(position, light);
    Vector3 light_vector = shadow_ray.direction;
    switch (light->type) {
        case Light_Directional: {
           //if (!TraceRay(shadow_ray, scene, NULL)) {
                float spec_cos = Dot(view_ray.direction * -1.0f, Reflect(light_vector, normal));
                result.direct_diffuse = light->color * 2.0f * max(0.0f, Dot(normal, light_vector));
                result.direct_specular = light->color * powf(max(0.0f, spec_cos), specular_intensity);
           //}
        } break;
        case Light_Point: {
            Vector3 d = light->position - position;
            float light_dist_sq = Dot(d, d);
            RaycastHit hit;
            // No hit, *or* nearest hit is beyond the light itself
            if (!TraceRay(shadow_ray, scene, &hit) || hit.t*hit.t <= light_dist_sq) {
                // Distance attenuation
                float falloff_denom = (sqrtf(light_dist_sq) / light->falloff) + 1.0f;
                Vector4 light_color = light->color * (1.0f / (falloff_denom*falloff_denom));

                float spec_cos = Dot(view_ray.direction * -1.0f, Reflect(light_vector, normal));
                result.direct_diffuse = light_color * 2.0f * max(0.0f, Dot(normal, light_vector));
                result.direct_specular = light_color * powf(max(0.0f, spec_cos), specular_intensity);
            }
        } break;
        default: {
            Assert(false, "Unrecognized light type!");
        } break;
    }
    return result;
}

static Vector4
TraceRayColor(Ray ray, Scene * scene, s32 iters) {
    Vector4 color = Vector4(0.0f, 0.0f, 0.0f, 0.0f);
    RaycastHit hit;
    if (TraceRay(ray, scene, &hit)) {
        // Shade pixel
        Vector3 hit_p = hit.position + hit.normal * gParams.ray_bias;
        Vector3 hit_normal = hit.normal;
        Material * mat = hit.object->material;

        Vector4 direct_light;
        Vector4 direct_specular_light;
        for (u32 i = 0; i < scene->light_count; ++i) {
            LightingResult light_result = ShadeLight(scene, scene->lights + i, ray, hit_normal, hit_p, mat->specular_intensity);
            direct_light += light_result.direct_diffuse;
            direct_specular_light += light_result.direct_specular;
        }

        Vector4 indirect_light;
        Vector4 indirect_specular_light;
        if (iters > 0) {
            for (u32 samp = 0; samp < gParams.reflection_samples; ++samp) {
                float cos_theta = 0.0f;
                Ray reflect_ray = GetDiffuseReflectionRay(hit_p, hit_normal, &cos_theta);
                Vector4 reflect_color = TraceRayColor(reflect_ray, scene, iters - 1);
                indirect_light += reflect_color * cos_theta;
            }
            indirect_light /= gParams.reflection_samples;

            for (u32 samp = 0; samp < gParams.spec_samples; ++samp) {
                float cos_theta = 0.0f;
                // Ray reflect_ray = GetRayInCone(hit_p, Reflect(ray.direction, hit_normal), &cos_theta, DEG2RAD(90.0f));
                Ray reflect_ray = GetRayInCone(hit_p, Reflect(ray.direction, hit_normal), &cos_theta, 1.0f - DEG2RAD(30.0f));
                Vector4 spec_color = TraceRayColor(reflect_ray, scene, iters - 1);

                indirect_specular_light += spec_color;
            }
            indirect_specular_light /= gParams.spec_samples;
        }

        float object_reflectivity = 0.05f;// TODO
        float fresnel = FresnelAmount(1.0f, mat->index_of_refraction, hit_normal, ray.direction);
        float w_reflect = (object_reflectivity + (1.0f - object_reflectivity) * fresnel);
        float w_diffuse = 1.0f - w_reflect;

        color += mat->ambient_color * 0.01f;
        color += (indirect_light + direct_light) * mat->diffuse_color * w_diffuse;
        color += (indirect_specular_light + direct_specular_light) * mat->specular_color * w_reflect;

        color /= PI32;
        color = Color_FromNormal(hit_normal);
        if (hit.object->type == ObjectType_MeshGroup) {
            Vector3 interp_normal;
            u32 idx = hit.vertex0;
            MeshGroup * mg = hit.object->mesh_group;
            Mesh * mesh = hit.object->mesh;

            interp_normal += mesh->normals[mg->idx_normals[idx + 0]] * hit.bw.x;
            interp_normal += mesh->normals[mg->idx_normals[idx + 1]] * hit.bw.y;
            interp_normal += mesh->normals[mg->idx_normals[idx + 2]] * hit.bw.z;
            color = Color_FromNormal(interp_normal);
        }
    }
    else {
        color = gParams.background_color;
    }
    return color;
}

struct Framebuffer {
    u8 * bytes;
    u32 width;
    u32 height;
    u32 * DEBUG_rays_cast;
};

static void
WriteColor(Framebuffer * fb, u32 x, u32 y, float r, float g, float b, float a) {
    Vector4 c(Color_LinearToSRGB(r),
              Color_LinearToSRGB(g),
              Color_LinearToSRGB(b),
              Color_LinearToSRGB(a));

    u32 offset = (y * fb->width + x) * 4;
    Color_Pack(fb->bytes + offset, c);
}

struct Camera {
    float tan_a2;
    float aspect;
    float inv_width;
    float inv_height;

    Matrix33 world_from_cam;
};

static Camera
MakeCamera(float fov, Framebuffer * fb) {
    Camera cam = {
        tanf(DEG2RAD((fov / 2.0f))),
        (float)fb->width / (float)fb->height,
        1.0f / fb->width,
        1.0f / fb->height,
        Matrix33_FromToRotation(Vector3(0.0f, 0.0f, 1.0f), gParams.camera_facing),
    };

    Vector3 l = gParams.camera_facing;
    Vector3 up(0, 1, 0);

    Vector3 s = Cross(l, up);
    Vector3 u = Cross(s, l);

    cam.world_from_cam(0, 0) = s.x;
    cam.world_from_cam(1, 0) = s.y;
    cam.world_from_cam(2, 0) = s.z;
    cam.world_from_cam(0, 1) = u.x;
    cam.world_from_cam(1, 1) = u.y;
    cam.world_from_cam(2, 1) = u.z;
    cam.world_from_cam(0, 2) = -l.x;
    cam.world_from_cam(1, 2) = -l.y;
    cam.world_from_cam(2, 2) = -l.z;

    return cam;
}

inline Ray
MakeCameraRay(Camera * cam, Vector2 origin) {
    float rx = (2.0f * (origin.x + 0.5f) * cam->inv_width - 1.0f) * cam->tan_a2 * cam->aspect;
    float ry = (1.0f - 2.0f * (origin.y + 0.5f) * cam->inv_height) * cam->tan_a2;
    Vector3 dir = Normalize(Vector3(rx, ry, 1.0f));

    Ray ray;
    // TODO(bryan):  This may not be quite correct?
    ray.origin = gParams.camera_position;
    ray.direction = Normalize(cam->world_from_cam * dir) * -1.0f;
    return ray;
}

Vector2 SSAA_Offsets[] = {
#if 1
    // Four Rooks / Rotated Grid sampling pattern.
    Vector2(-0.375f,  0.125f),
    Vector2( 0.125f,  0.375f),
    Vector2( 0.375f, -0.125f),
    Vector2(-0.125f,  0.375f),
#else
    Vector2(0, 0)
#endif
};

static void
Render(Framebuffer * fb, Scene * scene) {
    Camera cam = MakeCamera(gParams.camera_fov, fb);
    for (u32 y = 0; y < fb->height; ++y) {
        // TIME_BLOCK("render row");
        for (u32 x = 0; x < fb->width; ++x) {
            // TIME_BLOCK("render pixel");
            gRayCount = 0;
            Vector4 color;
            // Super-sampled anti-aliasing
            for (u32 s = 0; s < array_count(SSAA_Offsets); ++s) {
                Vector2 pos = Vector2(x, y) + SSAA_Offsets[s];
                Ray ray = MakeCameraRay(&cam, pos);
                color += TraceRayColor(ray, scene, gParams.bounce_depth);
            }
            color /= array_count(SSAA_Offsets);
            WriteColor(fb, x, y, color.x, color.y, color.z, 1.0f);
            fb->DEBUG_rays_cast[x + y * fb->width] = gRayCount;
        }
    }
}

static void
UsageAndDie() {
    fprintf(stderr, "Incorrect arguments.\n");
    exit(1);
}

static void
Argv_ReadFloat(int argc, char ** argv, int arg_i, float * out) {
    arg_i++;

    if (arg_i >= argc) {
        UsageAndDie();
    }
    else {
        *out = atof(argv[arg_i]);
    }
}

static void
Argv_ReadU32(int argc, char ** argv, int arg_i, u32 * out) {
    arg_i++;

    if (arg_i >= argc) {
        UsageAndDie();
    }
    else {
        *out = atoi(argv[arg_i]);
    }
}

static void
Argv_ReadVec3(int argc, char ** argv, int arg_i, Vector3 * out) {
    Vector3 result;
    Argv_ReadFloat(argc, argv, arg_i + 0, &result.x);
    Argv_ReadFloat(argc, argv, arg_i + 1, &result.y);
    Argv_ReadFloat(argc, argv, arg_i + 2, &result.z);
    *out = result;
}

static void
Argv_ReadVec4(int argc, char ** argv, int arg_i, Vector4 * out) {
    Vector4 result;
    Argv_ReadFloat(argc, argv, arg_i + 0, &result.x);
    Argv_ReadFloat(argc, argv, arg_i + 1, &result.y);
    Argv_ReadFloat(argc, argv, arg_i + 2, &result.z);
    Argv_ReadFloat(argc, argv, arg_i + 3, &result.w);
    *out = result;
}

// Initialize global parameters.
//
// We have 3 sources for global params:
// - Static defaults, set in code here.
// - Config INI, loaded ontop of the defaults.
// - Command-line arguments, overriding either of those.
//
static void
InitParams(int argc, char ** argv) {
    // Defaults
    gParams.ray_bias = 1e-3;
    gParams.reflection_samples = 8;
    gParams.spec_samples = 8;
    gParams.bounce_depth = 2;
    gParams.background_color = Vector4(0.0f, 0.3f, 0.5f, 1.0f);
    gParams.camera_fov = 60.0f;
    // gParams.camera_position = Vector3(0.0f, 15.0f, 10.0f);
    // gParams.camera_facing = Normalize(Vector3(0.0f, -0.5f, -1));
    gParams.camera_position = Vector3(20.0f, 100.0f, 0.0f);
    gParams.camera_facing = Normalize(Vector3(1.0f, 0.0f, 0.0f));
    gParams.image_output_filename = strdup("rt_out.png"); // Need this to be on the heap; might be free'd later.
    gParams.use_system_rand = 0;

    // TODO(bryan):  INI

    // Parse argv
    for (int arg_i = 1; arg_i < argc; ++arg_i) {
        char * arg = argv[arg_i];
#define STATIC_STRNCMP(_static, _dynamic) (!strncmp(_static, _dynamic, array_count(_static) - 1))
        if (STATIC_STRNCMP("--ray-bias", arg)) {
            Argv_ReadFloat(argc, argv, arg_i, &gParams.ray_bias);
            arg_i++;
        }
        else if (STATIC_STRNCMP("--reflection_samples", arg) || STATIC_STRNCMP("-rs", arg)) {
            Argv_ReadU32(argc, argv, arg_i, &gParams.reflection_samples);
            arg_i++;        
        }
        else if (STATIC_STRNCMP("--specular_samples", arg) || STATIC_STRNCMP("-ss", arg)) {
            Argv_ReadU32(argc, argv, arg_i, &gParams.spec_samples);
            arg_i++;
        }
        else if (STATIC_STRNCMP("--bounce_depth", arg) || STATIC_STRNCMP("-b", arg)) {
            Argv_ReadU32(argc, argv, arg_i, &gParams.bounce_depth);
            arg_i++;
        }
        else if (STATIC_STRNCMP("--background_color", arg) || STATIC_STRNCMP("-bg", arg)) {
            Argv_ReadVec4(argc, argv, arg_i, &gParams.background_color);
            arg_i += 4;
        }
        else if (STATIC_STRNCMP("--fov", arg)) {
            Argv_ReadFloat(argc, argv, arg_i, &gParams.camera_fov);
            arg_i++;
        }
        else if (STATIC_STRNCMP("--camera_position", arg)) {
            Argv_ReadVec3(argc, argv, arg_i, &gParams.camera_position);
            arg_i += 3;
        }
        else if (STATIC_STRNCMP("--camera_facing", arg)) {
            Argv_ReadVec3(argc, argv, arg_i, &gParams.camera_facing);
            arg_i += 3;
        }
        else if (STATIC_STRNCMP("--output", arg) || STATIC_STRNCMP("-o", arg)) {
            if (gParams.image_output_filename) {
                free(gParams.image_output_filename);
            }
            // We may not actually need this copy, argv is going to stay around,
            // but in theory something else could modify the string, so copy to be
            // safe.
            //
            // Use of strdup is questionable, but argv is *guaranteed* to be 
            // null-terminated, so this is fine.
            gParams.image_output_filename = strdup(argv[arg_i + 1]);
            arg_i++;
        }
        else if (STATIC_STRNCMP("--rng", arg)) {
            Argv_ReadU32(argc, argv, arg_i, &gParams.use_system_rand);
            arg_i++;            
        }
#undef STATIC_STRNCMP
    }
}

static Material *
MakeMaterial(Vector4 color) {
    Material * result = (Material *)calloc(1, sizeof(Material));
    result->specular_intensity = 10.0f;
    result->index_of_refraction = 1.5f;
    result->alpha = 1.0f;
    result->ambient_color = color;
    result->diffuse_color = color;
    result->specular_color = Vector4(1, 1, 1, 1);

    return result;
}

static Scene
InitScene() {
    Vector3 SUN_DIR = Normalize(Vector3(1.0f, 0.5f, 1.0f));
    SceneObject * spheres = (SceneObject *)calloc(4, sizeof(SceneObject));
    spheres[0].sphere.center = Vector3(0.0f, 0.0f, -10.0f);
    spheres[0].sphere.radius = 2.0f;
    spheres[0].material = MakeMaterial(Vector4(0.0f, 1.0f, 0.0f, 1.0f));

    spheres[1].sphere.center = spheres[0].sphere.center + SUN_DIR * 4.0f; //Vector3(3.5f, 0.0f, -10.0f);
    spheres[1].sphere.center.z -= 1.0f;
    spheres[1].sphere.radius = 1.0f;
    spheres[1].material = MakeMaterial(Vector4(1.0f, 0.0f, 0.0f, 1.0f));

    spheres[2].sphere.center = Vector3(0.0f, 0.0f, -5.0f);
    spheres[2].sphere.radius = 0.5f;
    spheres[2].material = MakeMaterial(Vector4(0.0f, 0.5f, 1.0f, 1.0f));

    spheres[3].sphere.center = Vector3(0, -1002, -10);
    spheres[3].sphere.radius = 1000.0f;
    spheres[3].material = MakeMaterial(Vector4(1, 1, 1, 1));

    // LightSource lights[2];
    // lights[0].type = Light_Directional;
    // lights[0].color = Vector4(1.0f, 1.0f, 1.0f, 1.0f);
    // lights[0].facing = SUN_DIR * -1.0f;

    // lights[1].type = Light_Directional;
    // lights[1].color = Vector4(0.5f, 0.5f, 0.5f, 1.0f);
    // lights[1].facing = Normalize(Vector3(10.0f, -2.0f, 0.0f));

    LightSource * lights = (LightSource *)calloc(2, sizeof(LightSource));
    lights[0].type = Light_Directional;
    lights[0].color = Vector4(1, 1, 1, 1);
    lights[0].facing = Vector3(0, -1, 0);

    lights[1].type = Light_Directional;
    lights[1].color = Vector4(1.0f, 1.0f, 1.0f, 1.0f);
    lights[1].facing = Normalize(Vector3(1.0f, -1.0f, 0.0f));

    Scene scene;
    // scene.objects = spheres;
    // scene.object_count = 4;
    scene.lights = lights;
    scene.light_count = 1;

    return scene;    
}

int main(int argc, char ** argv) {
    Mesh * mesh;
    BoundingHierarchy hierarchy;
    Matrix33 transform = Matrix33_FromEuler(0.0f, 0.0f, PI32);
    transform.SetIdentity();
    {
        TIME_BLOCK("Load Mesh");
        // mesh = ParseOBJ("D:/Users/Bryan/Desktop/meshes/san-miguel/sanMiguel/sanMiguel.obj");
        mesh = ParseOBJ("D:/Users/Bryan/Desktop/meshes/crytek-sponza/sponza.obj", transform);
    }
    {
        TIME_BLOCK("Build Hierarchy");
        BuildHierarchy(&hierarchy, mesh);
    }

    InitParams(argc, argv);
    gParams.camera_position = hierarchy.spheres[0].s.center;
    gParams.camera_position.z -= 200.0f;
    gParams.camera_position.x += 500.0f;
    gParams.camera_position.y *= 0.25f;

    Framebuffer fb;
    // fb.width = 640;
    // fb.height = 480;
    fb.width = 128;
    fb.height = 96;
    fb.bytes = (u8 *)calloc(4, fb.width * fb.height);
    fb.DEBUG_rays_cast = (u32 *)calloc(sizeof(u32), fb.width*fb.height);

    u32 total_tris = 0;
    Scene scene = InitScene();
    scene.hierarchy = &hierarchy;
    // TODO(bryan):  We need to actually load materials.  In the meantime, give everything a default
    // so that we can just test the loading / mesh tracing.
    scene.default_mat = MakeMaterial(Vector4(0.75f, 0.5f, 0.75f, 1.0f));
    for (u32 i = 0; i < hierarchy.mesh_groups.size(); ++i) {
        MeshGroup * mg = hierarchy.mesh_groups[i];
        if (mg) {
            total_tris += mg->idx_positions.size() / 3;
        }
        SceneObject * obj = (SceneObject *)calloc(1, sizeof(SceneObject));
        obj->mesh_group = mg;
        obj->mesh = mesh;
        obj->type = ObjectType_MeshGroup;
        obj->material = scene.default_mat;

        scene.objects.push_back(obj);
    }
    printf("Triangles: %u\n", total_tris);

    Render(&fb, &scene);

    stbi_write_png(gParams.image_output_filename, fb.width, fb.height, 4, fb.bytes, 0);

    float scale = 1.0f / (float)(1 << 16);

    s64 total_rays_cast = 0;
    s64 max_rays_cast = 0;
    for (u32 y = 0; y < fb.height; ++y) {
        for (u32 x = 0; x < fb.width; ++x) {
            u32 count = fb.DEBUG_rays_cast[x + y * fb.width];
            total_rays_cast += count;
            max_rays_cast = max(max_rays_cast, count);

            float h = (1.0f - (float)(count * scale)) * 240.0f;
            Vector4 hsv(h/360.0f, 1.0f, 5.0f, 1.0f);
            Vector4 rgb = Color_HSVToRGB(hsv);

            u32 offset = (y * fb.width + x) * 4;
            Color_Pack(fb.bytes + offset, rgb);
        }
    }

    double total_pixels = (double)(fb.height*fb.width);
    double mean = (double)total_rays_cast / total_pixels;
    double variance = 0.0;
    for (u32 i = 0; i < fb.height * fb.width; ++i) {
        u32 count = fb.DEBUG_rays_cast[i];
        double delta = (double)count - mean;
        variance += delta*delta;
    }
    variance /= total_pixels;
    double std_dev = sqrt(variance);

    printf("Total rays cast: %lld\n", total_rays_cast);
    printf("Max rays cast:   %lld\n", max_rays_cast);
    printf("Mean:            %8.8f\n", mean);
    printf("Variance:        %8.8f\n", variance);
    printf("Std. Dev:        %8.8f\n", std_dev);

    stbi_write_png("rt_rays_out.png", fb.width, fb.height, 4, fb.bytes, 0);

    return 0;
}