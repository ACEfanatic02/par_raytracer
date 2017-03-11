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
#include "scene.h"
#include "globals.h"

#include "texture.cpp"
#include "obj_parser.cpp"
#include "bsphere.cpp"

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
    gSpheresChecked++;
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
    Vector3 p = ray.origin + ray.direction * t;
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

    if (b > 0.0f) return false;

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

    Vector3 q = ray.origin + ray.direction;
    Vector3 qp = ray.origin - q;

    Vector3 normal = Cross(ab, ac);
    float d = Dot(qp, normal);
    if (d <= 0.0f) {
        return false;
    }

    Vector3 ap = ray.origin - a;
    float t = Dot(ap, normal);

    if (t < 0.0f) {
        return false;
    }
    // We're closer than the t value passed in, so the caller doesn't care about this hit.
    if (t > out_hit->t * d) {
        return false;
    }

    // At this point, we know we hit the plane of the triangle, now check if
    // we're actually inside it.
    Vector3 e = Cross(qp, ap);
    float v = Dot(ac, e);
    if (v < 0.0f || v > d) return false;
    float w = -Dot(ab, e);
    if (w < 0.0f || (v + w) > d) return false;

    float ood = 1.0f / d;
    out_hit->t = t * ood;
    out_hit->bw.y = v * ood;
    out_hit->bw.z = w * ood;
    out_hit->bw.x = 1.0f - out_hit->bw.y - out_hit->bw.z;
    out_hit->position = ray.origin + ray.direction * out_hit->t;
    out_hit->normal = Normalize(normal);

    return true;
}

static bool
IntersectRayMesh(Ray ray, SceneObject * obj, RaycastHit * out_hit) {
    gMeshesChecked++;
    assert(obj->type == ObjectType_MeshGroup);
    MeshGroup * mesh_group = obj->mesh_group;
    Mesh * mesh = obj->mesh;

    bool hit = false;
    float max_t = out_hit ? out_hit->t : FLT_MAX;
    RaycastHit best_hit = { max_t };
    for (u32 i = 0; i < mesh_group->idx_positions.size(); i += 3) {
        u32 idx_a = mesh_group->idx_positions[i + 0];
        u32 idx_b = mesh_group->idx_positions[i + 1];
        u32 idx_c = mesh_group->idx_positions[i + 2];

        Vector3 pa = mesh->positions[idx_a];
        Vector3 pb = mesh->positions[idx_b];
        Vector3 pc = mesh->positions[idx_c];

        RaycastHit current_hit = { best_hit.t };
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

    std::vector<u32> check_spheres;
    check_spheres.push_back(0);
    while (check_spheres.size() > 0) {
        u32 i = check_spheres.back();
        check_spheres.pop_back();
        BoundingSphere s = scene->hierarchy->spheres[i];
        RaycastHit sphere_test_hit;
        if (IntersectRaySphere(ray, s.s, &sphere_test_hit)) {
            if (sphere_test_hit.t > best_hit.t) {
                // We've already seen a closer hit, no need to check
                // this node's children.
                continue;
            }
            if (s.c0 && s.c1) {
#if 0
                // NOTE(bryan): Experimentally, this heuristic actually leads 
                // to more work?  What.
                BoundingSphere s0 = scene->hierarchy->spheres[s.c0];
                BoundingSphere s1 = scene->hierarchy->spheres[s.c1];
                Vector3 v0 = ray.origin - s0.s.center;
                Vector3 v1 = ray.origin - s1.s.center;
                float lv0_sq = Dot(v0, v0);
                float lv1_sq = Dot(v1, v1);

                // We insert the closest sphere first; this increases
                // our chances of culling the other sphere without 
                // checking its children.
                //
                // This doesn't take into account each sphere's radius, 
                // but that computation may not be worth the effort?
                if (lv0_sq < lv1_sq) {
                    check_spheres.push_back(s.c0);
                    check_spheres.push_back(s.c1);
                }
                else {
                    check_spheres.push_back(s.c1);                    
                    check_spheres.push_back(s.c0);
                }
#else
                check_spheres.push_back(s.c0);
                check_spheres.push_back(s.c1);
#endif
            }
            else {
                assert(s.c0 == 0 && s.c1 == 0);
                // We've hit a leaf sphere, try intersecting its mesh
                {
                    RaycastHit current_hit = { best_hit.t };
                    SceneObject * obj = scene->objects[i];
                    if (IntersectRayMesh(ray, obj, &current_hit)) {
                        if (current_hit.t < best_hit.t) {
                            best_hit = current_hit;
                            hit = true;
                        }
                    }                    
                }
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
    if (!rng_init) {
        Random_Seed(&rng, 0);
        rng_init = true;
    }
    return Random_NextFloat01(&rng);      
}

static float
GetRandFloat11() {
    if (!rng_init) {
        Random_Seed(&rng, 0);
        rng_init = true;
    }
    return Random_NextFloat11(&rng);
}

static Ray
GetRayInCone(Vector3 origin, Vector3 normal, float * cos_theta, float min_cos) {
    Ray ray;
    ray.origin = origin;

    // Pick a random angle (this will be rotation around normal)
    float angle = GetRandFloat01() * PI32 * 2.0f;
    // Pick random height along normal
    float z = Lerp(min_cos, 1.0f, GetRandFloat01());
    z = Clamp(z, min_cos, 1.0f);
    // Project that height onto the tangent plane
    float xy = (1.0f - z*z);
    // Calculate the tangent components
    float ct = cosf(angle);
    float st = sinf(angle);
    float x = xy * ct;
    float y = xy * st;
    // Renormalize just to be sure.

    static const float epsilon = 1e-5;
    Vector3 v = Normalize(Vector3(x, y, z));
    float d = Dot(normal, Vector3(0, 0, 1));
    if (d > 1.0f - epsilon) {
        ray.direction = v;
    }
    else if (d < -1.0f + epsilon) {
        ray.direction = -v;
    }
    else {
        Vector3 axis = Normalize(Cross(Vector3(0, 0, 1), normal));
        ray.direction = Normalize(Matrix33_FromAxisAngle(axis, acosf(d)) * v);
    }

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
    float ct = max(0.0f, -Dot(normal, incident));
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
    float rv = r0 + (1.0f - r0)*x2*x3;

    assert(rv >= -1.0f && rv <= 1.0f);

    return rv;
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
           if (!TraceRay(shadow_ray, scene, NULL)) {
                float spec_cos = Dot(view_ray.direction * -1.0f, Reflect(light_vector, normal));
                result.direct_diffuse = light->color * 2.0f * max(0.0f, Dot(normal, light_vector));
                result.direct_specular = light->color * powf(max(0.0f, spec_cos), specular_intensity);
           }
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
        Vector4 ambient_color = mat->ambient_color;
        Vector4 diffuse_color = mat->diffuse_color;
        Vector4 specular_color = mat->specular_color;

        if (hit.object->type == ObjectType_MeshGroup) {
            Vector3 interp_normal;
            u32 idx = hit.vertex0;
            MeshGroup * mg = hit.object->mesh_group;
            Mesh * mesh = hit.object->mesh;
            interp_normal += mesh->normals[mg->idx_normals[idx + 0]] * hit.bw.x;
            interp_normal += mesh->normals[mg->idx_normals[idx + 1]] * hit.bw.y;
            interp_normal += mesh->normals[mg->idx_normals[idx + 2]] * hit.bw.z;
            hit_normal = Normalize(interp_normal);

            Vector2 uv;
            uv += mesh->texcoords[mg->idx_texcoords[idx + 0]] * hit.bw.x;
            uv += mesh->texcoords[mg->idx_texcoords[idx + 1]] * hit.bw.y;
            uv += mesh->texcoords[mg->idx_texcoords[idx + 2]] * hit.bw.z;
            if (mat->alpha <= 1.0f || mat->alpha_texture) {
                float alpha = mat->alpha;
                if (mat->alpha_texture) {
                    alpha *= Texture_SampleBilinear(mat->alpha_texture, uv.x, uv.y).x;
                }

                if (alpha <= 0.5f) {
                    // TODO(bryan):  If the pixel is translucent, we want to blend between transparent and opaque.
                    ray.origin = hit.position + ray.direction;// * gParams.ray_bias * 2.0f;
                    return TraceRayColor(ray, scene, iters);
                }
            }
            if (mat->ambient_texture) {
                ambient_color *= Texture_SampleBilinear(mat->ambient_texture, uv.x, uv.y);
            }
            if (mat->diffuse_texture) {
                diffuse_color *= Texture_SampleBilinear(mat->diffuse_texture, uv.x, uv.y);
            }
            if (mat->specular_texture) {
                specular_color = Texture_SampleBilinear(mat->specular_texture, uv.x, uv.y);
            }

            if (mat->bump_texture) {
                // Normal mapping.
                if (idx == 129) {
                    int break_here = 0;
                }
                Vector3 tangent;
                tangent += mesh->tangents[mg->idx_normals[idx + 0]] * hit.bw.x;
                tangent += mesh->tangents[mg->idx_normals[idx + 1]] * hit.bw.y;
                tangent += mesh->tangents[mg->idx_normals[idx + 2]] * hit.bw.z;
                tangent = Normalize(tangent);

                Vector3 bitangent = Normalize(Cross(hit_normal, tangent));

                Vector4 sample = Texture_SampleBilinear(mat->bump_texture, uv.x, uv.y);
                Vector3 sample_normal = Vector3(sample.x, sample.y, sample.z) * 2.0f - Vector3(1.0f, 1.0f, 1.0f);

                Matrix33 world_from_tangent_space;
                world_from_tangent_space(0, 0) = tangent.x;
                world_from_tangent_space(1, 0) = tangent.y;
                world_from_tangent_space(2, 0) = tangent.z;

                world_from_tangent_space(0, 1) = bitangent.x;
                world_from_tangent_space(1, 1) = bitangent.y;
                world_from_tangent_space(2, 1) = bitangent.z;
                
                world_from_tangent_space(0, 2) = hit_normal.x;
                world_from_tangent_space(1, 2) = hit_normal.y;
                world_from_tangent_space(2, 2) = hit_normal.z;

                sample_normal = world_from_tangent_space * sample_normal;
                hit_normal = sample_normal;

#if 1
                assert(isfinite(hit_normal.x));
                assert(isfinite(hit_normal.y));
                assert(isfinite(hit_normal.z));
#endif
            }
        }
        
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
                Ray reflect_ray;
                float d;
                do {
                    reflect_ray = GetDiffuseReflectionRay(hit_p, hit_normal, &cos_theta);
                    d = Dot(hit_normal, reflect_ray.direction);
                    // NOTE(bryan):  BUG
                    // It's unclear how we're getting rays that point away from
                    // the normal, since we're specifically selecting rays in the hemisphere.
                } while (d < 0.0f);
                Vector4 reflect_color = TraceRayColor(reflect_ray, scene, iters - 1);
                indirect_light += reflect_color * d;
            }
            // indirect_light /= gParams.reflection_samples;

            for (u32 samp = 0; samp < gParams.spec_samples; ++samp) {
                float cos_theta = 0.0f;
                // Ray reflect_ray = GetRayInCone(hit_p, Reflect(ray.direction, hit_normal), &cos_theta, DEG2RAD(90.0f));
                Ray reflect_ray = GetRayInCone(hit_p, Reflect(ray.direction, hit_normal), &cos_theta, 1.0f - DEG2RAD(30.0f));
                Vector4 spec_color = TraceRayColor(reflect_ray, scene, iters - 1);

                indirect_specular_light += spec_color;
            }
            // indirect_specular_light /= gParams.spec_samples;
        }

        float object_reflectivity = 0.05f;// TODO
        float fresnel = FresnelAmount(1.0f, mat->index_of_refraction, hit_normal, ray.direction);
        float w_reflect = (object_reflectivity + (1.0f - object_reflectivity) * fresnel);
        float w_diffuse = 1.0f - w_reflect;

        color += ambient_color * 0.1f;
        color += (indirect_light + direct_light) * diffuse_color * w_diffuse;
        color += (indirect_specular_light + direct_specular_light) * specular_color * w_reflect;
        
        // color /= PI32;
        assert(color.x >= 0.0f);
        assert(color.y >= 0.0f);
        assert(color.z >= 0.0f);
        assert(color.w >= 0.0f);

        // color = Color_FromNormal(hit_normal);
        // if (hit.object->type == ObjectType_MeshGroup) {
        //     Vector3 interp_normal;
        //     u32 idx = hit.vertex0;
        //     MeshGroup * mg = hit.object->mesh_group;
        //     Mesh * mesh = hit.object->mesh;

        //     interp_normal += mesh->normals[mg->idx_normals[idx + 0]] * hit.bw.x;
        //     interp_normal += mesh->normals[mg->idx_normals[idx + 1]] * hit.bw.y;
        //     interp_normal += mesh->normals[mg->idx_normals[idx + 2]] * hit.bw.z;
        //     color = Color_FromNormal(hit_normal);
        // }
    }
    else {
        color = gParams.background_color;
    }
    return color;
}
