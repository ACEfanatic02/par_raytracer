#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "lib/stb_image_write.h"

#include "raytracer.cpp"

struct Framebuffer {
    Vector4 * pixels;
    u32 width;
    u32 height;
    u32 * DEBUG_rays_cast;
};

static void
WriteColor(Framebuffer * fb, u32 x, u32 y, float r, float g, float b, float a) {
    u32 idx = fb->width * y + x;
    fb->pixels[idx].x = r;
    fb->pixels[idx].y = g;
    fb->pixels[idx].z = b;
    fb->pixels[idx].w = a;
}

static float
LogAverageLuma(Framebuffer * fb) {
    // Log-average / geometric mean of scene luma:
    //
    // L_w = exp(1/N * sum(log(d + L(x, y))))
    //
    float lavg = 0.0f;
    for (u32 y = 0; y < fb->height; ++y) {
        for (u32 x = 0; x < fb->width; ++x) {
            u32 idx = y * fb->width + x;
            Vector4 c = fb->pixels[idx];
            float cluma = Color_Luma(c);
            if (cluma > 0.0f) {
                lavg += logf(0.01f + cluma);
            }
            else {
                printf("Non-positive luma at (%u, %u): %f\n", x, y, cluma);
            }
        }
    }
    return expf(lavg / (float)(fb->width * fb->height));
}

static void
WriteFramebufferImage(Framebuffer * fb, char * filename) {
    float scene_luma = LogAverageLuma(fb);
    u8 * buffer = (u8 *)calloc(fb->width * fb->height, 4);
    printf("scene_luma = %f\n", scene_luma);
    for (u32 y = 0; y < fb->height; ++y) {
        for (u32 x = 0; x < fb->width; ++x) {
            u32 idx = y * fb->width + x;
            Vector4 c = fb->pixels[idx];

            float key_alpha = 0.18f;
            float pixel_luma = Color_Luma(c);
            float l_xy = key_alpha * pixel_luma / scene_luma;
            float l_d = l_xy / (1.0f + l_xy);

            float scale = l_d / pixel_luma;
            c.x *= scale;
            c.y *= scale;
            c.z *= scale;

            Color_Pack(buffer + idx * 4, c);
        }
    }

    stbi_write_png(filename, fb->width, fb->height, 4, buffer, 0);
    free(buffer);
}

struct Camera {
    float tan_a2;
    float aspect;
    float inv_width;
    float inv_height;

    Vector3 camera_position;
    Vector3 camera_forward;
    Vector3 camera_right;
    Vector3 camera_up;
};

static Camera
MakeCamera(float fov, Framebuffer * fb) {
    Camera cam;
    cam.tan_a2 = tanf(DEG2RAD(fov / 2.0f));
    cam.aspect = (float)fb->width / (float)fb->height;
    cam.inv_width = 1.0f / (float)fb->width;
    cam.inv_height = 1.0f / (float)fb->height;
    cam.camera_position = gParams.camera_position;

    // TODO(bryan):  This will not work if the camera is *facing* near world up.
    Vector3 up(0.0f, 1.0f, 0.0f);

    cam.camera_forward = Normalize(gParams.camera_facing);
    cam.camera_right   = Normalize(Cross(cam.camera_forward, up));
    cam.camera_up      = Normalize(Cross(cam.camera_right, cam.camera_forward));

    return cam;
}

inline Ray
MakeCameraRay(Camera * cam, Vector2 origin) {
    float normalized_x = 2.0f * (origin.x + 0.5f) * cam->inv_width - 1.0f;
    float normalized_y = 1.0f - 2.0f * (origin.y + 0.5f) * cam->inv_height;

    Vector3 dir = cam->camera_forward 
                + cam->camera_right * cam->tan_a2 * cam->aspect * normalized_x
                + cam->camera_up * cam->tan_a2 * normalized_y;

    Ray ray;
    ray.origin = cam->camera_position;
    ray.direction = Normalize(dir); 
    return ray;
}

inline float
Color_Distance(Vector4 a, Vector4 b) {
    // Manhattan distance to properly identify differences in hue as well.
    return fabsf(a.x - b.x)
         + fabsf(a.y - b.y)
         + fabsf(a.z - b.z)
         + fabsf(a.w - b.w);
}

static float
CalculateLocalVariance(Framebuffer *fb, u32 x, u32 y) {
    u32 min_x = min(x - 2, 0);
    u32 max_x = min(x + 2, fb->width);
    u32 min_y = min(y - 2, 0);
    u32 max_y = min(y + 2, fb->height);

    float sum = 0.0f;
    u32 count = (max_x - min_x) * (max_y - min_y) - 1;
    Vector4 mean;
    for (u32 yy = min_y; yy <= max_y; ++yy) {
        for (u32 xx = min_x; xx <= max_x; ++xx) {
            mean += fb->pixels[yy*fb->width + xx];
        }
    }

    for (u32 yy = min_y; yy <= max_y; ++yy) {
        for (u32 xx = min_x; xx <= max_x; ++xx) {
            u32 idx = yy * fb->width + xx;

            Vector4 c0 = fb->pixels[idx];
            float d = Color_Distance(c0, mean);
            sum += d*d;
        }
    }
    return sum / (float)count;
}

static void
MakeVarianceMap(Framebuffer *fb, float * out_var) {
    for (u32 y = 0; y < fb->height; ++y) {
        for (u32 x = 0; x < fb->width; ++x) {
            u32 idx = y * fb->width + x;
            out_var[idx] = CalculateLocalVariance(fb, x, y); 
        }
    }
}

#include <thread>
#include <atomic>

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

Vector2 * sample_patterns = SSAA_Offsets;
u32 sample_pattern_count = array_count(SSAA_Offsets);

struct RenderJob {
    Camera * cam;
    Scene * scene;
    Framebuffer * fb;
    u32 * sample_counts;
    u32 y;
};

static void
RenderTask(RenderJob * job) {
    for (u32 x = 0; x < job->fb->width; ++x) {
        Vector4 color;
        for (u32 s = 0; s < sample_pattern_count; ++s) {
            Vector2 pos = Vector2(x, job->y) + sample_patterns[s];
            Ray ray = MakeCameraRay(job->cam, pos);
            color += TraceRayColor(ray, job->scene, gParams.bounce_depth);
            job->sample_counts[x + job->y * job->fb->width]++;
        }
        color /= sample_pattern_count;
        WriteColor(job->fb, x, job->y, color.x, color.y, color.z, 1.0f);
    }
}

std::vector<RenderJob> task_list;
std::atomic<u32> next_task;

static void
RenderQueueWorker() {
    while (next_task < task_list.size()) {
        u32 task_id = next_task++;

        RenderJob * j = &task_list[task_id];
        {
            TIME_BLOCK("Render Row");
            RenderTask(j); 
        }
    }
}

static void
Render(Camera * cam, Framebuffer * fb, Scene * scene) {
    // TODO(bryan):  Need thread-local storage for these debug counters if we want
    // non-garbage values for multithreaded runs.
    gSpheresChecked = 0;
    gMeshesChecked = 0;

    u32 * sample_counts = (u32 *)calloc(fb->width * fb->height, sizeof(u32));

    sample_pattern_count = 8;
    sample_patterns = (Vector2 *)calloc(sample_pattern_count, sizeof(Vector2));
    for (u32 i = 0; i < sample_pattern_count; ++i) {
        sample_patterns[i].x = GetRandFloat11() * 0.5f;
        sample_patterns[i].y = GetRandFloat11() * 0.5f;
    }

    for (u32 y = 0; y < fb->height; ++y) {
        RenderJob job;
        job.cam = cam;
        job.scene = scene;
        job.fb = fb;
        job.sample_counts = sample_counts;
        job.y = y;

        task_list.push_back(job);
    }

    {
        TIME_BLOCK("Render MT");
        u32 thread_count = 2;//std::thread::hardware_concurrency() - 1;
        std::vector<std::thread> threads;
        for (u32 i = 0; i < thread_count; ++i) {
            threads.push_back(std::thread(RenderQueueWorker));
        }
        printf("%u render worker threads started.\n", thread_count);
        // RenderQueueWorker();

        for (u32 i = 0; i < thread_count; ++i) {
            threads[i].join();
        }
    }

    WriteFramebufferImage(fb, "rt_out_beforevar.png");

    printf("Start variance reduction\n");
    u32 max_variance_reduction_passes = 8;

    float * var_map = (float *)calloc(fb->width * fb->height, sizeof(float));
    for (u32 pass = 0; pass < max_variance_reduction_passes; ++pass) {
        const float variance_threshold = 3.0f;
        {
            TIME_BLOCK("Variance Map");
            MakeVarianceMap(fb, var_map);
        }
        for (u32 y = 0; y < fb->height; ++y) {
            TIME_BLOCK("Render Row");
            u32 skip_count = 0;
            for (u32 x = 0; x < fb->width; ++x) {
                u32 idx = y * fb->width + x;
                if (var_map[idx] < variance_threshold) {
                    skip_count++;
                    continue;
                }

                gRayCount = 0;
                Vector4 color = fb->pixels[idx];

                float off_x = GetRandFloat11() * 0.5f;
                float off_y = GetRandFloat11() * 0.5f;
                Vector2 pos = Vector2(x + off_x, y + off_y);
                Ray ray = MakeCameraRay(cam, pos);
                u32 sample_count = sample_counts[idx]++;
                Vector4 sample_color = TraceRayColor(ray, scene, gParams.bounce_depth);

                // Numerically robust incremental average.
                // http://realtimecollisiondetection.net/blog/?p=48
                color = color * ((float)sample_count / (sample_count + 1)) 
                      + sample_color / (float)sample_count;
                
                WriteColor(fb, x, y, color.x, color.y, color.z, 1.0f);
                fb->DEBUG_rays_cast[x + y * fb->width] += gRayCount;
            }
            printf("Skipped %d pixels\n", skip_count);
        }
        char buffer[256];
        _snprintf(buffer, sizeof(buffer), "rt_out_pass%02d.png", pass + 1);
        WriteFramebufferImage(fb, buffer);
    }
    free(var_map);

    u32 pixel_count = fb->height*fb->width;
    printf("Spheres checked:    %llu\n", gSpheresChecked);
    printf("   average / pixel: %f\n", (double)gSpheresChecked / pixel_count);
    printf("Meshes checked:     %llu\n", gMeshesChecked);
    printf("   average / pixel: %f\n", (double)gMeshesChecked / pixel_count);
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
    // gParams.reflection_samples = 8;
    // gParams.spec_samples = 8;
    gParams.reflection_samples = 1;
    gParams.spec_samples = 1;
    gParams.bounce_depth = 2;
    gParams.background_color = Vector4(0.0f, 0.3f, 0.5f, 1.0f);
    gParams.camera_fov = 60.0f;
    // gParams.camera_position = Vector3(0.0f, 15.0f, 10.0f);
    // gParams.camera_facing = Normalize(Vector3(0.0f, -0.5f, -1));
    gParams.camera_position = Vector3(475.0f, 100.0f, 0.0f);
    gParams.camera_facing = Normalize(Vector3(1.0f, 0.0f, 1.5f));
    gParams.image_output_filename = strdup("rt_out.png"); // Need this to be on the heap; might be free'd later.

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
    LightSource * lights = (LightSource *)calloc(3, sizeof(LightSource));
    lights[0].type = Light_Directional;
    lights[0].color = Vector4(0.9f, 1.0f, 0.95f, 1.0f) * 4.0f;
    lights[0].facing = Normalize(Vector3(1.0f, -1.5f, 0.15f));

    lights[1].type = Light_Directional;
    lights[1].color = Vector4(1.0f, 1.0f, 1.0f, 1.0f) * 1.0f;
    lights[1].facing = Normalize(Vector3(0.0f, -1.0f, 0.0f));

    // lights[0].type = Light_Directional;
    // lights[0].color = Vector4(1, 1, 1, 1) * 4.0f;
    // lights[0].facing = Vector3(0, -1, 0);

    // lights[1].type = Light_Directional;
    // lights[1].color = Vector4(1.0f, 1.0f, 1.0f, 1.0f);
    // lights[1].facing = Normalize(Vector3(1.0f, -1.0f, 0.0f));

    // lights[2].type = Light_Point;
    // lights[2].color = Vector4(1.0f, 1.0f, 1.0f, 1.0f);
    // lights[2].position = Vector3(500.0f, 250.0f, 0.0f);

    Scene scene;
    scene.lights = lights;
    scene.light_count = 1;

    return scene;
}

int main(int argc, char ** argv) {
    InitParams(argc, argv);

    Framebuffer fb;
    // fb.width = 640;
    // fb.height = 480;
    fb.width = 128*2;
    fb.height = 96*2;
    // fb.width  = 64;
    // fb.height = 48;
    fb.pixels = (Vector4 *)calloc(fb.width * fb.height, sizeof(Vector4));
    fb.DEBUG_rays_cast = (u32 *)calloc(fb.width*fb.height, sizeof(u32));

    Camera cam = MakeCamera(gParams.camera_fov, &fb);
    Mesh * mesh;
    BoundingHierarchy hierarchy;
    Matrix33 transform;
    transform.SetIdentity();
    {
        TIME_BLOCK("Load Mesh");
        // mesh = ParseOBJ("D:/Users/Bryan/Desktop/meshes/san-miguel/sanMiguel/sanMiguel.obj", transform);
        mesh = ParseOBJ("D:/Users/Bryan/Desktop/meshes/crytek-sponza/", "sponza.obj", transform);
    }
    {
        TIME_BLOCK("Calculate Tangents");
        CalculateTangents(mesh);
    }
    {
        // Verify normal vectors
        for (u32 i = 0; i < mesh->normals.size(); ++i) {
            assert(isfinite(mesh->normals[i].x));
            assert(isfinite(mesh->normals[i].y));
            assert(isfinite(mesh->normals[i].z));
        }
    }

    printf("Vertices (p): %u\n", mesh->positions.size());
    printf("Vertices (t): %u\n", mesh->texcoords.size());
    printf("Vertices (n): %u\n", mesh->normals.size());
    {
        TIME_BLOCK("Build Hierarchy");
        BuildHierarchy(&hierarchy, mesh);
    }

    u32 total_tris = 0;
    Scene scene = InitScene();
    scene.hierarchy = &hierarchy;
    // TODO(bryan):  We need to actually load materials.  In the meantime, give everything a default
    // so that we can just test the loading / mesh tracing.
    scene.default_mat = MakeMaterial(Vector4(0.75f, 0.5f, 0.75f, 1.0f));
    for (u32 i = 0; i < hierarchy.mesh_groups.size(); ++i) {
        MeshGroup * mg = hierarchy.mesh_groups[i];
        SceneObject * obj = (SceneObject *)calloc(1, sizeof(SceneObject));
        obj->mesh_group = mg;
        obj->mesh = mesh;
        obj->type = ObjectType_MeshGroup;
        obj->material = scene.default_mat;
        if (mg) {
            total_tris += mg->idx_positions.size() / 3;
            if (mg->material) obj->material = mg->material;
        }

        scene.objects.push_back(obj);
    }
    printf("Triangles: %u\n", total_tris);

    Render(&cam, &fb, &scene);

    WriteFramebufferImage(&fb, gParams.image_output_filename);

    float scale = 1.0f / (float)(1 << 16);

    s64 total_rays_cast = 0;
    s64 max_rays_cast = 0;

    for (u32 y = 0; y < fb.height; ++y) {
        for (u32 x = 0; x < fb.width; ++x) {
            u32 count = fb.DEBUG_rays_cast[x + y * fb.width];
            total_rays_cast += count;
            max_rays_cast = max(max_rays_cast, count);
        }
    }
    //         float h = (1.0f - (float)(count * scale)) * 240.0f;
    //         Vector4 hsv(h/360.0f, 1.0f, 5.0f, 1.0f);
    //         Vector4 rgb = Color_HSVToRGB(hsv);

    //         u32 offset = (y * fb.width + x) * 4;
    //         Color_Pack(fb.bytes + offset, rgb);
    //     }
    // }

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

    // stbi_write_png("rt_rays_out.png", fb.width, fb.height, 4, fb.bytes, 0);

    return 0;
}