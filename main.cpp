#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "lib/stb_image_write.h"

#include <mpi.h>

#include "raytracer.cpp"

struct Framebuffer {
    Vector4 * pixels;
    u32 width;
    u32 height;
    u32 * DEBUG_rays_cast;
};

static s32 gMPI_CommSize;
static s32 gMPI_CommRank;

static void
ReduceImageMPI(Framebuffer * fb) {
    u32 total_pixel_count = fb->width * fb->height;
    Vector4 * recv_buffer = (Vector4 *)calloc(total_pixel_count, sizeof(Vector4));

    // Each rank processes individual rows in a cycle, so we need multiple gathers to
    // collect all the data.
    //
    // We use an Allgather so that each rank ends up with the complete framebuffer; this
    // will be used to calculate pixel variance between passes.

    u32 cycle_count = (fb->height + gMPI_CommSize - 1) / gMPI_CommSize;
    for (u32 cycle = 0; cycle < cycle_count; ++cycle) {
        u32 row = cycle * gMPI_CommSize;
        // Final cycle may not be complete.
        // e.g, 5 rows over 4 processes.
        if (row + gMPI_CommRank < fb->height) {
            Vector4 * recv_pointer = recv_buffer + (fb->width * row);
            Vector4 * send_pointer = fb->pixels + (fb->width * (row + gMPI_CommRank));
            MPI_Allgather(send_pointer, fb->width * 4, MPI_FLOAT,
                          recv_pointer, fb->width * 4, MPI_FLOAT,
                          MPI_COMM_WORLD);
        }
    }

    MPI_Barrier(MPI_COMM_WORLD);
    free(fb->pixels);
    fb->pixels = recv_buffer;
}

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
    if (gMPI_CommRank != 0) {
        return;
    }

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
         + fabsf(a.z - b.z);
         //+ fabsf(a.w - b.w);
}

static float
CalculateLocalVariance(Framebuffer *fb, u32 x, u32 y) {
    u32 min_x = Min(x - 2, 0);
    u32 max_x = Min(x + 2, fb->width);
    u32 min_y = Min(y - 2, 0);
    u32 max_y = Min(y + 2, fb->height);

    float sum = 0.0f;
    u32 count = (max_x - min_x) * (max_y - min_y);
    Vector4 mean;
    for (u32 yy = min_y; yy <= max_y; ++yy) {
        for (u32 xx = min_x; xx <= max_x; ++xx) {
            mean += fb->pixels[yy*fb->width + xx];
        }
    }
    mean /= (float)count;

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

//#include <thread>
//#include <atomic>

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
RenderTask(RenderJob * job, DebugCounters * debug) {
    for (u32 x = 0; x < job->fb->width; ++x) {
        Vector4 color;
        for (u32 s = 0; s < sample_pattern_count; ++s) {
            Vector2 pos = Vector2(x, job->y) + sample_patterns[s];
            Ray ray = MakeCameraRay(job->cam, pos);
            color += TraceRayColor(ray, job->scene, gParams.bounce_depth, debug);
            job->sample_counts[x + job->y * job->fb->width]++;
        }
        color /= sample_pattern_count;
        WriteColor(job->fb, x, job->y, color.x, color.y, color.z, 1.0f);
    }
}

std::vector<RenderJob> task_list;
//std::atomic<u32> next_task;
u32 next_task;

static void
RenderQueueWorker(DebugCounters * debug) {
    while (next_task < task_list.size()) {
        u32 task_id = next_task++;

        RenderJob * j = &task_list[task_id];
        {
            TIME_BLOCK("Render Row");
            RenderTask(j, debug);
        }
    }
}

static void
Render(Camera * cam, Framebuffer * fb, Scene * scene) {
    u32 * sample_counts = (u32 *)calloc(fb->width * fb->height, sizeof(u32));

    sample_pattern_count = 16;
    sample_patterns = (Vector2 *)calloc(sample_pattern_count, sizeof(Vector2));
    for (u32 i = 0; i < sample_pattern_count; ++i) {
        sample_patterns[i].x = GetRandFloat11() * 0.5f;
        sample_patterns[i].y = GetRandFloat11() * 0.5f;
    }

    for (u32 y = 0; y < fb->height; ++y) {
        if (y % gMPI_CommSize == gMPI_CommRank) {
            RenderJob job;
            job.cam = cam;
            job.scene = scene;
            job.fb = fb;
            job.sample_counts = sample_counts;
            job.y = y;

            task_list.push_back(job);
        }
    }

    DebugCounters debug = {};
    {
        MPI_Barrier(MPI_COMM_WORLD);
        TIME_BLOCK("Render, Initial Pass");
        //u32 thread_count = std::thread::hardware_concurrency() - 1;
        //DebugCounters * debug_counters = (DebugCounters *)calloc(thread_count, sizeof(DebugCounters));
        //std::vector<std::thread> threads;
        //for (u32 i = 0; i < thread_count; ++i) {
        //    threads.push_back(std::thread(RenderQueueWorker, debug_counters + i));
        //}
        // printf("%u render worker threads started.\n", thread_count);
        RenderQueueWorker(&debug);

        //for (u32 i = 0; i < thread_count; ++i) {
        //    threads[i].join();
        //    debug.ray_count += debug_counters[i].ray_count;
        //    debug.sphere_check_count += debug_counters[i].sphere_check_count;
        //    debug.mesh_check_count += debug_counters[i].mesh_check_count;
        //}
        MPI_Barrier(MPI_COMM_WORLD);
    }

    ReduceImageMPI(fb);
    WriteFramebufferImage(fb, "rt_out_beforevar.png");

    printf("Start variance reduction\n");
    u32 max_variance_reduction_passes = 2;

    float min_var = FLT_MAX;
    float max_var = -FLT_MAX;
    float * var_map = (float *)calloc(fb->width * fb->height, sizeof(float));
    for (u32 pass = 0; pass < max_variance_reduction_passes; ++pass) {
        const float variance_threshold = 1.0f;
        {
            TIME_BLOCK("Variance Map");
            MakeVarianceMap(fb, var_map);
        }
        for (u32 y = 0; y < fb->height; ++y) {
            if (y % gMPI_CommSize != gMPI_CommRank) {
                continue;  // HACK, this entire loop needs serious cleanup.
            }
            TIME_BLOCK("Render Row");
            u32 skip_count = 0;
            for (u32 x = 0; x < fb->width; ++x) {
                u32 idx = y * fb->width + x;

                min_var = Min(min_var, var_map[idx]);
                max_var = Max(max_var, var_map[idx]);

                if (var_map[idx] < variance_threshold) {
                    skip_count++;
                    continue;
                }

                Vector4 color = fb->pixels[idx];

                for (u32 i = 0; i < sample_pattern_count; ++i) {
                    float off_x = GetRandFloat11() * 0.5f;
                    float off_y = GetRandFloat11() * 0.5f;
                    Vector2 pos = Vector2(x + off_x, y + off_y);
                    Ray ray = MakeCameraRay(cam, pos);
                    u32 sample_count = sample_counts[idx]++;
                    Vector4 sample_color = TraceRayColor(ray, scene, gParams.bounce_depth, &debug);

                    // Numerically robust incremental average.
                    // http://realtimecollisiondetection.net/blog/?p=48
                    color = color * ((float)sample_count / (sample_count + 1)) 
                          + sample_color / (float)sample_count;
                }
                
                WriteColor(fb, x, y, color.x, color.y, color.z, 1.0f);
            }
            // printf("Skipped %d pixels\n", skip_count);
        }
        // printf("Variance min: %f  max: %f\n", min_var, max_var);

        char buffer[256];
        snprintf(buffer, sizeof(buffer), "rt_out_pass%02d.png", pass + 1);
        WriteFramebufferImage(fb, buffer);
    }
    free(var_map);

    u32 pixel_count = fb->height*fb->width;
    printf("Process %d\n", gMPI_CommRank);
    printf("Rays cast:          %llu\n", debug.ray_count);
    printf("Spheres checked:    %llu\n", debug.sphere_check_count);
    printf("   average / pixel: %f\n", (double)debug.sphere_check_count / pixel_count);
    printf("Meshes checked:     %llu\n", debug.mesh_check_count);
    printf("   average / pixel: %f\n", (double)debug.mesh_check_count / pixel_count);
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
    gParams.background_color = Vector4(0.1275f, 0.8913f, 1.0f, 1.0f)*1.5f; // Sky color, serves as an area light if all rays miss.
    gParams.camera_fov = 60.0f;
    // gParams.camera_position = Vector3(0.0f, 15.0f, 10.0f);
    // gParams.camera_facing = Normalize(Vector3(0.0f, -0.5f, -1));
    gParams.camera_position = Vector3(475.0f, 250.0f, 0.0f);
    gParams.camera_facing = Normalize(Vector3(1.25f, -0.5f, 1.25f));
    gParams.image_output_filename = strdup("rt_out.png"); // Need this to be on the heap; might be free'd later.
    gParams.data_dirname = strdup("/scratch/taylorbr/data/crytek-sponza/");
    gParams.image_width = 720;
    gParams.image_height = 480;

    // TODO(bryan):  INI

    // Parse argv
    for (int arg_i = 1; arg_i < argc; ++arg_i) {
        char * arg = argv[arg_i];
#define STATIC_STRNCMP(_static, _dynamic) (!strncmp(_static, _dynamic, array_count(_static) - 1))
        if (STATIC_STRNCMP("--ray-bias", arg)) {
            Argv_ReadFloat(argc, argv, arg_i, &gParams.ray_bias);
            arg_i++;
        }
        else if (STATIC_STRNCMP("--width", arg) || STATIC_STRNCMP("-w", arg)) {
            Argv_ReadU32(argc, argv, arg_i, &gParams.image_width);
            arg_i++;        
        }
        else if (STATIC_STRNCMP("--height", arg) || STATIC_STRNCMP("-h", arg)) {
            Argv_ReadU32(argc, argv, arg_i, &gParams.image_height);
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
        else if (STATIC_STRNCMP("--data", arg) || STATIC_STRNCMP("-d", arg)) {
            if (gParams.data_dirname) {
                free(gParams.data_dirname);
            }
            gParams.data_dirname = strdup(argv[arg_i + 1]);
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
    lights[0].facing = Normalize(Vector3(1.0f, -1.5f, 0.25f));

    lights[1].type = Light_Directional;
    lights[1].color = Vector4(1.0f, 1.0f, 1.0f, 1.0f) * 1.0f;
    lights[1].facing = Normalize(Vector3(0.0f, -1.0f, 0.0f));

    Scene scene;
    scene.lights = lights;
    scene.light_count = 1;

    return scene;
}

int main(int argc, char ** argv) {
    fprintf(stderr, "MPI_Init\n");
    MPI_Init(&argc, &argv);
    fprintf(stderr, "Getting MPI params.\n");
    MPI_Comm_size(MPI_COMM_WORLD, &gMPI_CommSize);
    MPI_Comm_rank(MPI_COMM_WORLD, &gMPI_CommRank);

    InitParams(argc, argv);

    Framebuffer fb;
    fb.width = gParams.image_width;
    fb.height = gParams.image_height;
    fb.pixels = (Vector4 *)calloc(fb.width * fb.height, sizeof(Vector4));
    fb.DEBUG_rays_cast = (u32 *)calloc(fb.width*fb.height, sizeof(u32));

    Camera cam = MakeCamera(gParams.camera_fov, &fb);
    Mesh * mesh;
    BoundingHierarchy hierarchy;
    Matrix33 transform;
    transform.SetIdentity();
    {
        TIME_BLOCK("Load Mesh");
        mesh = ParseOBJ(gParams.data_dirname, "sponza.obj", transform);
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

    // printf("Vertices (p): %u\n", mesh->positions.size());
    // printf("Vertices (t): %u\n", mesh->texcoords.size());
    // printf("Vertices (n): %u\n", mesh->normals.size());
    {
        TIME_BLOCK("Build Hierarchy");
        BuildHierarchy(&hierarchy, mesh);
    }

    u32 total_tris = 0;
    Scene scene = InitScene();
    scene.hierarchy = &hierarchy;
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
            // printf("MeshGroup '%s', material '%s' position: { %f, %f, %f }\n", 
            //         mg->name,
            //         mg->material ? mg->material->name : "None",
            //         hierarchy.spheres[i].s.center.x,
            //         hierarchy.spheres[i].s.center.y,
            //         hierarchy.spheres[i].s.center.z);
        }

        scene.objects.push_back(obj);
    }
    printf("Triangles: %u\n", total_tris);

    Render(&cam, &fb, &scene);

    WriteFramebufferImage(&fb, gParams.image_output_filename);

    fprintf(stderr, "Process %d done, exiting...\n", gMPI_CommRank);

    MPI_Barrier(MPI_COMM_WORLD);
    MPI_Finalize();

    return 0;
}
