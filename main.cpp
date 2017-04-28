#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "lib/stb_image_write.h"

#include <mpi.h>

#include "raytracer.cpp"

// "True" random bytes for seeds.
u64 gRNGInitTable[96] = {
    0x835fdd9143716fe3, 0x766b5859cff0b8af, 
    0xa6e413e7db131ae8, 0xf937ae7b5dfa5f9e, 
    0xca1da92e487e5dda, 0x4572044b46751956, 
    0x3ecc4cd8b7da619c, 0xc9c946cf5b554bfe, 
    0xd88bf2e733a7724e, 0xa2a3df20cbb69d57, 
    0x405a331e428599d6, 0xaa596a37b752e07f, 
    0xd662bedca87d669c, 0x76d3f659517d381a, 
    0xcdede6fa437952b0, 0xcde0101320a890a9, 
    0x3190fdfb6f56d995, 0x6ccac23f5cedb255, 
    0x03f70862bfa504a3, 0x711506ac8688dd62, 
    0x68c4d831d507c08d, 0x1826f68dbfd4bd2d, 
    0xf594d21a7bbc7f1b, 0x20a1aba419d35a11, 
    0x0c34f20b15e98e44, 0xe84ca37db4fd90cd, 
    0x02e6b82454860f8c, 0x0083f64f18ae2a66, 
    0xaea8f6929ae5c72a, 0xa1c26f00a1f61b5d, 
    0x15055adabf3728c7, 0x243bc00f89fadf77, 
    0x3018bb7a88d763fa, 0xf48398b9b8408198, 
    0xfbe5422ce184ce04, 0xdb74b5a8958da74b, 
    0xa33992eec0b0a00b, 0xbc76994066f79b5f, 
    0xdee7e04ffcb817f5, 0xc9d4b45587ca3f40, 
    0xf1a3d45f612df83b, 0xd54a32df4fa418dc, 
    0x187107e77b3a55d0, 0x5a81cfea84fe69d9, 
    0xe10b82423cb2f841, 0x23ce39e24eccfccb, 
    0x9e2390f72f8ddc53, 0x4b172f7c1701b787, 
    0x4b22b493189f49c1, 0x57c995ef7e96cd1c, 
    0xf5e792a7112361d8, 0x36042fe2a4590b09, 
    0x47fa9894e9128c04, 0xa1f8ab6f95b3270c, 
    0xe7a2e57345e8b9eb, 0x8e5ed76c080b667a, 
    0x455459799a3af869, 0x35d2adba6c69d966, 
    0x582242f263c40f8b, 0xffd827f43c11f58d, 
    0x762dc0459e350655, 0xf2e589aa370ca22f, 
    0x0ef225880283464f, 0x3ac446246d6c0810, 
    0x9397032ba5e8d2f2, 0x1fb8e9d372aee480, 
    0xdb13349a665cfc7e, 0x1004319e83016bf9, 
    0x64bbf5ea8a5b64f2, 0xbcfd4254df754a14, 
    0xe0ce0a93bcbf5d27, 0x529369f715b36806, 
    0x47893798eacd302a, 0x43ef1854d3e4c8ce, 
    0xdbf975a99f9450ca, 0x0a709e78f5e65d20, 
    0x98e236f202644dde, 0xa5883cff1a026c6f, 
    0xb9bb81b1d7d93048, 0x6b7d779911baa9cc, 
    0xf9db1b72ebc5470d, 0x7e5a96e8a322f03b, 
    0x78b7c9ff8ad99ec6, 0x4d110f542e72ad71, 
    0xb3e2060505d633dc, 0xdc8eda857aa8e780, 
    0xca598eb3ead7ecfc, 0x849a9d886fb2b2d3, 
    0xa496d8dc0ff0d0da, 0xf8133ad72001ec56, 
    0x1a0a073fabf62c12, 0xd7034304d881d6e3, 
    0x92c2baa0d06ea9be, 0xc5dc91e742f1d52e, 
    0x42fe9ba237b6887e, 0xff8eef282d88e8e9, 
};

static RandomState
GetRNG(u32 process_id, u32 thread_id) {
    u64 seed_idx = (process_id << 3 | thread_id) % array_count(gRNGInitTable);

    RandomState result;
    Random_Seed(&result, gRNGInitTable[seed_idx]);
    return result;
}

struct Framebuffer {
    Vector4 * pixels;
    u32 width;
    u32 height;
};

static s32 gMPI_CommSize;
static s32 gMPI_CommRank;

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
MakeCamera(float fov, u32 width, u32 height) {
    Camera cam;
    cam.tan_a2 = tanf(DEG2RAD(fov / 2.0f));
    cam.aspect = (float)width / (float)height;
    cam.inv_width = 1.0f / (float)width;
    cam.inv_height = 1.0f / (float)height;
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

struct RenderSharedData {
    Camera * cam;
    Scene * scene;
    u32 width;
    u32 height;
    u32 min_samples;
    u32 max_samples;
};

struct RenderJob {
    RenderSharedData * shared;
    u32 start_idx;
    u32 end_idx;
    Vector4 * buffer;

    RandomState rng;
};

static float
CalculateVariance(Vector4 * vals, u32 count) {
    Vector4 mean;
    for (u32 i = 0; i < count; ++i) {
        mean += vals[i];
    }
    mean /= count;

    float variance = 0.0f;
    for (u32 i = 0; i < count; ++i) {
        float d = Color_Distance(vals[i], mean);
        variance += d * d;
    }
    variance /= (count - 1);

    return variance;
}

static Vector4
RenderPixel(RenderJob * job, DebugCounters * debug, u32 x, u32 y) {
    Camera * cam      = job->shared->cam;
    Scene * scene     = job->shared->scene;
    u32 min_samples   = job->shared->min_samples;
    u32 max_samples   = job->shared->max_samples;
    RandomState * rng = &job->rng;

    Vector4 * scratch_buffer = (Vector4 *)calloc(sizeof(Vector4), max_samples);

    Vector2 base_position(x, y);
    Vector4 color;
    u32 samp = 0;
    for (; samp < min_samples; ++samp) {
        Vector2 sample_offset(Random_NextFloat11(rng), Random_NextFloat11(rng));

        Ray ray = MakeCameraRay(cam, base_position + sample_offset * 0.5f);
        scratch_buffer[samp] = TraceRayColor(ray, scene, gParams.bounce_depth, debug, rng);
        color += scratch_buffer[samp];
    }

    float var = CalculateVariance(scratch_buffer, samp);
    for (; samp < max_samples; ++samp) {
        Vector2 sample_offset(Random_NextFloat11(rng), Random_NextFloat11(rng));

        Ray ray = MakeCameraRay(cam, base_position + sample_offset);
        scratch_buffer[samp] = TraceRayColor(ray, scene, gParams.bounce_depth, debug, rng);
        color += scratch_buffer[samp];

        var = CalculateVariance(scratch_buffer, samp);
        static const float variance_threshold = 0.01f; 
        if (var <= variance_threshold) {
            break;
        }
    }

    free(scratch_buffer);

    color /= samp;
    color.w = 1.0f;
    return color;
}

static void
RenderTask(RenderJob * job, DebugCounters * debug) {
    u32 w = job->shared->width;
    u32 h = job->shared->height;
    Assert(w > 0 && h > 0, "Must have positive size.");

    for (u32 i = job->start_idx; i < job->end_idx; ++i) {
        u32 x = i % w;
        u32 y = i / w;
        if (y >= h) {
            break;
        }

        u32 buffer_idx = i - job->start_idx;
        job->buffer[buffer_idx] = RenderPixel(job, debug, x, y);
    }
}

std::vector<RenderJob> task_list;
u32 next_task;

static void
RenderQueueWorker(DebugCounters * debug) {
    while (next_task < task_list.size()) {
        u32 task_id = next_task++;

        RenderJob * j = &task_list[task_id];
        {
            // TIME_BLOCK("Render Row");
            RenderTask(j, debug);
        }
    }
}

static Framebuffer
Render(Camera * cam, Scene * scene, u32 width, u32 height) {
    RenderSharedData shared;
    shared.cam = cam;
    shared.scene = scene;
    shared.width = width;
    shared.height = height;
    shared.min_samples = 10;
    shared.max_samples = 50;
   
    u32 total_pixel_count = width * height;
    Assert(total_pixel_count % gMPI_CommSize == 0, "Pixel count must be a multiple of process count.");
    u32 count_per_proc = (total_pixel_count + gMPI_CommSize - 1) / gMPI_CommSize;
    RenderJob job;
    job.shared = &shared;
    job.start_idx = count_per_proc * gMPI_CommRank;
    job.end_idx = count_per_proc * (gMPI_CommRank + 1);
    job.buffer = (Vector4 *)calloc(sizeof(Vector4), count_per_proc);
    job.rng = GetRNG(gMPI_CommRank, 0);
    // TODO(bryan):  Need to allocate jobs across local threads as well.

    task_list.push_back(job);

    DebugCounters debug = {};
    {
        MPI_Barrier(MPI_COMM_WORLD);
        TIME_BLOCK("Render, sync");
        {
            TIME_BLOCK("Render, async");
            RenderQueueWorker(&debug);
        }
        MPI_Barrier(MPI_COMM_WORLD);
    }

    Framebuffer result;
    result.width  = width;
    result.height = height;
    if (gMPI_CommRank == 0) {
        result.pixels = (Vector4 *)calloc(sizeof(Vector4), total_pixel_count);
    }

    {
        TIME_BLOCK("Reduce");
        // MPI Reduction
        MPI_Gather(job.buffer,    count_per_proc*4, MPI_FLOAT,
                   result.pixels, count_per_proc*4, MPI_FLOAT,
                   0, MPI_COMM_WORLD);
        MPI_Barrier(MPI_COMM_WORLD);
    }

    printf("Process %d\n", gMPI_CommRank);
    printf("Rays cast:          %llu\n", debug.ray_count);
    printf("Spheres checked:    %llu\n", debug.sphere_check_count);
    printf("   average / pixel: %f\n", (double)debug.sphere_check_count / total_pixel_count);
    printf("Meshes checked:     %llu\n", debug.mesh_check_count);
    printf("   average / pixel: %f\n", (double)debug.mesh_check_count / total_pixel_count);
    return result;
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
    gParams.background_color = Vector4(0.8275f, 0.8913f, 1.0f, 1.0f)*1.5f; // Sky color, serves as an area light if all rays miss.
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

    Camera cam = MakeCamera(gParams.camera_fov, gParams.image_width, gParams.image_height);
    Mesh * mesh;
    BoundingHierarchy hierarchy;
    Matrix33 transform;
    transform.SetIdentity();
    {
        // TIME_BLOCK("Load Mesh");
        mesh = ParseOBJ(gParams.data_dirname, "sponza.obj", transform);
    }
    {
        // TIME_BLOCK("Calculate Tangents");
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
        // TIME_BLOCK("Build Hierarchy");
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

    Framebuffer fb = Render(&cam, &scene, gParams.image_width, gParams.image_height);

    WriteFramebufferImage(&fb, gParams.image_output_filename);

    fprintf(stderr, "Process %d done, exiting...\n", gMPI_CommRank);

    MPI_Barrier(MPI_COMM_WORLD);
    MPI_Finalize();

    return 0;
}
