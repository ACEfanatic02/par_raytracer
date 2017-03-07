#include "brt.h"
#include "mathlib.h"
#include "mesh.h"
#include <vector>
#include <cstring>
#include <cctype>

#ifdef _WIN32
#include <direct.h>
#define chdir _chdir

// MSVC does not implement strndup, so we'll do it ourselves:
char * 
strndup(const char * s, size_t n) {
    char * rv = (char *)malloc(n + 1);
    strncpy(rv, s, n);
    rv[n] = '\0';
    return rv;
}
#else
#include <unistd.h>
#endif

#define STB_IMAGE_IMPLEMENTATION
#include "lib/stb_image.h"

static u32 current_line;

inline void
ExpectWhitespace(char * c) {
    if (!isspace(*c)) {
        fprintf(stderr, "Failed to find whitespace where expected on line %d, got '%c'\n", current_line, *c);
        __debugbreak();
    }
}

inline void
Expect(char * c, char expect) {
    if (*c != expect) {
        fprintf(stderr, "Failed to find '%c' where expected on line %d\n", expect, current_line);
        __debugbreak();
    }
}

inline char *
NextToken(char * p) {
    while (*p && isspace(*p) && *p != '\n') {
        p++;
    }
    return p;
}

inline char *
NextLine(char * p) {
    while (*p && *p != '\n') {
        p++;
    }
    if (*p == '\n') {
        p++;
        current_line++;
    }
    return p;
}

inline Vector3
ReadVector3(char * cur) {
    Vector3 result;
    char * end;
    cur = NextToken(cur);
    result.x = strtof(cur, &end);
    ExpectWhitespace(end);
    cur = NextToken(end);
    result.y = strtof(cur, &end);
    ExpectWhitespace(end);
    cur = NextToken(end);
    result.z = strtof(cur, &end);
    ExpectWhitespace(end);

    return result;
}

inline Vector2
ReadVector2(char * cur) {
    Vector2 result;
    char * end;
    cur = NextToken(cur);
    result.x = strtof(cur, &end);
    ExpectWhitespace(end);
    cur = NextToken(end);
    result.y = strtof(cur, &end);
    ExpectWhitespace(end);

    return result;
}

inline float 
ReadFloat(char * cur) {
    char * end;
    cur = NextToken(cur);
    float result = strtof(cur, &end);
    ExpectWhitespace(end);
    return result;
}

inline Vector4
ReadColor(char * cur) {
    Vector3 rgb = ReadVector3(cur);
    return Vector4(rgb.x, rgb.y, rgb.z, 1.0f);
}

inline char *
ReadToken(char * cur) {
    char * start = NextToken(cur);
    cur = start;
    while (*cur && !isspace(*cur)) {
        cur++;
    }
    char * result = NULL;
    u32 length = cur - start;
    if (length) {
        result = strndup(start, length);
    }
    return result;
}

inline char * 
ReadFaceTriple(Mesh * mesh, char * cur, u32 * out_pos, u32 * out_tex, u32 * out_norm) {
    char * end;
    cur = NextToken(cur);
    s32 pos = (s32)strtol(cur, &end, 10);
    Expect(end, '/');
    cur = end + 1;
    s32 tex = (s32)strtol(cur, &end, 10);
    Expect(end, '/');
    cur = end + 1;
    s32 norm = (s32)strtol(cur, &end, 10);
    ExpectWhitespace(end);

    if (pos > 0) {
        *out_pos = (u32)(pos - 1);
    }
    else {
        *out_pos = (u32)((s32)mesh->positions.size() + pos);
    }
    if (tex > 0) {
        *out_tex = (u32)(tex - 1);
    }
    else {
        *out_tex = (u32)((s32)mesh->texcoords.size() + tex);
    }
    if (norm > 0) {
        *out_norm = (u32)(norm - 1);
    }
    else {
        *out_norm = (u32)((s32)mesh->normals.size() + norm);
    }

    return end;
}

inline void
ReadFace(Mesh * mesh, MeshGroup * group, char * cur) {
    // We need to read all the indices into a buffer first so that we can
    // triangulate them.  Doing this later is harder; we want to do this
    // as soon in the pipeline as possible.
    u32 pos[8];
    u32 tex[8];
    u32 norm[8];
    u32 vertex_count = 0;
    cur = NextToken(cur);
    while (*cur && *cur != '\n' && *cur != '\r') {
        assert(vertex_count < array_count(pos));
        cur = ReadFaceTriple(mesh, cur, pos + vertex_count, tex + vertex_count, norm + vertex_count);
        cur = NextToken(cur);
        vertex_count++;
    }

    // Triangulate
    for (u32 i = 1; i < vertex_count - 1; ++i) {
        group->idx_positions.push_back(pos[0]);
        group->idx_positions.push_back(pos[i]);
        group->idx_positions.push_back(pos[i + 1]);

        group->idx_texcoords.push_back(tex[0]);
        group->idx_texcoords.push_back(tex[i]);
        group->idx_texcoords.push_back(tex[i + 1]);

        group->idx_normals.push_back(norm[0]);
        group->idx_normals.push_back(norm[i]);
        group->idx_normals.push_back(norm[i + 1]);
    }
}

static Texture *
LoadTexture(char * filename) {
    s32 x, y, channels;
    u8 * bytes = stbi_load(filename, &x, &y, &channels, 0);
    if (!bytes) {
        printf("Failed to load image [%s] :: %s\n", filename, stbi_failure_reason());
        return NULL;
    }

    printf("Loaded image [%s]:  (x=%d y=%d c=%d)\n", filename, x, y, channels);

    Texture * result = (Texture *)calloc(1, sizeof(Texture));
    result->size_x = x;
    result->size_y = y;
    result->channels = channels;
    result->texels = bytes;
    return result;
}

#define STATIC_STRNCMP(_static, _dynamic) (!strncmp(_static, _dynamic, array_count(_static) - 1))

static char *
ReadEntireFile(char * filename) {
    // Load file into buffer (TODO(bryan):  We'd really like some way of doing
    // this incrementally, if we're going to use the larger models.)
    FILE * fp = fopen(filename, "rb");
    fseek(fp, 0, SEEK_END);
    s64 file_length = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char * bytes = (char *)calloc(1, file_length + 1);
    file_length = fread(bytes, 1, file_length, fp);
    bytes[file_length] = '\0';
    fclose(fp);

    return bytes;
}

static MaterialLibrary * 
ParseMTL(char * filename) {
    printf("MTL library: [%s]\n", filename);
    // HACK(bryan):  HACK HACK HACK HACK HACK
    // If we're parsing multiple files interleaved we *really* need to wrap up
    // the parser state for each of them.
    u32 stored_current_line = current_line;

    char * bytes = ReadEntireFile(filename); 

    current_line = 1;
    MaterialLibrary * lib = new MaterialLibrary;
    Material * material = NULL;
    char * cur = bytes;
    while (*cur) {
        cur = NextToken(cur);
        switch (*cur) {
            case 'n': {
                if (STATIC_STRNCMP("newmtl", cur)) {
                    char * name = ReadToken(cur + 6);
                    printf("Material: [%s]\n", name);
                    if (lib->materials.find(name) != lib->materials.end()) {
                        assert(false);
                    }
                    material = (Material *)calloc(1, sizeof(Material));
                    material->name = name;
                    lib->materials[name] = material;
                }
            } break;

            case 'N': {
                cur++;
                switch (*cur) {
                    case 's': {
                        material->specular_intensity = ReadFloat(cur + 1);
                    } break;
                    case 'i': {
                        material->index_of_refraction = ReadFloat(cur + 1);
                    } break;
                }
            } break;

            case 'd': {
                material->alpha = ReadFloat(cur + 1);
            } break;

            case 'T': {
                // TODO
            } break;

            case 'i': {
                // TODO: illum
            } break;

            case 'K': {
                cur++;
                switch (*cur) {
                    case 'a': {
                        material->ambient_color = ReadColor(cur + 1);
                    } break;
                    case 'd': {
                        material->diffuse_color = ReadColor(cur + 1);
                    } break;
                    case 's': {
                        material->specular_color = ReadColor(cur + 1);
                    } break;
                    case 'e': {
                        material->emissive_color = ReadColor(cur + 1);
                    } break;
                }
            } break;

            case 'm': {
                if (STATIC_STRNCMP("map_", cur)) {
                    cur += 4;
                    if (STATIC_STRNCMP("Ka", cur)) {
                        // ambient texture map
                        material->ambient_texture = LoadTexture(ReadToken(cur + 2));
                    }
                    else if (STATIC_STRNCMP("Kd", cur)) {
                        // diffuse texture map
                        material->diffuse_texture = LoadTexture(ReadToken(cur + 2));
                    }
                    else if (STATIC_STRNCMP("d", cur)) {
                        // alpha mask texture
                        material->alpha_texture = LoadTexture(ReadToken(cur + 1));
                    }
                    else if (STATIC_STRNCMP("bump", cur)) {
                        // bump map
                        material->bump_texture = LoadTexture(ReadToken(cur + 4));
                    }
                }
            } break;
        }
        cur = NextLine(cur);
    }

    current_line = stored_current_line;
    free(bytes);
    return lib;
}

static MeshGroup *
AddMeshGroup(Mesh * mesh, char * name) {
    mesh->groups.resize(mesh->groups.size() + 1);
    MeshGroup * result = &mesh->groups[mesh->groups.size() - 1];
    memset(result, 0, sizeof(*result));
    result->name = name;
    return result;
}

static Mesh *
ParseOBJ(char * working_dir, char * filename, Matrix33 transform) {
    // TODO(bryan):  Harden this function and give the entire parser
    // some proper error handling.
    if (chdir(working_dir)) {
        return NULL;
    }
    char * bytes = ReadEntireFile(filename);

    // Parse file.  Not threadsafe but we don't care here, tbh.
    current_line = 1;
    Mesh * mesh = new Mesh;
    mesh->material_library = NULL;
    MeshGroup * current_group = NULL;
    char * cur = bytes;
    while (*cur) {
        // Skip leading whitespace, if any.
        cur = NextToken(cur);
        switch (*cur) {
            case 'v': {
                cur++;
                switch (*cur) {
                    case ' ': {
                        // Vertex position
                        mesh->positions.push_back(transform * ReadVector3(cur + 1));
                    } break;
                    case 't': {
                        // Vertex texcoord
                        mesh->texcoords.push_back(ReadVector2(cur + 1));
                    } break;
                    case 'n': {
                        // Vertex normal
                        mesh->normals.push_back(transform * ReadVector3(cur + 1));
                    } break;
                }
            } break;

            case 'f': {
                if (!current_group) {
                    fprintf(stderr, "Face declared with no active group, line %d.\n", current_line);
                }
                ReadFace(mesh, current_group, cur + 1);
            } break;

            case 'g': {
                current_group = AddMeshGroup(mesh, ReadToken(cur + 1));
            } break;

            case 'u':
            case 'm': {
                if (STATIC_STRNCMP("usemtl", cur)) {
                    assert(mesh->material_library);
                    if (current_group->material) {
                        // We only support one material per group, so split this group up.
                        current_group = AddMeshGroup(mesh, strdup(current_group->name));
                    }
                    char * material_name = ReadToken(cur + 6);
                    current_group->material = mesh->material_library->materials[material_name];
                }
                else if (STATIC_STRNCMP("mtllib", cur)) {
                    // We only support one material library for the entire mesh.
                    // (I don't think I've seen an OBJ with multiple, so this should be fine?)
                    assert(!mesh->material_library);
                    mesh->material_library = ParseMTL(ReadToken(cur + 6));
                }
            } break;
        }
        // Go to the next line (or NUL).
        cur = NextLine(cur);
    }

    free(bytes);
    return mesh;
}

#undef STATIC_STRNCMP
