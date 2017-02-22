#include "brt.h"
#include "mathlib.h"
#include <vector>
#include <cctype>

typedef std::vector<u32> IndexBuffer;

struct MeshGroup {
    IndexBuffer idx_positions;
    IndexBuffer idx_texcoords;
    IndexBuffer idx_normals;

    char * name;
    char * material_name;
};

struct Mesh {
    std::vector<MeshGroup> groups;

    std::vector<Vector3> positions;
    std::vector<Vector2> texcoords;
    std::vector<Vector3> normals;
};

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
        result = (char *)calloc(length + 1, 1);
        memcpy(result, start, length);
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

#define STATIC_STRNCMP(_static, _dynamic) (!strncmp(_static, _dynamic, array_count(_static) - 1))

static Mesh *
ParseOBJ(char * filename) {
    // Load file into buffer (TODO(bryan):  We'd really like some way of doing
    // this incrementally, if we're going to use the larger models.)
    FILE * fp = fopen(filename, "rb");
    fseek(fp, 0, SEEK_END);
    s64 file_length = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    char * bytes = (char *)calloc(1, file_length + 1);
    file_length = fread(bytes, 1, file_length, fp);
    bytes[file_length] = '\0';

    // Parse file.  Not threadsafe but we don't care here, tbh.
    current_line = 1;
    Mesh * mesh = (Mesh *)calloc(1, sizeof(Mesh));
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
                        mesh->positions.push_back(ReadVector3(cur + 1));
                    } break;
                    case 't': {
                        // Vertex texcoord
                        mesh->texcoords.push_back(ReadVector2(cur + 1));
                    } break;
                    case 'n': {
                        // Vertex normal
                        mesh->normals.push_back(ReadVector3(cur + 1));
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
                mesh->groups.resize(mesh->groups.size() + 1);
                current_group = &mesh->groups[mesh->groups.size() - 1];
                current_group->name = ReadToken(cur + 1);
            } break;

            case 'u': {
                if (STATIC_STRNCMP("usemtl", cur)) {
                    current_group->material_name = ReadToken(cur + 6);
                }
            } break;
        }
        // Go to the next line (or NUL).
        cur = NextLine(cur);
    }

//    __debugbreak();

    free(bytes);
    return mesh;
}

#undef STATIC_STRNCMP
