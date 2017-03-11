#pragma once
#include <vector>
#include <unordered_map>

#include "brt.h"
#include "mathlib.h"

struct Texture {
    u32 size_x;
    u32 size_y;
    u32 channels;
    u8 * texels;
};

struct Material {
    float specular_intensity;
    float index_of_refraction;
    float alpha;
    char * name;

    Vector4 ambient_color;
    Vector4 diffuse_color;
    Vector4 specular_color;
    Vector4 emissive_color;

    Texture * ambient_texture;
    Texture * diffuse_texture;
    Texture * specular_texture;
    Texture * alpha_texture;
    Texture * bump_texture;
};

struct MaterialLibrary {
    std::unordered_map<std::string, Material *> materials;
};

typedef std::vector<u32> IndexBuffer;

struct MeshGroup {
    IndexBuffer idx_positions;
    IndexBuffer idx_texcoords;
    IndexBuffer idx_normals;

    char * name;
    Material * material;
};

struct Mesh {
    std::vector<MeshGroup> groups;

    std::vector<Vector3> positions;
    std::vector<Vector2> texcoords;
    std::vector<Vector3> normals;
    std::vector<Vector3> tangents;

    MaterialLibrary * material_library;
};

void
CalculateTangents(Mesh * mesh) {
    mesh->tangents.resize(mesh->normals.size());

    for (u32 mg_idx = 0; mg_idx < mesh->groups.size(); ++mg_idx) {
        MeshGroup * mg = &mesh->groups[mg_idx];
        assert(mg->idx_positions.size() == mg->idx_texcoords.size());
        assert(mg->idx_positions.size() == mg->idx_normals.size());
        assert(mg->idx_positions.size() % 3 == 0);

        if (!mg->material || (!mg->material->bump_texture)) {
            // We're not going to be doing normal mapping on these vertices,
            // so don't bother computing tangents for them!
            continue;
        }

        // Iterate through all triangles
        for (u32 vert_i = 0; vert_i < mg->idx_positions.size(); vert_i += 3) {
            Vector3 p0 = mesh->positions[mg->idx_positions[vert_i + 0]];
            Vector3 p1 = mesh->positions[mg->idx_positions[vert_i + 1]];
            Vector3 p2 = mesh->positions[mg->idx_positions[vert_i + 2]];

            // We use UV deltas to compute our tangent vectors so that they are
            // continuous between triangles.
            Vector2 uv0 = mesh->texcoords[mg->idx_texcoords[vert_i + 0]];
            Vector2 uv1 = mesh->texcoords[mg->idx_texcoords[vert_i + 1]];
            Vector2 uv2 = mesh->texcoords[mg->idx_texcoords[vert_i + 2]];

            Vector3 dp0 = p1 - p0;
            Vector3 dp1 = p2 - p0;

            Vector2 duv0 = uv1 - uv0;
            Vector2 duv1 = uv2 - uv0;

            float f = (duv0.x * duv1.y - duv1.x * duv0.y);
            if (f <= 1e-7) {
                // Degenerate texture coordinates.  This is rare, but *does* 
                // happen on the Crytek Sponza model, and will break things,
                // so (for now) we punt.
                continue;
            }
            f = 1.0f / f;

            Vector3 tangent;
            tangent.x = f * (duv1.y * dp0.x - duv0.y * dp1.x);
            tangent.y = f * (duv1.y * dp0.y - duv0.y * dp1.y);
            tangent.z = f * (duv1.y * dp0.z - duv0.y * dp1.z);
            if (!isfinite(tangent.x) ||
                !isfinite(tangent.y) ||
                !isfinite(tangent.z)) {
                int breakhere = 1;
            }

            // We don't normalize the tangents immediately.  Because our normals
            // are reused between vertices, we want to blend smoothly.  By normalizing
            // the *sum*, we weight larger triangles more heavily.
            mesh->tangents[mg->idx_normals[vert_i + 0]] += tangent;
            mesh->tangents[mg->idx_normals[vert_i + 1]] += tangent;
            mesh->tangents[mg->idx_normals[vert_i + 2]] += tangent;
        }
    }

    // Now normalize all of these.  
    for (u32 i = 0; i < mesh->tangents.size(); ++i) {
        if (!isfinite(mesh->tangents[i].x) ||
            !isfinite(mesh->tangents[i].y) ||
            !isfinite(mesh->tangents[i].z)) {
            int breakhere = 1;
        }
        mesh->tangents[i] = Normalize(mesh->tangents[i]);
    }
}