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

    Vector4 ambient_color;
    Vector4 diffuse_color;
    Vector4 specular_color;
    Vector4 emissive_color;

    Texture * ambient_texture;
    Texture * diffuse_texture;
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

    MaterialLibrary * material_library;
};

