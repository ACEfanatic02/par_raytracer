#pragma once
#include <vector>

#include "brt.h"
#include "mathlib.h"

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
