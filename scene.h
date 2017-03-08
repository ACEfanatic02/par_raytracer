#pragma once

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

struct BoundingHierarchy;
struct Scene {
    std::vector<SceneObject *> objects;
    BoundingHierarchy * hierarchy;
    LightSource * lights;
    u32 light_count;
    Material * default_mat;
};
