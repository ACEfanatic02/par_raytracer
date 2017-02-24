#include "brt.h"
#include "mathlib.h"
#include "mesh.h"
#include "geometry.h"
#include "timing.h"

struct BoundingSphereP {
    Sphere s;
    BoundingSphereP * children[2];
    MeshGroup * mesh_group;
};
#if 0
static Matrix33
CovarianceMatrix(Vector3 * points, u32 point_count) {
    float one_over_count = 1.0f / (float)point_count;
    Vector3 c;
    float e00 = 0.0f;
    float e11 = 0.0f;
    float e22 = 0.0f;
    float e01 = 0.0f;
    float e02 = 0.0f;
    float e12 = 0.0f;

    for (u32 i = 0; i < point_count; ++i) {
        c += points[i];
    }
    c *= one_over_count;

    for (u32 i = 0; i < point_count; ++i) {
        Vector3 p = points[i] - c;

        e00 += p.x*p.x;
        e11 += p.y*p.y;
        e22 += p.z*p.z;
        e01 += p.x*p.y;
        e02 += p.x*p.z;
        e12 += p.y*p.z;
    }
    
    Matrix33 m;
    m(0, 0) = e00 * one_over_count;
    m(1, 1) = e11 * one_over_count;
    m(2, 2) = e22 * one_over_count;
    m(0, 1) = m(1, 0) = e01 * one_over_count;
    m(0, 2) = m(2, 0) = e02 * one_over_count;
    m(1, 2) = m(1, 2) = e12 * one_over_count;

    return m;
}

static void
SymSchur2(Matrix33 m, u32 p, u32 q, float * out_c, float * out_s) {
    const float epsilon = 0.0001f;
    if (fabsf(m(p, q)) > epsilon) {
        float r = (m(q, q) - m(p, p)) / (2.0f * m(p, q));
        float t;
        if (r >= 0.0f) {
            t = 1.0f / (r + sqrtf(1.0f + r*r));
        }
        else {
            t = -1.0f / (-r + sqrtf(1.0f + r*r));
        }
        *out_c = 1.0f / sqrtf(1.0f + t*t);
        *out_s = (*out_c) * t;
    }
    else {
        *out_c = 1.0f;
        *out_s = 0.0f;
    }
}

static BoundingSphere
BoundingSphere_FromTriangle(Vector3 p0, Vector3 p1, Vector3 p2) {
    BoundingSphere result;
    result.center = (p0 + p1 + p2) / 3.0f;
    float d0 = Dot(result.center, p0);
    float d1 = Dot(result.center, p1);
    float d2 = Dot(result.center, p2);

    float dmax = max(max(d0, d1), d2);
    result.radius = sqrtf(dmax);
    return result;
}
#endif

static void
UpdateSphereWithPoint(Sphere * s, Vector3 p) {
    Vector3 pc = p - s->center;
    float sq_dist = Dot(pc, pc);
    if (sq_dist > (s->radius * s->radius)) {
        float dist = sqrtf(sq_dist);
        float new_radius = (s->radius + dist) * 0.5f + 1e-2;
        float k = (new_radius - s->radius) / dist;

        s->radius = new_radius;
        s->center += pc * k;
    }
}

static Sphere
BoundingSphere_FromMesh(Mesh * mesh, MeshGroup * group) {
    // To estimate the bounding sphere, we start by finding the min/max points
    // along all axes, and then use the most separated of those to form a 
    // sphere.  We then apply Ritter's algorithm, looping over all points and
    // expanding the sphere to fit any that are not included.
    //
    // This is not optimal, but should be Good Enough?
    //
    // Alternatives:
    // - Eigen-Ritter -- more accurate, more complex.
    // - Welzl -- optimal, difficult to make robust.
    u32 point_count = group->idx_positions.size();

    u32 first_idx = group->idx_positions[0];

    u32 min_x_idx = first_idx;
    u32 min_y_idx = first_idx;
    u32 min_z_idx = first_idx;
    u32 max_x_idx = first_idx;
    u32 max_y_idx = first_idx;
    u32 max_z_idx = first_idx;
    for (u32 i = 1; i < point_count; ++i) {
        u32 idx = group->idx_positions[i];
        if (mesh->positions[idx].x < mesh->positions[min_x_idx].x) min_x_idx = idx;
        if (mesh->positions[idx].y < mesh->positions[min_y_idx].y) min_y_idx = idx;
        if (mesh->positions[idx].z < mesh->positions[min_z_idx].z) min_z_idx = idx;

        if (mesh->positions[idx].x > mesh->positions[max_x_idx].x) max_x_idx = idx;
        if (mesh->positions[idx].y > mesh->positions[max_y_idx].y) max_y_idx = idx;
        if (mesh->positions[idx].z > mesh->positions[max_z_idx].z) max_z_idx = idx;
    }

    Vector3 vx = mesh->positions[min_x_idx] - mesh->positions[max_x_idx];
    Vector3 vy = mesh->positions[min_y_idx] - mesh->positions[max_y_idx];
    Vector3 vz = mesh->positions[min_z_idx] - mesh->positions[max_z_idx];

    float sq_dist_x = Dot(vx, vx);
    float sq_dist_y = Dot(vy, vy);
    float sq_dist_z = Dot(vz, vz);

    // Pick the two most separated points.
    u32 min_idx = min_x_idx;
    u32 max_idx = max_x_idx;
    if (sq_dist_y > sq_dist_x && sq_dist_y > sq_dist_z) {
        max_idx = max_y_idx;
        min_idx = min_y_idx; 
    }
    if (sq_dist_z > sq_dist_x && sq_dist_z > sq_dist_y) {
        max_idx = max_z_idx;
        min_idx = min_z_idx;
    }

    Sphere result;
    result.center = (mesh->positions[min_idx] + mesh->positions[max_idx]) * 0.5f;
    result.radius = Length(mesh->positions[max_idx] - result.center) * 0.95f;

    for (u32 i = 0; i < point_count; ++i) {
        u32 idx = group->idx_positions[i];
        Vector3 v = mesh->positions[idx];
        UpdateSphereWithPoint(&result, v);

        assert(Length(v - result.center) <= result.radius);
    }
    // printf("[GROUP = %s] (%f %f %f %f)\n", group->name, result.center.x, result.center.y, result.center.z, result.radius);

    return result;
}

static Sphere
BoundingSphere_FromChildren(Sphere s0, Sphere s1) {
    Sphere result;
    Vector3 v = s1.center - s0.center;
    float sq_dist = Dot(v, v);

    float diff_r = s1.radius - s0.radius;
    if ((diff_r*diff_r) >= sq_dist) {
        if (s1.radius >= s0.radius) {
            result = s1;
        }
        else {
            result = s0;
        }
    }
    else {
        float dist = sqrtf(sq_dist);
        result.radius = (dist + s0.radius + s1.radius) * 0.5f;
        result.center = s0.center;
        if (dist > 0.001f) {
            v /= dist;
            result.center += v * (result.radius - s0.radius);
        }
    }

    // HACK(bryan):  We can get a resulting sphere here that doesn't *quite* fit
    // the child spheres, so we multiply out a bit to get there.
    //
    // There is probably a better way to do this. 
    result.radius *= 1.005f;

    if (false) {
        Vector3 v0 = result.center - s0.center;
        float lv0 = Length(v0);
        ASSERT(lv0 + s0.radius <= result.radius);
        Vector3 v1 = result.center - s1.center;
        float lv1 = Length(v1);
        ASSERT(lv1 + s1.radius <= result.radius);
    }
    return result;
}


//
// Building sphere heirarchy:
// 
// Start by building sphere for each mesh group.
// Remove two closest spheres, create parent, add parent to list.
// Continue until list contains only one sphere.
//

static bool
FindClosestSpheres(std::vector<BoundingSphereP *> spheres, u32 * idx_a, u32 * idx_b) {
    u32 ssize = spheres.size();
    if (ssize < 2) {
        return false;
    }
    else {
        float min_dist = FLT_MAX;
        u32 best[2] = { ssize, ssize };
        for (u32 i = 0; i < ssize; ++i) {
            for (u32 j = i + 1; j < ssize; ++j) {
                BoundingSphereP * a = spheres[i];
                BoundingSphereP * b = spheres[j];

                Vector3 dc = a->s.center - b->s.center;
                float sq_dist = Dot(dc, dc);
                if (sq_dist < min_dist) {
                    best[0] = i;
                    best[1] = j;
                    min_dist = sq_dist;
                }
            }
        }
        *idx_a = best[0];
        *idx_b = best[1];
        return true;
    }
}

struct BoundingSphere {
    Sphere s;
    u32 c0;
    u32 c1;
};

struct BoundingHierarchy {
    std::vector<BoundingSphere> spheres;
    std::vector<MeshGroup *> mesh_groups;
    Mesh * mesh;
};

static u32
FlattenHierarchyTree(std::vector<BoundingSphere> * spheres, std::vector<MeshGroup *> * mesh_groups, BoundingSphereP * root) {
    assert(mesh_groups->size() == spheres->size());
    u32 my_idx = spheres->size();
    mesh_groups->push_back(root->mesh_group);
    spheres->push_back(BoundingSphere());
    BoundingSphere * ps = &(*spheres)[my_idx];
    ps->s = root->s;
    if (root->children[0]) {
        assert(root->children[1]);
        ps->c0 = FlattenHierarchyTree(spheres, mesh_groups, root->children[0]);
        ps->c1 = FlattenHierarchyTree(spheres, mesh_groups, root->children[1]);
    }
    else {
        // 0 is always the root of the tree, so we can use it as an invalid 
        // child sentinel.
        assert(root->mesh_group);
        ps->c0 = 0;
        ps->c1 = 0;
    }
    free(root);
    return my_idx;
}

static void
CheckInvariants(BoundingHierarchy * h, BoundingSphere s, u32 idx) {
    if (s.c0) {
        assert(s.c1);
        BoundingSphere s0 = h->spheres[s.c0];
        BoundingSphere s1 = h->spheres[s.c1];

        Vector3 v0 = s.s.center - s0.s.center;
        assert(Length(v0) + s0.s.radius <= s.s.radius);
        Vector3 v1 = s.s.center - s1.s.center;
        assert(Length(v1) + s1.s.radius <= s.s.radius);

        CheckInvariants(h, s0, s.c0);
        CheckInvariants(h, s1, s.c1);
    }
    else {
        assert(!s.c1);
        MeshGroup * mg = h->mesh_groups[idx];
        Mesh * mesh = h->mesh;
        for (u32 i = 0; i < mg->idx_positions.size(); ++i) {
            Vector3 p = mesh->positions[mg->idx_positions[i]];
            Vector3 d = p - s.s.center;
            assert(Dot(d, d) <= s.s.radius * s.s.radius);
        }
    }
}

static void
BuildHierarchy(BoundingHierarchy * h, Mesh * mesh) {
    std::vector<BoundingSphereP *> spheres;
    {
        TIME_BLOCK("Build Leaf Spheres");
        for (u32 i = 0; i < mesh->groups.size(); ++i) {
            MeshGroup * mg = &mesh->groups[i];

            BoundingSphereP * s = (BoundingSphereP *)calloc(1, sizeof(BoundingSphereP));
            s->s = BoundingSphere_FromMesh(mesh, mg);
            s->children[0] = NULL;
            s->children[1] = NULL;
            s->mesh_group = mg;
            spheres.push_back(s);
        }
    }

    u32 total_sphere_count = spheres.size();
    {
        TIME_BLOCK("Build Sphere Tree");
        // This algorithm is ~O(n^2) in number of leaf spheres.
        // This is *really slow* on large models.  So, we do want to
        // prebake this stuff as well.
        u32 sp_a;
        u32 sp_b;
        while (FindClosestSpheres(spheres, &sp_a, &sp_b)) {
            BoundingSphereP * a = spheres[sp_a];
            BoundingSphereP * b = spheres[sp_b];
            assert(a && b);
            if (sp_a > sp_b) {
                spheres.erase(spheres.begin() + sp_a);
                spheres.erase(spheres.begin() + sp_b);
            }
            else {
                spheres.erase(spheres.begin() + sp_b);
                spheres.erase(spheres.begin() + sp_a);
            }

            BoundingSphereP * parent = (BoundingSphereP *)calloc(1, sizeof(BoundingSphereP));
            assert(parent);
            parent->s = BoundingSphere_FromChildren(a->s, b->s);
            parent->children[0] = a;
            parent->children[1] = b;
            parent->mesh_group = NULL;

            spheres.push_back(parent);
            total_sphere_count++;
        }
    }
    
    // Flatten tree into an array.  This is not terribly efficient, but we're 
    // going to do this offline so I don't care. 
    h->spheres.reserve(total_sphere_count);
    h->mesh_groups.reserve(total_sphere_count);
    h->mesh = mesh;
    {
        TIME_BLOCK("Flatten Tree");
        FlattenHierarchyTree(&h->spheres, &h->mesh_groups, spheres[0]);
    }

    {
        TIME_BLOCK("Invariant Check");
        CheckInvariants(h, h->spheres[0], 0);
    }
}

#if 0
// Casting against spheres:
// Cast against root, if collides, cast against all children.  May hit multiple children,
// in which case *all* of them need to be checked.

static std::vector<u32>
Raycast_BoundingSpheres(Ray ray, BoundingSphere * root) {
    std::vector<BoundingSphere *> open_spheres;
    std::vector<u32> possible_tris;

    open_spheres.push_back(root);

    float t;
    while (open_spheres.size() > 0) {
        BoundingSphere * s = open_spheres.pop_back();
        if (IntersectRaySphere(ray, s, &t)) {
            if (s->is_leaf) {
                possible_tris.push_back(s->tri_first_index);
            }
            else {
                open_spheres.push_back(s->children[0]);
                open_spheres.push_back(s->children[1]);
            }
        }
    }

    return possible_tris;
}

const u32 INVALID_INDEX = 0xffffffff;

static bool
Raycast_TriangleList(Ray ray, std::vector<u32> possible_tris, u32 * hit_tri, TriangleHitParams * hit_params) {
    bool hit = false;
    u32 best_tri = INVALID_INDEX;
    TriangleHitParams best_params = {};
    float best_t;
    for (u32 i = 0; i < possible_tris.size(); ++i) {
        Vector3 a = GetVertexPos(possible_tris[i] + 0);
        Vector3 b = GetVertexPos(possible_tris[i] + 1);
        Vector3 c = GetVertexPos(possible_tris[i] + 2);
        float t;
        TriangleHitParams params;
        // TODO(bryan):  Due to alpha, this is not 100% correct, which poses
        // a problem for shadow ray casting.  (We may hit the polygon but not
        // *actually* be occluded if it has alpha < 1.)
        if (IntersectRayTriangle(ray, a, b, c, &params)) {
            if (t < best_t) {
                best_tri = possible_tris[i];
                best_params = params;
                best_t = t;
                hit = true;
            }
        }
    }
    if (hit_tri) *hit_tri = best_tri;
    if (hit_params) *hit_params = best_params;
    return hit;
}
#endif