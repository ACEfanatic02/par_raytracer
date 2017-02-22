// TODO(bryan): Cleanup this data structure.
struct BoundingSphere {
    Vector3 center;
    float radius;
    BoundingSphere * children[2];
    u32 tri_first_index;
    bool is_leaf;
};

struct BoundingSphere {
    Sphere s;
    BoundingSphere * children[2];
    Mesh * mesh;
};

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

static void
UpdateSphereWithPoint(BoundingSphere * s, Vector3 p) {
    Vector3 pc = p - s->center;
    float sq_dist = Dot(pc, pc);
    if (sq_dist > (s->radius * s->radius)) {
        float dist = sqrtf(sq_dist);
        float new_radius = (s->radius + dist) * 0.5f;
        float k = (new_radius - s->radius) / dist;

        s->radius = new_radius;
        s->center += pc * k;
    }
}

static BoundingSphere
BoundingSphere_FromMesh(Vector3 * points, u32 point_count) {
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
    u32 minx_idx = 0;
    u32 miny_idx = 0;
    u32 minz_idx = 0;
    u32 maxx_idx = 0;
    u32 maxy_idx = 0;
    u32 maxz_idx = 0;
    for (u32 i = 0; i < point_count; ++i) {
        if (points[i].x < points[minx_idx].x) minx_idx = i;
        if (points[i].y < points[miny_idx].y) miny_idx = i;
        if (points[i].z < points[minz_idx].z) minz_idx = i;

        if (points[i].x > points[maxx_idx].x) maxx_idx = i;
        if (points[i].y > points[maxy_idx].y) maxy_idx = i;
        if (points[i].z > points[maxz_idx].z) maxz_idx = i;
    }

    Vector3 vx = points[minx_idx] - points[maxx_idx];
    Vector3 vy = points[miny_idx] - points[maxy_idx];
    Vector3 vz = points[minz_idx] - points[maxz_idx];

    float sq_distx = Dot(vx, vx);
    float sq_disty = Dot(vy, vy);
    float sq_distz = Dot(vz, vz);

    // Pick the two most separated points.
    u32 min_idx = minx_idx;
    u32 max_idx = maxx_idx;
    if (sq_disty > sq_distx && sq_disty > sq_distz) {
        max_idx = maxy_idx;
        min_idx = miny_idx; 
    }
    if (sq_distz > sq_distx && sq_distz > sq_disty) {
        max_idx = maxz_idx;
        min_idx = minz_idx;
    }

    BoundingSphere result;
    result.center = (points[min_idx] + points[max_idx]) * 0.5f;
    result.radius = sqrtf(Dot(points[max_idx] - result.center, points[max_idx] - result.center));

    for (u32 i = 0; i < point_count; ++i) {
        UpdateSphereWithPoint(&result, points[i]);
    }

    return result;
}

static BoundingSphere
BoundingSphere_FromChildren(BoundingSphere * s0, BoundingSphere * s1) {
    BoundingSphere result;
    result.center = (s0->center + s1->center) * 0.5f;

    Vector3 v0 = s0->center - result.center;
    Vector3 v1 = s1->center - result.center;

    float r0 = Length(v0) + s0->radius;
    float r1 = Length(v1) + s1->radius;

    result.radius = max(r0, r1);

    result.children[0] = s0;
    result.children[1] = s1;
}


//
// Building sphere heirarchy:
// 
// Start by building sphere for each triangle.
// Remove two closest spheres, create parent, add parent to list.
// Continue until list contains only one sphere.
//

static bool
FindClosestSpheres(std::vector<BoundingSphere *> spheres, u32 * idx_a, u32 * idx_b) {
    if (spheres.size() < 2) {
        return false;
    }
    else {
        float min_dist = FLT_MAX;
        u32 best[2];
        for (u32 i = 0; i < spheres.size(); ++i) {
            for (u32 j = 0; j < spheres.size(); ++j) {
                BoundingSphere * a = spheres[i];
                BoundingSphere * b = spheres[j];

                Vector3 dc = a->center - b->center;
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

static BoundingSphere *
BuildHierarchy(Mesh * meshes, u32 mesh_count) {
    std::vector<BoundingSphere *> spheres;
    for (u32 mesh_idx = 0; mesh_idx < mesh_count; ++mesh_idx) {
        Mesh * mesh = meshes + mesh_idx;
        u32 index_count = mesh->index_count;
        for (u32 i = 0; i < index_count; i += 3) {
            u32 idx_a = mesh->indices[i + 0];
            u32 idx_b = mesh->indices[i + 1]
            u32 idx_c = mesh->indices[i + 2]
            Vector3 a = mesh->GetVertexPos(idx_a);
            Vector3 b = mesh->GetVertexPos(idx_b);
            Vector3 c = mesh->GetVertexPos(idx_c);

            BoundingSphere * s = calloc(1, sizeof(BoundingSphere));
            *s = BoundingSphere_FromTriangle(a, b, c);
            s->tri_first_index = idx_a;
            s->is_leaf = true;
            spheres.push_back(s);
        }
    }

    u32 sp_a;
    u32 sp_b;
    while (FindClosestSpheres(spheres, &sp_a, &sp_b)) {
        BoundingSphere * a = spheres[sp_a];
        BoundingSphere * b = spheres[sp_b];
        spheres.erase(spheres.begin() + sp_a, spheres.begin() + sp_a);
        spheres.erase(spheres.begin() + sp_b, spheres.begin() + sp_b);

        BoundingSphere * parent = calloc(1, sizeof(BoundingSphere))
        *parent = BoundingSphere_FromChildren(a, b);

        spheres.push_back(parent);
    }

    assert(spheres.size() == 1);
    return spheres[0];
}


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