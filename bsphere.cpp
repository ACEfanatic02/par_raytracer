#include "brt.h"
#include "mathlib.h"
#include "mesh.h"
#include "geometry.h"
#include "random.h"
#include "timing.h"

struct BoundingSphereP {
    Sphere s;
    BoundingSphereP * children[2];
    MeshGroup * mesh_group;
};

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

static void
FindExtremePointsAlongDirection(Vector3 dir, Vector3 * points, u32 point_count, u32 * idx_min, u32 * idx_max) {
    float min_proj = FLT_MAX;
    float max_proj = -FLT_MAX;
    for (u32 i = 0; i < point_count; ++i) {
        float proj = Dot(points[i], dir);
        if (proj < min_proj) {
            *idx_min = i;
            min_proj = proj;
        }
        if (proj > max_proj) {
            *idx_max = i;
            max_proj = proj;
        }
    }
}

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

static void
Jacobi(Matrix33 * a, Matrix33 * v) {
    float prevoff;
    float c, s;
    Matrix33 J, b, t;

    v->SetIdentity();

    const u32 max_iterations = 50;
    for (u32 n = 0; n < max_iterations; ++n) {
        u32 p = 0;
        u32 q = 1;
        for (u32 i = 0; i < 3; ++i) {
            for (u32 j = 0; j < 3; ++j) {
                if (i != j) {
                    float ij = (*a)(i, j);
                    float pq = (*a)(p, q);
                    if (fabsf(ij) > fabsf(pq)) {
                        p = i;
                        q = j;
                    }
                }
            }
        }

        SymSchur2(*a, p, q, &c, &s);
        J.SetIdentity();
        J(p, p) = c;
        J(p, q) = s;
        J(q, p) = -s;
        J(q, q) = c;

        *v = *v * J;

        *a = Transpose(J) * (*a) * J;

        float off = 0.0f;
        for (u32 i = 0; i < 3; ++i) {
            for (u32 j = 0; j < 3; ++j) {
                if (i != j) {
                    off += (*a)(i, j) * (*a)(i, j);
                }
            }
        }

        if (n > 2 && off >= prevoff) {
            return;
        }
        prevoff = off;
    }
}

static Sphere
EigenSphere(Vector3 * points, u32 point_count) {
    Matrix33 m = CovarianceMatrix(points, point_count);
    Matrix33 v;
    Jacobi(&m, &v);

    Vector3 e;
    u32 max_c = 0;
    float max_e = fabsf(m(0, 0));
    if (fabsf(m(1, 1)) > max_e) {
        max_c = 1;
        max_e = fabsf(m(1, 1));
    }
    if (fabsf(m(2, 2)) > max_e) {
        max_c = 2;
        max_e = fabsf(m(2, 2));
    }

    e.x = v(0, max_c);
    e.y = v(1, max_c);
    e.z = v(2, max_c);

    u32 idx_min;
    u32 idx_max;
    FindExtremePointsAlongDirection(e, points, point_count, &idx_min, &idx_max);
    Vector3 min_pt = points[idx_min];
    Vector3 max_pt = points[idx_max];

    Sphere result;
    result.center = (min_pt + max_pt) * 0.5f;
    result.radius = Length(min_pt - max_pt) * 0.5f;

    for (u32 i = 0; i < point_count; ++i) {
        UpdateSphereWithPoint(&result, points[i]);

        assert(Length(points[i] - result.center) <= result.radius);
    }

    return result;
}

static Sphere
Ritter_Iterative(Sphere s, Vector3 * points, u32 point_count) {
    const u32 num_iters = 16;

    RandomState rng;
    Random_Seed(&rng, 0x201701260526ull);
    Sphere s2 = s;
    for (u32 k = 0; k < num_iters; ++k) {
        s2.radius *= 0.9f;

        for (u32 i = 0; i < point_count; ++i) {
            // Shuffle points (results in better spheres for same iter count.)
            u32 remaining = point_count - i - 1;
            if (remaining) {
                u32 j = (u32)Random_Next(&rng) % remaining;
                j += i + 1;
                assert(j < point_count);
                Vector3 tmp = points[i];
                points[i] = points[j];
                points[j] = tmp;
            }

            UpdateSphereWithPoint(&s2, points[i]);
        }

        if (s2.radius < s.radius) s = s2;
    }

    // One final pass to make absolutely sure everything's inside.
    for (u32 i = 0; i < point_count; ++i) {
        UpdateSphereWithPoint(&s, points[i]);
    }
    return s;
}

static Sphere
BoundingSphere_FromMesh(Mesh * mesh, MeshGroup * group) {
    u32 point_count = group->idx_positions.size();
    Vector3 * points = (Vector3 *)calloc(point_count, sizeof(Vector3));
    for (u32 i = 0; i < point_count; ++i) {
        u32 idx = group->idx_positions[i];
        points[i] = mesh->positions[idx];
    }

    Sphere result = EigenSphere(points, point_count);
    result = Ritter_Iterative(result, points, point_count);

    free(points);
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
    result.radius *= 1.0001f;
    return result;
}

static bool
FindMergeCandidates(std::vector<BoundingSphereP *> spheres, u32 * idx_a, u32 * idx_b, Sphere * merged) {
    u32 sphere_count = spheres.size();
    if (sphere_count < 2) {
        return false;
    }
    else {
        float min_heuristic = FLT_MAX;
        u32 best[2] = { sphere_count, sphere_count };
        for (u32 i = 0; i < sphere_count; ++i) {
            for (u32 j = i + 1; j < sphere_count; ++j) {
                BoundingSphereP * s0 = spheres[i];
                BoundingSphereP * s1 = spheres[j];
                Sphere parent = BoundingSphere_FromChildren(s0->s, s1->s);
                // NOTE(bryan):  We minimize surface area.  SA=4pi*r^2
                // Because constant multiplication and squaring do not affect ordering, 
                // we can just use the radius directly as our heuristic.  (In addition 
                // to less work, this gives us slightly better precision.)
                float heuristic = parent.radius;

                if (heuristic < min_heuristic) {
                    best[0] = i;
                    best[1] = j;

                    *merged = parent;
                    min_heuristic = heuristic;
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
            assert(Length(d) <= s.s.radius);
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
        Sphere merged;
        while (FindMergeCandidates(spheres, &sp_a, &sp_b, &merged)) {
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
            parent->s = merged;
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
