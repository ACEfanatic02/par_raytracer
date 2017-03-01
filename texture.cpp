#include "brt.h"
#include "mathlib.h"
#include "mesh.h"

inline float
WrapUV(float uv) {
    if (uv >= 0.0f) {
        return fmodf(uv, 1.0f);
    }
    else {
        return 1.0f + fmodf(uv, 1.0f);
    }
}

const float gOneOver255 = 1.0f / 255.0f;

inline Vector4
GetTexel(Texture * texture, u32 x, u32 y) {
    assert(texture);
    assert(x < texture->size_x);
    assert(y < texture->size_y);
    u32 pixel_idx = y * texture->size_x + x;

    u8 r = 0;
    u8 g = 0;
    u8 b = 0;
    u8 a = 0;
    switch (texture->channels) {
        case 4: a = texture->texels[pixel_idx * texture->channels + 3];
        case 3: b = texture->texels[pixel_idx * texture->channels + 2];
        case 2: g = texture->texels[pixel_idx * texture->channels + 1];
        case 1: r = texture->texels[pixel_idx * texture->channels + 0];
    }

    // Set alpha to full for all maps without an alpha channel.
    if (texture->channels < 4) {
        a = 255;
    }
    // One-channel maps get splatted out to all three color channels.
    if (texture->channels == 1) {
        b = g = r;
    }

    Vector4 color = Vector4(r, g, b, a) * gOneOver255;
    color.x = Color_SRGBToLinear(color.x);
    color.y = Color_SRGBToLinear(color.y);
    color.z = Color_SRGBToLinear(color.z);
    color.w = Color_SRGBToLinear(color.w);

    return color;
}

Vector4
Texture_SampleBilinear(Texture * texture, float u, float v) {
    // TODO(bryan):  We can get much better cache performance by swizzling our textures.
    if (!texture) {
        return Vector4();
    }

    u = WrapUV(u);
    v = WrapUV(v);

    float tx = u * (texture->size_x - 1);
    float ty = v * (texture->size_y - 1);

    u32 tx0 = (u32)floorf(tx);
    u32 ty0 = (u32)floorf(ty);
    u32 tx1 = tx0 + 1;
    u32 ty1 = ty0 + 1;

    float fx = tx - tx0;
    float fy = ty - ty0;

    Vector4 s00 = GetTexel(texture, tx0, ty0);
    Vector4 s01 = GetTexel(texture, tx0, ty1);
    Vector4 s10 = GetTexel(texture, tx1, ty0);
    Vector4 s11 = GetTexel(texture, tx1, ty1);

    return Lerp(Lerp(s00, s01, fy), Lerp(s10, s11, fy), fx);
}

inline void
WriteNormal(Texture * t, u32 x, u32 y, Vector3 n) {
    assert(t);
    assert(t->channels == 3);
    assert(x < t->size_x);
    assert(y < t->size_y);

    // Remap from (-1, 1) to (0, 1)
    n = (n + Vector3(1.0f, 1.0f, 1.0f)) * 0.5f;

    u32 pixel_idx = y * t->size_x + x;
    // TODO(bryan):  We probably don't *really* want to store normals in sRGB?  
    t->texels[pixel_idx * 3 + 0] = Color_LinearToSRGB(n.x);
    t->texels[pixel_idx * 3 + 1] = Color_LinearToSRGB(n.y);
    t->texels[pixel_idx * 3 + 2] = Color_LinearToSRGB(n.z);
}

Texture *
ConvertHeightMapToNormalMap(Texture * height_map) {
    assert(height_map);
    assert(height_map->channels == 1);

    Texture * result = (Texture *)calloc(1, sizeof(Texture));
    result->size_x = height_map->size_x;
    result->size_y = height_map->size_y;
    result->texels = (u8 *)calloc(result->size_x * result->size_y, 3);
    result->channels = 3;

    for (u32 y = 0; y < result->size_y; ++y) {
        for (u32 x = 0; x < result->size_x; ++x) {
            float h00 = GetTexel(height_map, x,     y    ).x;
            float h10 = GetTexel(height_map, x + 1, y    ).x;
            float h01 = GetTexel(height_map, x,     y + 1).x;

            Vector3 tx(h10 - h00, 0.0f, 0.0f);
            Vector3 ty(0.0f, h01 - h00, 0.0f);

            Vector3 norm = Cross(tx, ty);

            WriteNormal(result, x, y, norm);
        }
    }

    return result;
}
