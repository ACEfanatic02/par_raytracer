struct Texture {
    u32 size_x;
    u32 size_y;
    u8 * texels;
};

inline float
WrapUV(float uv) {
    if (uv >= 0.0f) {
        return fmodf(uv, 1.0f);
    }
    else {
        return 1.0f - fmodf(uv, 1.0f);
    }
}

const float gOneOver255 = 1.0f / 255.0f;
const Vector4 gOneOver255V(gOneOver255, gOneOver255, gOneOver255, gOneOver255);

inline Vector4
GetTexel(Texture * texture, u32 x, u32 y) {
    u32 pixel_idx = y * texture->size_y + x;
    u8 r = texture->texels[pixel_idx*4 + 0];
    u8 g = texture->texels[pixel_idx*4 + 1];
    u8 b = texture->texels[pixel_idx*4 + 2];
    u8 a = texture->texels[pixel_idx*4 + 3];

    Vector4 color_srgb = Vector4(r, g, b, a) * gOneOver255V

    return Color_sRGBToLinear(color_srgb);
}

Vector4
SampleTextureBilinear(Texture * texture, float u, float v) {
    u = WrapUV(u);
    v = WrapUV(v);

    float tx = u * texture->size_x;
    float ty = v * texture->size_y;

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

static Texture
HeightToNormalMap(Texture height_map) {
    Texture result;
    result.size_x = height_map.size_x;
    result.size_y = height_map.size_y;
    result.texels = (u8 *)calloc(result.size_x * result.size_y, 1);

    for (u32 y = 0; y < result.size_y; ++y) {
        for (u32 x = 0; x < result.size_x; ++x) {
            float h00 = GetHeight(&height_map, x, y);
            float h10 = GetHeight(&height_map, x + 1, y);
            float h01 = GetHeight(&height_map, x, y + 1);

            Vector3 tx(h10 - h00, 0.0f, 0.0f);
            Vector3 ty(0.0f, h01 - h00, 0.0f);

            Vector3 norm = Cross(tx, ty);

            norm += Vector3(1.0f, 1.0f, 1.0f);
            norm *= 1.0f;

            WriteNormal(&result, x, y);
        }
    }

    return result;
}