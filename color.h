#include "brt.h"

inline float
Color_LinearToSRGB(float linear) {
    if (linear <= 0.0031308f) {
        return 12.92f*linear;
    }
    else {
        return 1.055f*powf(linear, 1.0f/2.4f) - 0.055f;
    }
}

inline float
Color_SRGBToLinear(float srgb) {
    if (srgb <= 0.04045f) {
        return srgb / 12.92f;
    }
    else {
        return powf((srgb + 0.055f) / 1.055f, 2.4f);
    }
}

static Vector4
Color_RGBToHSV(Vector4 rgba) {
    // http://lolengine.net/blog/2013/01/13/fast-rgb-to-hsv
    float K = 0.0f;
    if (rgba.y < rgba.z) {
        float temp = rgba.y;
        rgba.y = rgba.z;
        rgba.z = temp;
        K = -1.0f;
    }
    if (rgba.x < rgba.y) {
        float temp = rgba.x;
        rgba.x = rgba.y;
        rgba.y = temp;
        K = (-2.0f / 6.0f) - K;
    }

    float chroma = rgba.x - min(rgba.y, rgba.z);
    float h = fabs(K + (rgba.y - rgba.z) / (6.0f * chroma + 1e-20f));
    float s = chroma / (rgba.x + 1e-20f);
    float v = rgba.x;

    return Vector4(h, s, v, rgba.w);
}

static Vector4
Color_HSVToRGB(Vector4 hsva) {
    // http://lolengine.net/blog/2013/07/27/rgb-to-hsv-in-glsl
    float K0 = 1.0f;
    float K1 = 2.0f / 3.0f;
    float K2 = 1.0f / 3.0f;
    float K3 = 3.0f;

    float p0 = fabs(Fract(hsva.x + K0) * 6.0f - K3);
    float p1 = fabs(Fract(hsva.x + K1) * 6.0f - K3);
    float p2 = fabs(Fract(hsva.x + K2) * 6.0f - K3);

    float r = hsva.z * Lerp(K0, Clamp(p0 - K0, 0.0f, 1.0f), hsva.y);
    float g = hsva.z * Lerp(K0, Clamp(p1 - K0, 0.0f, 1.0f), hsva.y);
    float b = hsva.z * Lerp(K0, Clamp(p2 - K0, 0.0f, 1.0f), hsva.y);
    return Vector4(r, g, b, hsva.w);
}

inline Vector4
Color_BlendHSV(Vector4 rgba0, Vector4 rgba1, float t) {
    Vector4 hsv0 = Color_RGBToHSV(rgba0);
    Vector4 hsv1 = Color_RGBToHSV(rgba1);

    return Color_HSVToRGB(Lerp(hsv0, hsv1, t));
}

inline Vector4
Color_Unpack(u8 * p) {
    Vector4 result(p[0], p[1], p[2], p[3]);
    return result / 255.0f;
}

inline void
Color_Pack(u8 * p, Vector4 c) {
    p[0] = (u8)(Clamp(c.x, 0.0f, 1.0f) * 255.0f);
    p[1] = (u8)(Clamp(c.y, 0.0f, 1.0f) * 255.0f);
    p[2] = (u8)(Clamp(c.z, 0.0f, 1.0f) * 255.0f);
    p[3] = (u8)(Clamp(c.w, 0.0f, 1.0f) * 255.0f);
}

inline Vector4
Color_FromNormal(Vector3 n) {
    return (Vector4(n.x, n.y, n.z, 1.0f) + Vector4(1.0f, 1.0f, 1.0f, 1.0f)) * 0.5f;
}