#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position;
    float3 normal;
};

struct Uniforms {
    float4x4 mvp;
    float4x4 model;
    float3x3 normalMatrix;
    float3   lightDir;   // direction FROM surface TO light (world space)
    float3   cameraPos;  // world space
    float    _pad;       // padding for 16-byte alignment
};

struct VertexOut {
    float4 position [[position]];
    float3 worldPos;
    float3 normal;
};

vertex VertexOut vertex_main(const device VertexIn* vtx [[buffer(0)]],
                             constant Uniforms& u [[buffer(1)]],
                             uint vid [[vertex_id]]) {
    VertexOut out;
    float4 world = u.model * float4(vtx[vid].position, 1.0);
    out.position = u.mvp * float4(vtx[vid].position, 1.0);
    out.worldPos = world.xyz;
    out.normal   = normalize(u.normalMatrix * vtx[vid].normal);
    return out;
}

fragment float4 fragment_main(VertexOut in [[stage_in]],
                              constant Uniforms& u [[buffer(1)]]) {
    float3 N = normalize(in.normal);
    float3 L = normalize(u.lightDir);
    float3 V = normalize(u.cameraPos - in.worldPos);
    float3 H = normalize(L + V);

    // "Matte metal" look: low diffuse, broad soft spec
    float3 albedo = float3(0.72, 0.73, 0.76);  // metal gray
    float  NdotL  = max(dot(N, L), 0.0);

    float3 ambient = albedo * 0.08;

    // Metals are mostly specular; keep diffuse subtle
    float3 diffuse = albedo * (0.22 * NdotL);

    // Broad specular highlight (matte-ish): low-ish exponent
    float  spec = pow(max(dot(N, H), 0.0), 18.0);
    float3 specular = albedo * (0.55 * spec) * step(0.0, NdotL);

    float3 color = ambient + diffuse + specular;

    // simple gamma-ish correction (optional but helps)
    color = pow(color, float3(1.0 / 2.2));

    return float4(color, 1.0);
}
