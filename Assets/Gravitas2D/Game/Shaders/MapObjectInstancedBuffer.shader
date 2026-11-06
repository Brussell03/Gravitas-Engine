Shader "GravitasDemo/MapObjectInstancedBuffer"
{
    Properties
    {
        _MainTex ("Sprite Texture", 2D) = "white" {}
    }

    SubShader
    {
        Tags
        {
            "Queue" = "Transparent"
            "RenderType" = "Transparent"
            "IgnoreProjector" = "True"
        }

        Pass
        {
            Blend SrcAlpha OneMinusSrcAlpha
            ZWrite Off
            Cull Off

            CGPROGRAM
            #pragma target 4.5
            #pragma vertex Vert
            #pragma fragment Frag

            #include "UnityCG.cginc"

            struct MapGpuInstance
            {
                float4 positionSizeRotation;
                float4 color;
            };

            StructuredBuffer<MapGpuInstance> _MapInstances;

            sampler2D _MainTex;
            float4 _MapWorldOrigin;
            float4 _MapPositionRight;
            float4 _MapPositionUp;
            float4 _MapSizeRight;
            float4 _MapSizeUp;

            struct AppData
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct Varyings
            {
                float4 position : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR0;
            };

            Varyings Vert(AppData input, uint instanceID : SV_InstanceID)
            {
                MapGpuInstance instance = _MapInstances[instanceID];
                float sine;
                float cosine;
                sincos(instance.positionSizeRotation.w, sine, cosine);

                // The cached sprite mesh is normalized to its local bounds [-0.5, 0.5].
                // Its per-instance scale and rotation are applied here, not through a
                // 64-byte object-to-world matrix on the CPU.
                float2 spriteOffset = input.vertex.xy * instance.positionSizeRotation.z;
                float2 rotatedOffset = float2(
                    spriteOffset.x * cosine - spriteOffset.y * sine,
                    spriteOffset.x * sine + spriteOffset.y * cosine);

                float3 worldPosition = _MapWorldOrigin.xyz
                    + _MapPositionRight.xyz * instance.positionSizeRotation.x
                    + _MapPositionUp.xyz * instance.positionSizeRotation.y
                    + _MapSizeRight.xyz * rotatedOffset.x
                    + _MapSizeUp.xyz * rotatedOffset.y;

                Varyings output;
                output.position = mul(UNITY_MATRIX_VP, float4(worldPosition, 1.0));
                output.uv = input.uv;
                output.color = instance.color;
                return output;
            }

            fixed4 Frag(Varyings input) : SV_Target
            {
                return tex2D(_MainTex, input.uv) * input.color;
            }
            ENDCG
        }
    }
}
