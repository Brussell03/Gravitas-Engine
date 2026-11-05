Shader "GravitasDemo/MapObjectSelectionRing"
{
    Properties
    {
        _RingRadius ("Ring Radius", Range(0.1, 0.5)) = 0.465
        _RingHalfWidth ("Ring Half Width", Range(0.001, 0.1)) = 0.0175
    }

    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" }
        Cull Off
        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma target 3.5
            #include "UnityCG.cginc"

            float _RingRadius;
            float _RingHalfWidth;

            UNITY_INSTANCING_BUFFER_START(InstanceProperties)
                UNITY_DEFINE_INSTANCED_PROP(float4, _InstanceColor)
            UNITY_INSTANCING_BUFFER_END(InstanceProperties)

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float4 position : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR0;
            };

            v2f vert(appdata input)
            {
                v2f output;
                UNITY_SETUP_INSTANCE_ID(input);
                output.position = UnityObjectToClipPos(input.vertex);
                output.uv = input.uv;
                output.color = UNITY_ACCESS_INSTANCED_PROP(InstanceProperties, _InstanceColor);
                return output;
            }

            fixed4 frag(v2f input) : SV_Target
            {
                float radius = length(input.uv - 0.5);
                float distanceFromRing = abs(radius - _RingRadius);
                float aa = max(fwidth(distanceFromRing), 0.001);
                float alpha = 1.0 - smoothstep(_RingHalfWidth, _RingHalfWidth + aa, distanceFromRing);
                return fixed4(input.color.rgb, input.color.a * alpha);
            }
            ENDCG
        }
    }
}
