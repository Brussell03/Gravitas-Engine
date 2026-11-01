Shader "Unlit/BLineShader"
{
    Properties
    {
        _LineWidth ("Line Width", Range(0.1, 100)) = 1
        _SheenColor ("Sheen Color", Color) = (1,1,1,0.5)
        _SheenSpeed ("Sheen Speed", float) = 1.0
        _SheenWidth ("Sheen Width", Range(0.01, 1)) = 0.2
        _SheenCount ("Sheen Count", Range(1, 10)) = 2

        // --- DASH PROPERTIES ---
        //_DashLength ("Dash Length", Range(0, 100)) = 10.0 // World space length of a dash
        //_DashSolidRatio ("Dash Solid Ratio", Range(0.01, 1.0)) = 0.8
    }
    SubShader
    {
        Tags { "RenderType"="Transparent" "Queue"="Transparent" }
        LOD 100
        Cull Off 
        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma geometry geo
            #pragma fragment frag
            #include "UnityCG.cginc"
            
            struct LineData
            {
                uint startIndex;
                uint pointCount;
            };

            StructuredBuffer<float3> _PointsBuffer;
            StructuredBuffer<float4> _ColorsBuffer;
            //StructuredBuffer<float3> _CentersBuffer;
            StructuredBuffer<LineData> _LineDataBuffer;
            
            float _LineWidth;
            float4 _SheenColor;
            float _SheenSpeed;
            float _SheenWidth;
            float _SheenCount;

            // --- DASH UNIFORMS ---
            //float _DashLength;
            //float _GapLength;
            //float _DashSolidRatio;

            struct v2g
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
                uint vertexId : TEXCOORD0;
                uint pointCount : TEXCOORD1;
            };

            struct g2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
            };

            v2g vert(uint vertexId : SV_VertexID, uint instanceId : SV_InstanceID)
            {
                v2g o;

                LineData lineData = _LineDataBuffer[instanceId];
                
                uint clampedVertexId = min(vertexId, lineData.pointCount - 1);

                uint pointIndex = lineData.startIndex + clampedVertexId;

                o.pos = float4(_PointsBuffer[pointIndex]/* + _CentersBuffer[instanceId]*/, 1.0);
                o.color = _ColorsBuffer[instanceId];
                o.vertexId = vertexId;
                o.pointCount = lineData.pointCount;
                return o;
            }

            [maxvertexcount(4)]
            void geo(line v2g p[2], inout TriangleStream<g2f> triStream)
            {
                if (p[1].vertexId >= p[0].pointCount)
                {
                    return;
                }

                // Get the start and end points of the line segment in world space
                float3 p0 = p[0].pos.xyz;
                float3 p1 = p[1].pos.xyz;

                // Transform world positions to clip space (screen space)
                float4 p0_clip = UnityObjectToClipPos(p0);
                float4 p1_clip = UnityObjectToClipPos(p1);

                // Calculate the direction of the line in screen space
                float2 screen_dir = normalize((p1_clip.xy / p1_clip.w) - (p0_clip.xy / p0_clip.w));
                
                // Find the perpendicular vector (the normal) for the offset
                float2 normal = float2(-screen_dir.y, screen_dir.x);
                
                // Get the screen's aspect ratio.
                float aspect = _ScreenParams.x / _ScreenParams.y;

                // Correct the normal vector by scaling its horizontal component.
                float2 aspect_normal = float2(normal.x / aspect, normal.y);

                // Calculate the offset amount based on line width
                // We divide by the screen resolution (_ScreenParams.y) to make the width consistent
                float offset = _LineWidth / _ScreenParams.y;

                // Create the four vertices for the quad
                g2f v;
                v.color = p[0].color;

                // Vertex 1: Start point, offset left
                v.pos = p0_clip;
                v.pos.xy += offset * aspect_normal * p0_clip.w;
                triStream.Append(v);

                // Vertex 2: End point, offset left
                v.pos = p1_clip;
                v.pos.xy += offset * aspect_normal * p1_clip.w;
                triStream.Append(v);
                
                // Vertex 3: Start point, offset right
                v.pos = p0_clip;
                v.pos.xy -= offset * aspect_normal * p0_clip.w;
                triStream.Append(v);

                // Vertex 4: End point, offset right
                v.pos = p1_clip;
                v.pos.xy -= offset * aspect_normal * p1_clip.w;
                triStream.Append(v);
            }

            // Fragment shader: just outputs the final color
            fixed4 frag(g2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
    Fallback "Transparent/VertexLit"
}