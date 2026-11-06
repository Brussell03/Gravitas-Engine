Shader "GravitasDemo/ProceduralGrid"
{
    Properties
    {
        [PerRendererData] _MainTex ("Sprite Texture", 2D) = "white" {}
        _Color ("Tint", Color) = (1, 1, 1, 1)
        _GridMajorColor ("Major Grid Color", Color) = (1, 1, 1, 0.1)
        _GridMinorColor ("Minor Grid Color", Color) = (1, 1, 1, 0.05)
        _GridMajorThickness ("Major Grid Thickness (UI Units)", Float) = 1
        _GridMinorThickness ("Minor Grid Thickness (UI Units)", Float) = 0.5
        _GridPhase ("Grid Phase", Vector) = (0, 0, 0, 0)
        _GridCells ("Grid Cells", Vector) = (50, 50, 5, 2500)
        _GridEnabled ("Grid Enabled", Float) = 1
        _MaskSoftnessX ("Mask Softness X", Float) = 0
        _MaskSoftnessY ("Mask Softness Y", Float) = 0

        // Standard UI stencil/clip properties. These allow Mask and RectMask2D to work.
        _StencilComp ("Stencil Comparison", Float) = 8
        _Stencil ("Stencil ID", Float) = 0
        _StencilOp ("Stencil Operation", Float) = 0
        _StencilWriteMask ("Stencil Write Mask", Float) = 255
        _StencilReadMask ("Stencil Read Mask", Float) = 255
        _ColorMask ("Color Mask", Float) = 15
        [Toggle(UNITY_UI_ALPHACLIP)] _UseUIAlphaClip ("Use Alpha Clip", Float) = 0
    }

    SubShader
    {
        Tags
        {
            "Queue" = "Transparent"
            "IgnoreProjector" = "True"
            "RenderType" = "Transparent"
            "PreviewType" = "Plane"
            "CanUseSpriteAtlas" = "True"
        }

        Stencil
        {
            Ref [_Stencil]
            Comp [_StencilComp]
            Pass [_StencilOp]
            ReadMask [_StencilReadMask]
            WriteMask [_StencilWriteMask]
        }

        Cull Off
        Lighting Off
        ZWrite Off
        ZTest [unity_GUIZTestMode]
        Blend SrcAlpha OneMinusSrcAlpha
        ColorMask [_ColorMask]

        Pass
        {
            Name "ProceduralGrid"
            CGPROGRAM
            #pragma target 3.0
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_local _ UNITY_UI_CLIP_RECT
            #pragma multi_compile_local _ UNITY_UI_ALPHACLIP

            #include "UnityCG.cginc"
            #include "UnityUI.cginc"

            fixed4 _GridMajorColor;
            fixed4 _GridMinorColor;
            float _GridMajorThickness;
            float _GridMinorThickness;
            float4 _GridPhase; // x/y: fractional cell phase at left/bottom; z/w: start index modulo subdivisions
            float4 _GridCells; // x/y: cells across each axis; z: subdivisions; w: maximum rendered line index
            float _GridEnabled;
            float4 _ClipRect;
            float _UIMaskSoftnessX;
            float _UIMaskSoftnessY;
            float _MaskSoftnessX;
            float _MaskSoftnessY;
            fixed4 _Color;

            struct appdata_t
            {
                float4 vertex : POSITION;
                fixed4 color : COLOR;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                fixed4 color : COLOR;
                float2 uv : TEXCOORD0;
                float2 localPosition : TEXCOORD1;
                float4 worldPosition : TEXCOORD2;
                half4 mask : TEXCOORD3;
            };

            v2f vert(appdata_t input)
            {
                v2f output;
                output.worldPosition = input.vertex;
                output.vertex = UnityObjectToClipPos(input.vertex);
                output.localPosition = input.vertex.xy;
                output.uv = input.uv;
                output.color = input.color * _Color;

                // Match the built-in UI shader's RectMask2D softness calculation.
                float2 pixelSize = output.vertex.w;
                pixelSize /= abs(mul((float2x2)UNITY_MATRIX_P, _ScreenParams.xy));
                float4 clampedRect = clamp(_ClipRect, -2e10, 2e10);
                half2 maskSoftness = half2(
                    max(_UIMaskSoftnessX, _MaskSoftnessX),
                    max(_UIMaskSoftnessY, _MaskSoftnessY));
                output.mask = half4(
                    input.vertex.xy * 2.0 - clampedRect.xy - clampedRect.zw,
                    0.25 / (0.25 * maskSoftness + abs(pixelSize.xy)));
                return output;
            }

            float LineCoverage(
                float coordinate,
                float nearestIndex,
                float coordinatePerPixel,
                float localUnitsPerPixel,
                float thickness)
            {
                if (thickness <= 0.0)
                    return 0.0;

                // Grid line width is specified in the same local UI units as the old
                // LineGraphic mesh thickness. Derivatives convert pixel distance back
                // to those units, accounting for Canvas scale and RectTransform scale.
                float distanceLocal = abs(coordinate - nearestIndex) *
                    localUnitsPerPixel / max(coordinatePerPixel, 1e-7);
                float halfWidth = thickness * 0.5;
                float antialiasWidth = max(localUnitsPerPixel * 0.5, 1e-5);
                return 1.0 - smoothstep(
                    max(0.0, halfWidth - antialiasWidth),
                    halfWidth + antialiasWidth,
                    distanceLocal);
            }

            float IsMajorLine(float startModulo, float lineOffset, float subdivisions)
            {
                float wrappedIndex = fmod(startModulo + lineOffset, subdivisions);
                return 1.0 - step(0.25, min(wrappedIndex, subdivisions - wrappedIndex));
            }

            fixed4 frag(v2f input) : SV_Target
            {
                if (_GridEnabled < 0.5)
                    return fixed4(0, 0, 0, 0);

                float subdivisions = max(1.0, _GridCells.z);
                float maxLineIndex = _GridCells.w;

                // The grid coordinates start at floor((viewMin - referenceCenter)/minorStep).
                // Keeping only the fractional phase here avoids sending large world
                // coordinates to the GPU, where they would lose sub-grid precision.
                float coordinateX = _GridPhase.x + input.uv.x * _GridCells.x;
                float coordinateY = _GridPhase.y + input.uv.y * _GridCells.y;
                float nearestX = floor(coordinateX + 0.5);
                float nearestY = floor(coordinateY + 0.5);

                float validX = step(0.0, nearestX) * step(nearestX, maxLineIndex);
                float validY = step(0.0, nearestY) * step(nearestY, maxLineIndex);

                // Use the Euclidean screen-space gradient rather than fwidth's L1
                // approximation so local line thickness remains consistent when the
                // visualization RectTransform is rotated.
                float localPerPixelX = max(length(float2(ddx(input.localPosition.x), ddy(input.localPosition.x))), 1e-7);
                float localPerPixelY = max(length(float2(ddx(input.localPosition.y), ddy(input.localPosition.y))), 1e-7);
                float coordinatePerPixelX = max(length(float2(ddx(coordinateX), ddy(coordinateX))), 1e-7);
                float coordinatePerPixelY = max(length(float2(ddx(coordinateY), ddy(coordinateY))), 1e-7);

                float verticalMajor = IsMajorLine(_GridPhase.z, nearestX, subdivisions);
                float horizontalMajor = IsMajorLine(_GridPhase.w, nearestY, subdivisions);

                float verticalCoverage = validX * LineCoverage(
                    coordinateX, nearestX, coordinatePerPixelX, localPerPixelX,
                    lerp(_GridMinorThickness, _GridMajorThickness, verticalMajor));
                float horizontalCoverage = validY * LineCoverage(
                    coordinateY, nearestY, coordinatePerPixelY, localPerPixelY,
                    lerp(_GridMinorThickness, _GridMajorThickness, horizontalMajor));

                // The original implementation created all vertical lines first and
                // all horizontal lines second. Composite horizontal lines over vertical
                // lines to preserve their intersection ordering and alpha behavior.
                float4 verticalColor = lerp(_GridMinorColor, _GridMajorColor, verticalMajor) * input.color;
                float4 horizontalColor = lerp(_GridMinorColor, _GridMajorColor, horizontalMajor) * input.color;
                float verticalAlpha = saturate(verticalColor.a * verticalCoverage);
                float horizontalAlpha = saturate(horizontalColor.a * horizontalCoverage);

                float outputAlpha = horizontalAlpha + verticalAlpha * (1.0 - horizontalAlpha);
                float3 outputPremultiplied =
                    horizontalColor.rgb * horizontalAlpha +
                    verticalColor.rgb * verticalAlpha * (1.0 - horizontalAlpha);
                float3 outputRGB = outputPremultiplied / max(outputAlpha, 1e-6);

                fixed4 color = fixed4(outputRGB, outputAlpha);

                #ifdef UNITY_UI_CLIP_RECT
                    half2 maskCoverage = saturate(
                        (_ClipRect.zw - _ClipRect.xy - abs(input.mask.xy)) * input.mask.zw);
                    color.a *= maskCoverage.x * maskCoverage.y;
                #endif

                #ifdef UNITY_UI_ALPHACLIP
                    clip(color.a - 0.001);
                #endif

                return color;
            }
            ENDCG
        }
    }
}
