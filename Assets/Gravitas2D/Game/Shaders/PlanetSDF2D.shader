Shader "Gravitas2D/PlanetSDF2D"
{
    Properties
    {
        _MainTex ("Sprite Texture", 2D) = "white" {}
        _OceanColor ("Deep Ocean Color", Color) = (0.008, 0.035, 0.095, 1)
        _LandColor ("Vegetation Land Color", Color) = (0.06, 0.22, 0.08, 1)
        _DesertColor ("Arid Desert Color", Color) = (0.35, 0.28, 0.16, 1)
        _CityLightsColor ("City Lights Emission Color", Color) = (1.0, 0.75, 0.35, 1)
        _RayleighColor ("Rayleigh Scattering (Blue Sky)", Color) = (0.15, 0.42, 0.92, 1)
        _MiegColor ("Mie Aerosol Glare", Color) = (0.85, 0.94, 1.0, 1)
        _SunsetColor ("Sunset Terminator Glow", Color) = (1.0, 0.38, 0.08, 1)
        _PlanetRadiusRatio ("Planet Radius Ratio", Range(0.1, 0.49)) = 0.3846154
        _AtmosphereThickness ("Atmosphere Thickness", Range(0.005, 0.1)) = 0.03
        _RayleighDensity ("Rayleigh Optical Density", Range(0.1, 20.0)) = 10.0
        _MieDensity ("Mie Aerosol Density", Range(0.1, 5.0)) = 0.8
        _LightDirection ("Light Direction Vector", Vector) = (1, 0, 0, 0)
        _PlanetRotation ("Planet Rotation Angle (Radians)", Float) = 0
    }
    SubShader
    {
        Tags 
        { 
            "Queue"="Transparent" 
            "IgnoreProjector"="True" 
            "RenderType"="Transparent" 
            "PreviewType"="Plane"
        }

        Cull Off
        Lighting Off
        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float2 uv : TEXCOORD0;
            };

            fixed4 _OceanColor;
            fixed4 _LandColor;
            fixed4 _DesertColor;
            fixed4 _CityLightsColor;
            fixed4 _RayleighColor;
            fixed4 _MiegColor;
            fixed4 _SunsetColor;

            float _PlanetRadiusRatio;
            float _AtmosphereThickness;
            float _RayleighDensity;
            float _MieDensity;
            float4 _LightDirection;
            float _PlanetRotation;

            v2f vert(appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv - 0.5; // Center UV at (0,0)
                return o;
            }

            float Hash(float2 p) 
            {
                return frac(sin(dot(p, float2(127.1, 311.7))) * 43758.5453123);
            }

            float Noise(float2 p) 
            {
                float2 i = floor(p);
                float2 f = frac(p);
                f = f * f * (3.0 - 2.0 * f);
                float mix1 = lerp(Hash(i + float2(0,0)), Hash(i + float2(1,0)), f.x);
                float mix2 = lerp(Hash(i + float2(0,1)), Hash(i + float2(1,1)), f.x);
                return lerp(mix1, mix2, f.y);
            }

            float Fbm(float2 p) 
            {
                float v = 0.0;
                float a = 0.5;
                for (int i = 0; i < 4; i++) 
                {
                    v += a * Noise(p);
                    p = p * 2.1;
                    a *= 0.5;
                }
                return v;
            }

            fixed4 frag(v2f i) : SV_Target
            {
                float2 uv = i.uv;
                float r = length(uv);

                float R_crust = _PlanetRadiusRatio;
                float R_top = R_crust + _AtmosphereThickness;

                if (r > R_top) discard;

                float2 lightDir = normalize(_LightDirection.xy);

                // 1. Crust Surface & Procedural Continents
                float delta = fwidth(r);
                float crustAlpha = 1.0 - smoothstep(R_crust - delta, R_crust + delta, r);

                float3 normal;
                if (r <= R_crust)
                {
                    float2 sphereUV = uv / R_crust;
                    normal = float3(sphereUV, sqrt(max(0.0, 1.0 - dot(sphereUV, sphereUV))));
                }
                else
                {
                    float2 shellUV = uv / R_top;
                    normal = float3(shellUV, sqrt(max(0.0, 1.0 - dot(shellUV, shellUV))));
                }

                // Lighting calculated in world space
                float NdotL = dot(normal.xy, lightDir);
                float dayFactor = smoothstep(-0.05, 0.05, NdotL);

                // Rotate surface features independently using _PlanetRotation
                float cosR = cos(_PlanetRotation);
                float sinR = sin(_PlanetRotation);
                float2 crustUV = uv / R_crust;
                float2 rotCrustUV = float2(cosR * crustUV.x - sinR * crustUV.y, sinR * crustUV.x + cosR * crustUV.y);

                float elevation = Fbm(rotCrustUV * 3.2);
                float continent = smoothstep(0.48, 0.52, elevation);
                float desert = smoothstep(0.7, 0.88, elevation);
                float3 surfaceBase = lerp(_OceanColor.rgb, lerp(_LandColor.rgb, _DesertColor.rgb, desert), continent);

                // Day surface lighting + Ocean specular glint
                float3 daySurface = surfaceBase * max(0.0, NdotL);
                float3 viewDir = float3(0, 0, 1);
                float3 halfDir = normalize(float3(lightDir, 0.5) + viewDir);
                float specular = pow(max(0.0, dot(normal, halfDir)), 48.0) * (1.0 - continent) * max(0.0, NdotL);
                daySurface += specular * 0.4;

                // Night side: Clustered metropolitan city lights
                float nightMask = smoothstep(0.0, -0.25, NdotL);
                float cityClusterNoise = Fbm(rotCrustUV * 14.0);
                float cityMask = smoothstep(0.68, 0.82, cityClusterNoise) * smoothstep(0.49, 0.54, elevation);
                float3 cityLights = _CityLightsColor.rgb * cityMask * nightMask * 3.0;

                float3 surfaceColor = lerp(cityLights, daySurface, dayFactor);

                // 2. Physical Atmospheric Optical Path & Scattering Rim
                float height = max(0.0, r - R_crust);
                float density = exp(-height / max(1e-4, _AtmosphereThickness * 0.4));
                
                float pathLength = (r < R_crust) 
                    ? sqrt(max(0.0, R_top * R_top - r * r)) - sqrt(max(0.0, R_crust * R_crust - r * r))
                    : 2.0 * sqrt(max(0.0, R_top * R_top - r * r));

                float opticalDepth = pathLength * density;

                float sunVisibility = smoothstep(-0.15, 0.05, dot(normal.xy, lightDir));

                float cosTheta = dot(normal.xy, lightDir);
                float phaseRayleigh = 0.75 * (1.0 + cosTheta * cosTheta);
                float g = 0.82;
                float phaseMie = ((1.0 - g * g) / pow(max(1e-4, 1.0 + g * g - 2.0 * g * cosTheta), 1.5)) * 0.25;

                float3 rayleighScattering = _RayleighColor.rgb * _RayleighDensity * phaseRayleigh;
                float3 mieScattering = _MiegColor.rgb * _MieDensity * phaseMie;

                float sunsetBand = exp(-pow(NdotL * 3.5, 2.0)) * smoothstep(-0.35, 0.15, NdotL);
                float3 sunsetScattering = _SunsetColor.rgb * sunsetBand * 2.5;

                float3 skyRadiance = (rayleighScattering + mieScattering + sunsetScattering) * opticalDepth * sunVisibility;

                // 3. Surface Transmittance & Atmospheric Overlay
                float3 crustTransmittance = exp(-opticalDepth * _RayleighColor.rgb * _RayleighDensity * 0.4);
                float3 finalSurfacePass = (surfaceColor * crustTransmittance) + (skyRadiance * 0.65);

                // 4. Outer Atmosphere Limb Alpha & Blending
                float limbAlpha = saturate(1.0 - exp(-opticalDepth * (_RayleighDensity + _MieDensity) * 0.4));
                limbAlpha *= smoothstep(R_top, R_top - (_AtmosphereThickness * 0.3), r) * sunVisibility;

                float3 finalColor = lerp(skyRadiance, finalSurfacePass, crustAlpha);
                float finalAlpha = max(crustAlpha, limbAlpha);

                finalColor *= smoothstep(R_top, R_top - 0.002, r);

                return fixed4(finalColor, finalAlpha);
            }
            ENDHLSL
        }
    }
}