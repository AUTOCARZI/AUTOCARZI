Shader "Ciconia Studio/CS_Wet Animation/URP/Rainy Puddles/Pro"
{
	Properties
	{
		[HideInInspector] _AlphaCutoff("Alpha Cutoff ", Range(0, 1)) = 0.5
		[Space(35)][Header(Main Properties________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________)][Space(15)]_GlobalXYTilingXYZWOffsetXY("Global --> XY(TilingXY) - ZW(OffsetXY)", Vector) = (1,1,0,0)
		_Color("Color", Color) = (1,1,1,0)
		[Toggle]_InvertABaseColor("Invert Alpha", Float) = 0
		_BaseMap("Base Color  -->(Mask A)", 2D) = "white" {}
		_Saturation("Saturation", Float) = 0
		_Brightness("Brightness", Range( 1 , 8)) = 1
		[Space(35)]_BumpMap("Normal Map", 2D) = "bump" {}
		_BumpScale("Normal Intensity", Float) = 0.3
		[Space(35)]_MetallicGlossMapMAHS("Mask Map  -->M(R) - Ao(G) - H(B) - S(A)", 2D) = "white" {}
		_Metallic("Metallic", Range( 0 , 2)) = 0
		_Smoothness("Smoothness", Range( 0 , 2)) = 0.5
		[Space(15)]_Parallax("Height Scale", Range( -0.1 , 0.1)) = 0
		_AoIntensity("Ao Intensity", Range( 0 , 2)) = 0
		[HDR][Space(45)]_EmissionColor("Emission Color", Color) = (0,0,0,0)
		_EmissionMap("Emission Map", 2D) = "white" {}
		_EmissionIntensity("Intensity", Range( 0 , 2)) = 1
		[Space(35)][Header(Rain Dots Properties________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________)][Space(15)]_GradientTex("Gradient Tex", 2D) = "white" {}
		_Intensity("Intensity", Range( 0 , 1)) = 0
		_Tiling("Tiling", Float) = 100
		_SplashSpeed("Splash Speed", Range( 0 , 1)) = 0.1
		_Size("Size", Range( 0 , 1)) = 0.5
		[Space(35)][Header(Mask Properties________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________)][Space(15)][Toggle]_VisualizeMask("Visualize Mask", Float) = 0
		[Toggle]_ExcludeMask("Exclude - Use BaseColorAlpha", Float) = 0
		[Space(10)][KeywordEnum(DetailMask,BaseColorAlpha,Both)] _SourceMask("Source", Float) = 0
		[Toggle]_InvertMask("Invert Mask", Float) = 0
		[Space(15)][KeywordEnum(Red,Green,Blue,Alpha)] _ChannelSelectionMask("Channel Selection", Float) = 0
		_DetailMask("Detail Mask", 2D) = "black" {}
		_IntensityMask("Intensity", Range( 0 , 1)) = 1
		[Space(15)]_ContrastDetailMap("Contrast", Float) = 0
		_SpreadDetailMap("Spread", Range( 0 , 1)) = 0.5
		[Space(45)][Header(Reflection Properties________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________)][Space(15)]_ColorCubemap1("Color ", Color) = (1,1,1,1)
		[HDR]_Cubemap("Cubemap", CUBE) = "black" {}
		[Space(10)]_ReflectionIntensity("Intensity", Range( 0 , 10)) = 0.2
		_BlurReflection("Blur", Range( 0 , 7)) = 0.5
		[Toggle]_UseMainNormalMapasNormalDirection("Use Main Normal Map as Normal Direction", Float) = 1
		[Space(35)][Header(Puddle Properties________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________)][Space(15)]_DetailColor("Color", Color) = (0.5019608,0.5019608,0.5019608,0)
		[Toggle(_USEMAINALBEDOMAP_ON)] _UseMainAlbedoMap("Use Main Albedo Map", Float) = 1
		_DetailAlbedoMap("Base Color", 2D) = "white" {}
		_DetailSaturation("Saturation", Float) = 0
		_DetailBrightness("Brightness", Range( 1 , 8)) = 1
		[Space(35)][Toggle]_BlendMainNormal("Blend Main Normal", Float) = 0
		_DetailNormalMap("Wave Normal Map", 2D) = "bump" {}
		[Space(15)][Toggle(_MAINWAVE_ON)] _MainWave("Main Wave", Float) = 1
		_NormalWaveIntensity1("Intensity", Float) = 0.5
		_TranslationSpeed1("Speed", Float) = 1
		_RotationAngle1("Rotation", Float) = 0
		_TilingWave1("Tiling", Float) = 1
		[Space(25)][Toggle(_DETAILWAVE_ON)] _DetailWave("Detail Wave", Float) = 1
		_NormalWaveIntensity2("Intensity", Float) = 0.3
		_TranslationSpeed2("Speed", Float) = 1
		_RotationAngle2("Rotation", Float) = 90
		_TilingWave2("Tiling", Float) = 1
		[Space(15)]_Metallic2("Metallic", Range( 0 , 2)) = 0
		_Glossiness2("Smoothness", Range( 0 , 2)) = 0.95
		[Space(35)][Header(Rain Ripples Properties________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________)][Space(15)]_XColumnsYRowsZSpeedWStrartFrameNormal("X(Columns) - Y(Rows) - Z(Speed) - W(Strart Frame)Normal", Vector) = (8,8,0.25,0)
		_TextureAtlasNormal("Texture Atlas Normal", 2D) = "bump" {}
		_FlipBTilingNormal("FlipBook Tiling", Float) = 1
		_IntensityScaleNormal1("Instensity", Range( 0 , 1)) = 0.4
		[Space(35)][Toggle]_DuplicateTextureAtlasNormal("Duplicate Texture Atlas", Float) = 1
		_IntensityScaleNormal2("Instensity", Range( 0 , 1)) = 0.3
		_ScaleFBDetailsNormal("Scale", Float) = 1
		_FBDetailsNormal("Rotate Details", Float) = 45
		_OffsetFBDetailsNormal("Offset XY", Vector) = (1.5,1.5,0,0)
		[Space(10)]_Distortion("Distortion", Range( 0 , 1)) = 0
		[Space(15)][Toggle]_UseAoFromMainProperties("Use Ao From Main Properties", Float) = 1
		[Toggle]_UseEmissionFromMainProperties("Use Emission From Main Properties", Float) = 1

		//_TransmissionShadow( "Transmission Shadow", Range( 0, 1 ) ) = 0.5
		//_TransStrength( "Trans Strength", Range( 0, 50 ) ) = 1
		//_TransNormal( "Trans Normal Distortion", Range( 0, 1 ) ) = 0.5
		//_TransScattering( "Trans Scattering", Range( 1, 50 ) ) = 2
		//_TransDirect( "Trans Direct", Range( 0, 1 ) ) = 0.9
		//_TransAmbient( "Trans Ambient", Range( 0, 1 ) ) = 0.1
		//_TransShadow( "Trans Shadow", Range( 0, 1 ) ) = 0.5
		//_TessPhongStrength( "Tess Phong Strength", Range( 0, 1 ) ) = 0.5
		//_TessValue( "Tess Max Tessellation", Range( 1, 32 ) ) = 16
		//_TessMin( "Tess Min Distance", Float ) = 10
		//_TessMax( "Tess Max Distance", Float ) = 25
		//_TessEdgeLength ( "Tess Edge length", Range( 2, 50 ) ) = 16
		//_TessMaxDisp( "Tess Max Displacement", Float ) = 25
	}

	SubShader
	{
		LOD 0

		

		Tags { "RenderPipeline"="UniversalPipeline" "RenderType"="Opaque" "Queue"="Geometry" }
		Cull Back
		AlphaToMask Off
		HLSLINCLUDE
		#pragma target 2.0

		#pragma prefer_hlslcc gles
		#pragma exclude_renderers d3d11_9x 


		#ifndef ASE_TESS_FUNCS
		#define ASE_TESS_FUNCS
		float4 FixedTess( float tessValue )
		{
			return tessValue;
		}
		
		float CalcDistanceTessFactor (float4 vertex, float minDist, float maxDist, float tess, float4x4 o2w, float3 cameraPos )
		{
			float3 wpos = mul(o2w,vertex).xyz;
			float dist = distance (wpos, cameraPos);
			float f = clamp(1.0 - (dist - minDist) / (maxDist - minDist), 0.01, 1.0) * tess;
			return f;
		}

		float4 CalcTriEdgeTessFactors (float3 triVertexFactors)
		{
			float4 tess;
			tess.x = 0.5 * (triVertexFactors.y + triVertexFactors.z);
			tess.y = 0.5 * (triVertexFactors.x + triVertexFactors.z);
			tess.z = 0.5 * (triVertexFactors.x + triVertexFactors.y);
			tess.w = (triVertexFactors.x + triVertexFactors.y + triVertexFactors.z) / 3.0f;
			return tess;
		}

		float CalcEdgeTessFactor (float3 wpos0, float3 wpos1, float edgeLen, float3 cameraPos, float4 scParams )
		{
			float dist = distance (0.5 * (wpos0+wpos1), cameraPos);
			float len = distance(wpos0, wpos1);
			float f = max(len * scParams.y / (edgeLen * dist), 1.0);
			return f;
		}

		float DistanceFromPlane (float3 pos, float4 plane)
		{
			float d = dot (float4(pos,1.0f), plane);
			return d;
		}

		bool WorldViewFrustumCull (float3 wpos0, float3 wpos1, float3 wpos2, float cullEps, float4 planes[6] )
		{
			float4 planeTest;
			planeTest.x = (( DistanceFromPlane(wpos0, planes[0]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[0]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[0]) > -cullEps) ? 1.0f : 0.0f );
			planeTest.y = (( DistanceFromPlane(wpos0, planes[1]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[1]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[1]) > -cullEps) ? 1.0f : 0.0f );
			planeTest.z = (( DistanceFromPlane(wpos0, planes[2]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[2]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[2]) > -cullEps) ? 1.0f : 0.0f );
			planeTest.w = (( DistanceFromPlane(wpos0, planes[3]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos1, planes[3]) > -cullEps) ? 1.0f : 0.0f ) +
						  (( DistanceFromPlane(wpos2, planes[3]) > -cullEps) ? 1.0f : 0.0f );
			return !all (planeTest);
		}

		float4 DistanceBasedTess( float4 v0, float4 v1, float4 v2, float tess, float minDist, float maxDist, float4x4 o2w, float3 cameraPos )
		{
			float3 f;
			f.x = CalcDistanceTessFactor (v0,minDist,maxDist,tess,o2w,cameraPos);
			f.y = CalcDistanceTessFactor (v1,minDist,maxDist,tess,o2w,cameraPos);
			f.z = CalcDistanceTessFactor (v2,minDist,maxDist,tess,o2w,cameraPos);

			return CalcTriEdgeTessFactors (f);
		}

		float4 EdgeLengthBasedTess( float4 v0, float4 v1, float4 v2, float edgeLength, float4x4 o2w, float3 cameraPos, float4 scParams )
		{
			float3 pos0 = mul(o2w,v0).xyz;
			float3 pos1 = mul(o2w,v1).xyz;
			float3 pos2 = mul(o2w,v2).xyz;
			float4 tess;
			tess.x = CalcEdgeTessFactor (pos1, pos2, edgeLength, cameraPos, scParams);
			tess.y = CalcEdgeTessFactor (pos2, pos0, edgeLength, cameraPos, scParams);
			tess.z = CalcEdgeTessFactor (pos0, pos1, edgeLength, cameraPos, scParams);
			tess.w = (tess.x + tess.y + tess.z) / 3.0f;
			return tess;
		}

		float4 EdgeLengthBasedTessCull( float4 v0, float4 v1, float4 v2, float edgeLength, float maxDisplacement, float4x4 o2w, float3 cameraPos, float4 scParams, float4 planes[6] )
		{
			float3 pos0 = mul(o2w,v0).xyz;
			float3 pos1 = mul(o2w,v1).xyz;
			float3 pos2 = mul(o2w,v2).xyz;
			float4 tess;

			if (WorldViewFrustumCull(pos0, pos1, pos2, maxDisplacement, planes))
			{
				tess = 0.0f;
			}
			else
			{
				tess.x = CalcEdgeTessFactor (pos1, pos2, edgeLength, cameraPos, scParams);
				tess.y = CalcEdgeTessFactor (pos2, pos0, edgeLength, cameraPos, scParams);
				tess.z = CalcEdgeTessFactor (pos0, pos1, edgeLength, cameraPos, scParams);
				tess.w = (tess.x + tess.y + tess.z) / 3.0f;
			}
			return tess;
		}
		#endif //ASE_TESS_FUNCS
		ENDHLSL

		
		Pass
		{
			
			Name "Forward"
			Tags { "LightMode"="UniversalForward" }
			
			Blend One Zero, One Zero
			ZWrite On
			ZTest LEqual
			Offset 0 , 0
			ColorMask RGBA
			

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile_instancing
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define _EMISSION
			#define _NORMALMAP 1
			#define ASE_SRP_VERSION 70301

			
			#pragma multi_compile _ _MAIN_LIGHT_SHADOWS
			#pragma multi_compile _ _MAIN_LIGHT_SHADOWS_CASCADE
			#pragma multi_compile _ _ADDITIONAL_LIGHTS_VERTEX _ADDITIONAL_LIGHTS
			#pragma multi_compile _ _ADDITIONAL_LIGHT_SHADOWS
			#pragma multi_compile _ _SHADOWS_SOFT
			#pragma multi_compile _ _MIXED_LIGHTING_SUBTRACTIVE
			
			#pragma multi_compile _ DIRLIGHTMAP_COMBINED
			#pragma multi_compile _ LIGHTMAP_ON

			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_FORWARD

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			
			#if ASE_SRP_VERSION <= 70108
			#define REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR
			#endif

			#if defined(UNITY_INSTANCING_ENABLED) && defined(_TERRAIN_INSTANCED_PERPIXEL_NORMAL)
			    #define ENABLE_TERRAIN_PERPIXEL_NORMAL
			#endif

			#define ASE_NEEDS_FRAG_WORLD_TANGENT
			#define ASE_NEEDS_FRAG_WORLD_NORMAL
			#define ASE_NEEDS_FRAG_WORLD_BITANGENT
			#define ASE_NEEDS_FRAG_WORLD_VIEW_DIR
			#pragma shader_feature_local _USEMAINALBEDOMAP_ON
			#pragma shader_feature_local _SOURCEMASK_DETAILMASK _SOURCEMASK_BASECOLORALPHA _SOURCEMASK_BOTH
			#pragma shader_feature_local _CHANNELSELECTIONMASK_RED _CHANNELSELECTIONMASK_GREEN _CHANNELSELECTIONMASK_BLUE _CHANNELSELECTIONMASK_ALPHA
			#pragma shader_feature_local _DETAILWAVE_ON
			#pragma shader_feature_local _MAINWAVE_ON


			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_tangent : TANGENT;
				float4 texcoord1 : TEXCOORD1;
				float4 texcoord : TEXCOORD0;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				float4 lightmapUVOrVertexSH : TEXCOORD0;
				half4 fogFactorAndVertexLight : TEXCOORD1;
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
				float4 shadowCoord : TEXCOORD2;
				#endif
				float4 tSpace0 : TEXCOORD3;
				float4 tSpace1 : TEXCOORD4;
				float4 tSpace2 : TEXCOORD5;
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				float4 screenPos : TEXCOORD6;
				#endif
				float4 ase_texcoord7 : TEXCOORD7;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _DetailAlbedoMap_ST;
			float4 _EmissionColor;
			float4 _Color;
			float4 _BaseMap_ST;
			float4 _GlobalXYTilingXYZWOffsetXY;
			float4 _MetallicGlossMapMAHS_ST;
			float4 _ColorCubemap1;
			float4 _BumpMap_ST;
			float4 _DetailNormalMap_ST;
			float4 _DetailColor;
			float4 _EmissionMap_ST;
			float4 _DetailMask_ST;
			float4 _XColumnsYRowsZSpeedWStrartFrameNormal;
			float2 _OffsetFBDetailsNormal;
			float _UseEmissionFromMainProperties;
			float _RotationAngle2;
			float _TilingWave2;
			float _TranslationSpeed2;
			float _NormalWaveIntensity1;
			float _NormalWaveIntensity2;
			float _EmissionIntensity;
			float _VisualizeMask;
			float _BlurReflection;
			float _ReflectionIntensity;
			float _Metallic;
			float _Metallic2;
			float _Smoothness;
			float _SplashSpeed;
			float _Tiling;
			float _Size;
			float _Intensity;
			float _Glossiness2;
			float _UseMainNormalMapasNormalDirection;
			float _RotationAngle1;
			float _BlendMainNormal;
			float _TranslationSpeed1;
			float _Brightness;
			float _Parallax;
			float _Saturation;
			float _DetailBrightness;
			float _DuplicateTextureAtlasNormal;
			float _FlipBTilingNormal;
			float _IntensityScaleNormal1;
			float _ScaleFBDetailsNormal;
			float _FBDetailsNormal;
			float _TilingWave1;
			float _IntensityScaleNormal2;
			float _DetailSaturation;
			float _ExcludeMask;
			float _ContrastDetailMap;
			float _InvertMask;
			float _InvertABaseColor;
			float _SpreadDetailMap;
			float _IntensityMask;
			float _UseAoFromMainProperties;
			float _BumpScale;
			float _Distortion;
			float _AoIntensity;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			sampler2D _BaseMap;
			sampler2D _MetallicGlossMapMAHS;
			sampler2D _DetailAlbedoMap;
			sampler2D _TextureAtlasNormal;
			sampler2D _DetailMask;
			sampler2D _BumpMap;
			sampler2D _DetailNormalMap;
			sampler2D _EmissionMap;
			samplerCUBE _Cubemap;
			sampler2D _GradientTex;


			inline float2 ParallaxOffset( half h, half height, half3 viewDir )
			{
				h = h * height - height/2.0;
				float3 v = normalize( viewDir );
				v.z += 0.42;
				return h* (v.xy / v.z);
			}
			
			float4 CalculateContrast( float contrastValue, float4 colorTarget )
			{
				float t = 0.5 * ( 1.0 - contrastValue );
				return mul( float4x4( contrastValue,0,0,t, 0,contrastValue,0,t, 0,0,contrastValue,t, 0,0,0,1 ), colorTarget );
			}
					float2 voronoihash58_g887( float2 p )
					{
						
						p = float2( dot( p, float2( 127.1, 311.7 ) ), dot( p, float2( 269.5, 183.3 ) ) );
						return frac( sin( p ) *43758.5453);
					}
			
					float voronoi58_g887( float2 v, float time, inout float2 id, inout float2 mr, float smoothness, inout float2 smoothId )
					{
						float2 n = floor( v );
						float2 f = frac( v );
						float F1 = 8.0;
						float F2 = 8.0; float2 mg = 0;
						for ( int j = -1; j <= 1; j++ )
						{
							for ( int i = -1; i <= 1; i++ )
						 	{
						 		float2 g = float2( i, j );
						 		float2 o = voronoihash58_g887( n + g );
								o = ( sin( time + o * 6.2831 ) * 0.5 + 0.5 ); float2 r = f - g - o;
								float d = 0.5 * dot( r, r );
						 		if( d<F1 ) {
						 			F2 = F1;
						 			F1 = d; mg = g; mr = r; id = o;
						 		} else if( d<F2 ) {
						 			F2 = d;
						
						 		}
						 	}
						}
						return F1;
					}
			

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				o.ase_texcoord7.xyz = v.texcoord.xyz;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord7.w = 0;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif
				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float3 positionVS = TransformWorldToView( positionWS );
				float4 positionCS = TransformWorldToHClip( positionWS );

				VertexNormalInputs normalInput = GetVertexNormalInputs( v.ase_normal, v.ase_tangent );

				o.tSpace0 = float4( normalInput.normalWS, positionWS.x);
				o.tSpace1 = float4( normalInput.tangentWS, positionWS.y);
				o.tSpace2 = float4( normalInput.bitangentWS, positionWS.z);

				OUTPUT_LIGHTMAP_UV( v.texcoord1, unity_LightmapST, o.lightmapUVOrVertexSH.xy );
				OUTPUT_SH( normalInput.normalWS.xyz, o.lightmapUVOrVertexSH.xyz );

				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					o.lightmapUVOrVertexSH.zw = v.texcoord;
					o.lightmapUVOrVertexSH.xy = v.texcoord * unity_LightmapST.xy + unity_LightmapST.zw;
				#endif

				half3 vertexLight = VertexLighting( positionWS, normalInput.normalWS );
				#ifdef ASE_FOG
					half fogFactor = ComputeFogFactor( positionCS.z );
				#else
					half fogFactor = 0;
				#endif
				o.fogFactorAndVertexLight = half4(fogFactor, vertexLight);
				
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
				VertexPositionInputs vertexInput = (VertexPositionInputs)0;
				vertexInput.positionWS = positionWS;
				vertexInput.positionCS = positionCS;
				o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				
				o.clipPos = positionCS;
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				o.screenPos = ComputeScreenPos(positionCS);
				#endif
				return o;
			}
			
			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_tangent : TANGENT;
				float4 texcoord : TEXCOORD0;
				float4 texcoord1 : TEXCOORD1;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_tangent = v.ase_tangent;
				o.texcoord = v.texcoord;
				o.texcoord1 = v.texcoord1;
				
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_tangent = patch[0].ase_tangent * bary.x + patch[1].ase_tangent * bary.y + patch[2].ase_tangent * bary.z;
				o.texcoord = patch[0].texcoord * bary.x + patch[1].texcoord * bary.y + patch[2].texcoord * bary.z;
				o.texcoord1 = patch[0].texcoord1 * bary.x + patch[1].texcoord1 * bary.y + patch[2].texcoord1 * bary.z;
				
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif

			half4 frag ( VertexOutput IN 
						#ifdef ASE_DEPTH_WRITE_ON
						,out float outputDepth : ASE_SV_DEPTH
						#endif
						, FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_Target
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(IN);

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif

				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					float2 sampleCoords = (IN.lightmapUVOrVertexSH.zw / _TerrainHeightmapRecipSize.zw + 0.5f) * _TerrainHeightmapRecipSize.xy;
					float3 WorldNormal = TransformObjectToWorldNormal(normalize(SAMPLE_TEXTURE2D(_TerrainNormalmapTexture, sampler_TerrainNormalmapTexture, sampleCoords).rgb * 2 - 1));
					float3 WorldTangent = -cross(GetObjectToWorldMatrix()._13_23_33, WorldNormal);
					float3 WorldBiTangent = cross(WorldNormal, -WorldTangent);
				#else
					float3 WorldNormal = normalize( IN.tSpace0.xyz );
					float3 WorldTangent = IN.tSpace1.xyz;
					float3 WorldBiTangent = IN.tSpace2.xyz;
				#endif
				float3 WorldPosition = float3(IN.tSpace0.w,IN.tSpace1.w,IN.tSpace2.w);
				float3 WorldViewDirection = _WorldSpaceCameraPos.xyz  - WorldPosition;
				float4 ShadowCoords = float4( 0, 0, 0, 0 );
				#if defined(ASE_NEEDS_FRAG_SCREEN_POSITION)
				float4 ScreenPos = IN.screenPos;
				#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
					ShadowCoords = IN.shadowCoord;
				#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
					ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
				#endif
	
				WorldViewDirection = SafeNormalize( WorldViewDirection );

				float2 uv_BaseMap = IN.ase_texcoord7.xyz.xy * _BaseMap_ST.xy + _BaseMap_ST.zw;
				float2 break26_g854 = uv_BaseMap;
				float GlobalTilingX11 = ( _GlobalXYTilingXYZWOffsetXY.x - 1.0 );
				float GlobalTilingY8 = ( _GlobalXYTilingXYZWOffsetXY.y - 1.0 );
				float2 appendResult14_g854 = (float2(( break26_g854.x * GlobalTilingX11 ) , ( break26_g854.y * GlobalTilingY8 )));
				float GlobalOffsetX10 = _GlobalXYTilingXYZWOffsetXY.z;
				float GlobalOffsetY9 = _GlobalXYTilingXYZWOffsetXY.w;
				float2 appendResult13_g854 = (float2(( break26_g854.x + GlobalOffsetX10 ) , ( break26_g854.y + GlobalOffsetY9 )));
				float2 uv_MetallicGlossMapMAHS = IN.ase_texcoord7.xyz.xy * _MetallicGlossMapMAHS_ST.xy + _MetallicGlossMapMAHS_ST.zw;
				float2 break26_g831 = uv_MetallicGlossMapMAHS;
				float2 appendResult14_g831 = (float2(( break26_g831.x * GlobalTilingX11 ) , ( break26_g831.y * GlobalTilingY8 )));
				float2 appendResult13_g831 = (float2(( break26_g831.x + GlobalOffsetX10 ) , ( break26_g831.y + GlobalOffsetY9 )));
				float4 tex2DNode3_g830 = tex2D( _MetallicGlossMapMAHS, ( ( appendResult14_g831 + appendResult13_g831 ) + float2( 0,0 ) ) );
				float3 tanToWorld0 = float3( WorldTangent.x, WorldBiTangent.x, WorldNormal.x );
				float3 tanToWorld1 = float3( WorldTangent.y, WorldBiTangent.y, WorldNormal.y );
				float3 tanToWorld2 = float3( WorldTangent.z, WorldBiTangent.z, WorldNormal.z );
				float3 ase_tanViewDir =  tanToWorld0 * WorldViewDirection.x + tanToWorld1 * WorldViewDirection.y  + tanToWorld2 * WorldViewDirection.z;
				ase_tanViewDir = normalize(ase_tanViewDir);
				float2 paralaxOffset38_g830 = ParallaxOffset( tex2DNode3_g830.b , _Parallax , ase_tanViewDir );
				float2 switchResult37_g830 = (((ase_vface>0)?(paralaxOffset38_g830):(0.0)));
				float2 Parallaxe494 = switchResult37_g830;
				float4 tex2DNode7_g853 = tex2D( _BaseMap, ( ( appendResult14_g854 + appendResult13_g854 ) + Parallaxe494 ) );
				float clampResult27_g853 = clamp( _Saturation , -1.0 , 100.0 );
				float3 desaturateInitialColor29_g853 = ( _Color * tex2DNode7_g853 ).rgb;
				float desaturateDot29_g853 = dot( desaturateInitialColor29_g853, float3( 0.299, 0.587, 0.114 ));
				float3 desaturateVar29_g853 = lerp( desaturateInitialColor29_g853, desaturateDot29_g853.xxx, -clampResult27_g853 );
				float4 temp_output_764_0 = CalculateContrast(_Brightness,float4( desaturateVar29_g853 , 0.0 ));
				float2 uv_DetailAlbedoMap = IN.ase_texcoord7.xyz.xy * _DetailAlbedoMap_ST.xy + _DetailAlbedoMap_ST.zw;
				float FlipBookTiling100_g852 = _FlipBTilingNormal;
				float2 temp_cast_2 = (FlipBookTiling100_g852).xx;
				float2 texCoord114_g852 = IN.ase_texcoord7.xyz.xy * temp_cast_2 + float2( 0,0 );
				float2 appendResult124_g852 = (float2(frac( texCoord114_g852.x ) , frac( texCoord114_g852.y )));
				float XColumns119_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.x;
				float YRows118_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.y;
				float ZSpeed121_g852 = ( ( _XColumnsYRowsZSpeedWStrartFrameNormal.x * _XColumnsYRowsZSpeedWStrartFrameNormal.y ) * _XColumnsYRowsZSpeedWStrartFrameNormal.z );
				float WStrartFrame122_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.w;
				// *** BEGIN Flipbook UV Animation vars ***
				// Total tiles of Flipbook Texture
				float fbtotaltiles130_g852 = XColumns119_g852 * YRows118_g852;
				// Offsets for cols and rows of Flipbook Texture
				float fbcolsoffset130_g852 = 1.0f / XColumns119_g852;
				float fbrowsoffset130_g852 = 1.0f / YRows118_g852;
				// Speed of animation
				float fbspeed130_g852 = _Time[ 1 ] * ZSpeed121_g852;
				// UV Tiling (col and row offset)
				float2 fbtiling130_g852 = float2(fbcolsoffset130_g852, fbrowsoffset130_g852);
				// UV Offset - calculate current tile linear index, and convert it to (X * coloffset, Y * rowoffset)
				// Calculate current tile linear index
				float fbcurrenttileindex130_g852 = round( fmod( fbspeed130_g852 + WStrartFrame122_g852, fbtotaltiles130_g852) );
				fbcurrenttileindex130_g852 += ( fbcurrenttileindex130_g852 < 0) ? fbtotaltiles130_g852 : 0;
				// Obtain Offset X coordinate from current tile linear index
				float fblinearindextox130_g852 = round ( fmod ( fbcurrenttileindex130_g852, XColumns119_g852 ) );
				// Multiply Offset X by coloffset
				float fboffsetx130_g852 = fblinearindextox130_g852 * fbcolsoffset130_g852;
				// Obtain Offset Y coordinate from current tile linear index
				float fblinearindextoy130_g852 = round( fmod( ( fbcurrenttileindex130_g852 - fblinearindextox130_g852 ) / XColumns119_g852, YRows118_g852 ) );
				// Reverse Y to get tiles from Top to Bottom
				fblinearindextoy130_g852 = (int)(YRows118_g852-1) - fblinearindextoy130_g852;
				// Multiply Offset Y by rowoffset
				float fboffsety130_g852 = fblinearindextoy130_g852 * fbrowsoffset130_g852;
				// UV Offset
				float2 fboffset130_g852 = float2(fboffsetx130_g852, fboffsety130_g852);
				// Flipbook UV
				half2 fbuv130_g852 = appendResult124_g852 * fbtiling130_g852 + fboffset130_g852;
				// *** END Flipbook UV Animation vars ***
				float3 unpack132_g852 = UnpackNormalScale( tex2D( _TextureAtlasNormal, fbuv130_g852 ), _IntensityScaleNormal1 );
				unpack132_g852.z = lerp( 1, unpack132_g852.z, saturate(_IntensityScaleNormal1) );
				float3 tex2DNode132_g852 = unpack132_g852;
				float2 temp_cast_3 = (( FlipBookTiling100_g852 / _ScaleFBDetailsNormal )).xx;
				float2 texCoord105_g852 = IN.ase_texcoord7.xyz.xy * temp_cast_3 + _OffsetFBDetailsNormal;
				float cos111_g852 = cos( _FBDetailsNormal );
				float sin111_g852 = sin( _FBDetailsNormal );
				float2 rotator111_g852 = mul( texCoord105_g852 - float2( 0.5,0.5 ) , float2x2( cos111_g852 , -sin111_g852 , sin111_g852 , cos111_g852 )) + float2( 0.5,0.5 );
				float2 break112_g852 = rotator111_g852;
				float2 appendResult123_g852 = (float2(frac( break112_g852.x ) , frac( break112_g852.y )));
				float fbtotaltiles129_g852 = XColumns119_g852 * YRows118_g852;
				float fbcolsoffset129_g852 = 1.0f / XColumns119_g852;
				float fbrowsoffset129_g852 = 1.0f / YRows118_g852;
				float fbspeed129_g852 = _Time[ 1 ] * ZSpeed121_g852;
				float2 fbtiling129_g852 = float2(fbcolsoffset129_g852, fbrowsoffset129_g852);
				float fbcurrenttileindex129_g852 = round( fmod( fbspeed129_g852 + WStrartFrame122_g852, fbtotaltiles129_g852) );
				fbcurrenttileindex129_g852 += ( fbcurrenttileindex129_g852 < 0) ? fbtotaltiles129_g852 : 0;
				float fblinearindextox129_g852 = round ( fmod ( fbcurrenttileindex129_g852, XColumns119_g852 ) );
				float fboffsetx129_g852 = fblinearindextox129_g852 * fbcolsoffset129_g852;
				float fblinearindextoy129_g852 = round( fmod( ( fbcurrenttileindex129_g852 - fblinearindextox129_g852 ) / XColumns119_g852, YRows118_g852 ) );
				fblinearindextoy129_g852 = (int)(YRows118_g852-1) - fblinearindextoy129_g852;
				float fboffsety129_g852 = fblinearindextoy129_g852 * fbrowsoffset129_g852;
				float2 fboffset129_g852 = float2(fboffsetx129_g852, fboffsety129_g852);
				half2 fbuv129_g852 = appendResult123_g852 * fbtiling129_g852 + fboffset129_g852;
				float3 unpack134_g852 = UnpackNormalScale( tex2D( _TextureAtlasNormal, fbuv129_g852 ), _IntensityScaleNormal2 );
				unpack134_g852.z = lerp( 1, unpack134_g852.z, saturate(_IntensityScaleNormal2) );
				float3 RainDotsNormal739 = (( _DuplicateTextureAtlasNormal )?( BlendNormal( tex2DNode132_g852 , unpack134_g852 ) ):( tex2DNode132_g852 ));
				float lerpResult777 = lerp( 0.0 , 0.9 , _Distortion);
				float2 temp_cast_5 = (lerpResult777).xx;
				float4 tex2DNode7_g872 = tex2D( _DetailAlbedoMap, ( uv_DetailAlbedoMap + ( RainDotsNormal739.xy * temp_cast_5 ) ) );
				float4 BaseColorRGB403 = temp_output_764_0;
				#ifdef _USEMAINALBEDOMAP_ON
				float4 staticSwitch38_g872 = BaseColorRGB403;
				#else
				float4 staticSwitch38_g872 = tex2DNode7_g872;
				#endif
				float clampResult27_g872 = clamp( _DetailSaturation , -1.0 , 100.0 );
				float3 desaturateInitialColor29_g872 = ( _DetailColor * staticSwitch38_g872 ).rgb;
				float desaturateDot29_g872 = dot( desaturateInitialColor29_g872, float3( 0.299, 0.587, 0.114 ));
				float3 desaturateVar29_g872 = lerp( desaturateInitialColor29_g872, desaturateDot29_g872.xxx, -clampResult27_g872 );
				float4 AlbedoDetail153 = CalculateContrast(_DetailBrightness,float4( desaturateVar29_g872 , 0.0 ));
				float2 uv_DetailMask = IN.ase_texcoord7.xyz.xy * _DetailMask_ST.xy + _DetailMask_ST.zw;
				float4 tex2DNode27_g871 = tex2D( _DetailMask, uv_DetailMask );
				#if defined(_CHANNELSELECTIONMASK_RED)
				float staticSwitch28_g871 = tex2DNode27_g871.r;
				#elif defined(_CHANNELSELECTIONMASK_GREEN)
				float staticSwitch28_g871 = tex2DNode27_g871.g;
				#elif defined(_CHANNELSELECTIONMASK_BLUE)
				float staticSwitch28_g871 = tex2DNode27_g871.b;
				#elif defined(_CHANNELSELECTIONMASK_ALPHA)
				float staticSwitch28_g871 = tex2DNode27_g871.a;
				#else
				float staticSwitch28_g871 = tex2DNode27_g871.r;
				#endif
				float BaseColorAlpha46 = (( _InvertABaseColor )?( ( 1.0 - tex2DNode7_g853.a ) ):( tex2DNode7_g853.a ));
				float temp_output_43_0_g871 = BaseColorAlpha46;
				#if defined(_SOURCEMASK_DETAILMASK)
				float staticSwitch45_g871 = (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 ));
				#elif defined(_SOURCEMASK_BASECOLORALPHA)
				float staticSwitch45_g871 = temp_output_43_0_g871;
				#elif defined(_SOURCEMASK_BOTH)
				float staticSwitch45_g871 = ( (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 )) + temp_output_43_0_g871 );
				#else
				float staticSwitch45_g871 = (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 ));
				#endif
				float4 temp_cast_10 = (( staticSwitch45_g871 + (-1.2 + (_SpreadDetailMap - 0.0) * (0.7 - -1.2) / (1.0 - 0.0)) )).xxxx;
				float4 temp_output_35_0_g871 = CalculateContrast(( _ContrastDetailMap + 1.0 ),temp_cast_10);
				float4 clampResult38_g871 = clamp( (( _ExcludeMask )?( ( temp_output_35_0_g871 * ( 1.0 - temp_output_43_0_g871 ) ) ):( temp_output_35_0_g871 )) , float4( 0,0,0,0 ) , float4( 1,1,1,0 ) );
				float4 Mask158 = ( clampResult38_g871 * _IntensityMask );
				float4 lerpResult365 = lerp( temp_output_764_0 , AlbedoDetail153 , Mask158);
				float4 Albedo26 = (( _VisualizeMask )?( Mask158 ):( lerpResult365 ));
				
				float2 uv_BumpMap = IN.ase_texcoord7.xyz.xy * _BumpMap_ST.xy + _BumpMap_ST.zw;
				float2 break26_g893 = uv_BumpMap;
				float2 appendResult14_g893 = (float2(( break26_g893.x * GlobalTilingX11 ) , ( break26_g893.y * GlobalTilingY8 )));
				float2 appendResult13_g893 = (float2(( break26_g893.x + GlobalOffsetX10 ) , ( break26_g893.y + GlobalOffsetY9 )));
				float3 unpack4_g892 = UnpackNormalScale( tex2D( _BumpMap, ( ( appendResult14_g893 + appendResult13_g893 ) + Parallaxe494 ) ), _BumpScale );
				unpack4_g892.z = lerp( 1, unpack4_g892.z, saturate(_BumpScale) );
				float3 temp_output_495_0 = unpack4_g892;
				float2 temp_cast_12 = (_TranslationSpeed1).xx;
				float2 uv_DetailNormalMap = IN.ase_texcoord7.xyz.xy * _DetailNormalMap_ST.xy + _DetailNormalMap_ST.zw;
				float cos54_g889 = cos( radians( _RotationAngle1 ) );
				float sin54_g889 = sin( radians( _RotationAngle1 ) );
				float2 rotator54_g889 = mul( ( uv_DetailNormalMap * _TilingWave1 ) - float2( 0.5,0.5 ) , float2x2( cos54_g889 , -sin54_g889 , sin54_g889 , cos54_g889 )) + float2( 0.5,0.5 );
				float2 panner86_g889 = ( ( _TimeParameters.x * 0.05 ) * temp_cast_12 + rotator54_g889);
				float3 unpack107_g889 = UnpackNormalScale( tex2D( _DetailNormalMap, panner86_g889 ), _NormalWaveIntensity1 );
				unpack107_g889.z = lerp( 1, unpack107_g889.z, saturate(_NormalWaveIntensity1) );
				#ifdef _MAINWAVE_ON
				float3 staticSwitch109_g889 = unpack107_g889;
				#else
				float3 staticSwitch109_g889 = float3(0,0,1);
				#endif
				float2 temp_cast_13 = (_TranslationSpeed2).xx;
				float cos83_g889 = cos( radians( _RotationAngle2 ) );
				float sin83_g889 = sin( radians( _RotationAngle2 ) );
				float2 rotator83_g889 = mul( ( uv_DetailNormalMap * _TilingWave2 ) - float2( 0.5,0.5 ) , float2x2( cos83_g889 , -sin83_g889 , sin83_g889 , cos83_g889 )) + float2( 0.5,0.5 );
				float2 panner88_g889 = ( ( _TimeParameters.x * 0.05 ) * temp_cast_13 + rotator83_g889);
				float3 unpack108_g889 = UnpackNormalScale( tex2D( _DetailNormalMap, panner88_g889 ), _NormalWaveIntensity2 );
				unpack108_g889.z = lerp( 1, unpack108_g889.z, saturate(_NormalWaveIntensity2) );
				#ifdef _DETAILWAVE_ON
				float3 staticSwitch56_g889 = BlendNormal( staticSwitch109_g889 , unpack108_g889 );
				#else
				float3 staticSwitch56_g889 = staticSwitch109_g889;
				#endif
				float3 NormalPuddles459 = staticSwitch56_g889;
				float3 temp_output_740_0 = BlendNormal( NormalPuddles459 , RainDotsNormal739 );
				float3 lerpResult137 = lerp( temp_output_495_0 , temp_output_740_0 , Mask158.rgb);
				float3 lerpResult205 = lerp( temp_output_495_0 , BlendNormal( temp_output_495_0 , temp_output_740_0 ) , Mask158.rgb);
				float3 Normal27 = (( _BlendMainNormal )?( lerpResult205 ):( lerpResult137 ));
				
				float2 uv_EmissionMap = IN.ase_texcoord7.xyz.xy * _EmissionMap_ST.xy + _EmissionMap_ST.zw;
				float2 break26_g895 = uv_EmissionMap;
				float2 appendResult14_g895 = (float2(( break26_g895.x * GlobalTilingX11 ) , ( break26_g895.y * GlobalTilingY8 )));
				float2 appendResult13_g895 = (float2(( break26_g895.x + GlobalOffsetX10 ) , ( break26_g895.y + GlobalOffsetY9 )));
				float4 temp_output_503_0 = ( _EmissionColor * tex2D( _EmissionMap, ( ( appendResult14_g895 + appendResult13_g895 ) + Parallaxe494 ) ) * _EmissionIntensity );
				float4 temp_cast_16 = (0.0).xxxx;
				float4 lerpResult394 = lerp( temp_output_503_0 , temp_cast_16 , Mask158);
				float3 worldRefl28_g888 = normalize( reflect( -WorldViewDirection, float3( dot( tanToWorld0, (( _UseMainNormalMapasNormalDirection )?( Normal27 ):( NormalPuddles459 )) ), dot( tanToWorld1, (( _UseMainNormalMapasNormalDirection )?( Normal27 ):( NormalPuddles459 )) ), dot( tanToWorld2, (( _UseMainNormalMapasNormalDirection )?( Normal27 ):( NormalPuddles459 )) ) ) ) );
				float4 texCUBENode27_g888 = texCUBElod( _Cubemap, float4( worldRefl28_g888, _BlurReflection) );
				float clampResult39_g888 = clamp( _ReflectionIntensity , 0.0 , 100.0 );
				float4 Emission110 = ( (( _UseEmissionFromMainProperties )?( temp_output_503_0 ):( lerpResult394 )) + ( ( texCUBENode27_g888 * ( texCUBENode27_g888.a * clampResult39_g888 ) * _ColorCubemap1 ) * Mask158 ) );
				
				float DetailMetallic381 = _Metallic2;
				float lerpResult382 = lerp( ( tex2DNode3_g830.r * _Metallic ) , DetailMetallic381 , Mask158.r);
				float Metallic41 = lerpResult382;
				
				float temp_output_1_0_g830 = ( tex2DNode3_g830.a * _Smoothness );
				float2 temp_cast_21 = (_SplashSpeed).xx;
				float time58_g887 = ( 1.0 * 0.001 );
				float2 voronoiSmoothId0 = 0;
				float2 texCoord55_g887 = IN.ase_texcoord7.xyz.xy * float2( 1,1 ) + float2( 0,0 );
				float2 coords58_g887 = texCoord55_g887 * _Tiling;
				float2 id58_g887 = 0;
				float2 uv58_g887 = 0;
				float fade58_g887 = 0.5;
				float voroi58_g887 = 0;
				float rest58_g887 = 0;
				for( int it58_g887 = 0; it58_g887 <2; it58_g887++ ){
				voroi58_g887 += fade58_g887 * voronoi58_g887( coords58_g887, time58_g887, id58_g887, uv58_g887, 0,voronoiSmoothId0 );
				rest58_g887 += fade58_g887;
				coords58_g887 *= 2;
				fade58_g887 *= 0.5;
				}//Voronoi58_g887
				voroi58_g887 /= rest58_g887;
				float2 myVarName60_g887 = id58_g887;
				float2 panner63_g887 = ( 1.0 * _Time.y * temp_cast_21 + ( step( voroi58_g887 , 0.1 ) * myVarName60_g887 ));
				float RainDotsRoughness606 = ( saturate( ( tex2D( _GradientTex, panner63_g887 ).g * step( voroi58_g887 , (0.0 + (_Size - 0.0) * (0.05 - 0.0) / (1.0 - 0.0)) ) ) ) * _Intensity );
				float DetailSmoothness374 = _Glossiness2;
				float lerpResult377 = lerp( ( ( temp_output_1_0_g830 * ( 1.0 - RainDotsRoughness606 ) ) + RainDotsRoughness606 ) , DetailSmoothness374 , Mask158.r);
				float Smoothness40 = lerpResult377;
				
				float blendOpSrc34_g830 = tex2DNode3_g830.g;
				float blendOpDest34_g830 = ( 1.0 - _AoIntensity );
				float AoG829 = ( saturate( ( 1.0 - ( 1.0 - blendOpSrc34_g830 ) * ( 1.0 - blendOpDest34_g830 ) ) ));
				float lerpResult389 = lerp( AoG829 , 1.0 , Mask158.r);
				float AmbientOcclusion94 = (( _UseAoFromMainProperties )?( AoG829 ):( lerpResult389 ));
				
				float3 Albedo = Albedo26.rgb;
				float3 Normal = Normal27;
				float3 Emission = Emission110.rgb;
				float3 Specular = 0.5;
				float Metallic = Metallic41;
				float Smoothness = Smoothness40;
				float Occlusion = AmbientOcclusion94;
				float Alpha = 1;
				float AlphaClipThreshold = 0.5;
				float AlphaClipThresholdShadow = 0.5;
				float3 BakedGI = 0;
				float3 RefractionColor = 1;
				float RefractionIndex = 1;
				float3 Transmission = 1;
				float3 Translucency = 1;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				InputData inputData;
				inputData.positionWS = WorldPosition;
				inputData.viewDirectionWS = WorldViewDirection;
				inputData.shadowCoord = ShadowCoords;

				#ifdef _NORMALMAP
					#if _NORMAL_DROPOFF_TS
					inputData.normalWS = TransformTangentToWorld(Normal, half3x3( WorldTangent, WorldBiTangent, WorldNormal ));
					#elif _NORMAL_DROPOFF_OS
					inputData.normalWS = TransformObjectToWorldNormal(Normal);
					#elif _NORMAL_DROPOFF_WS
					inputData.normalWS = Normal;
					#endif
					inputData.normalWS = NormalizeNormalPerPixel(inputData.normalWS);
				#else
					inputData.normalWS = WorldNormal;
				#endif

				#ifdef ASE_FOG
					inputData.fogCoord = IN.fogFactorAndVertexLight.x;
				#endif

				inputData.vertexLighting = IN.fogFactorAndVertexLight.yzw;
				#if defined(ENABLE_TERRAIN_PERPIXEL_NORMAL)
					float3 SH = SampleSH(inputData.normalWS.xyz);
				#else
					float3 SH = IN.lightmapUVOrVertexSH.xyz;
				#endif

				inputData.bakedGI = SAMPLE_GI( IN.lightmapUVOrVertexSH.xy, SH, inputData.normalWS );
				#ifdef _ASE_BAKEDGI
					inputData.bakedGI = BakedGI;
				#endif
				half4 color = UniversalFragmentPBR(
					inputData, 
					Albedo, 
					Metallic, 
					Specular, 
					Smoothness, 
					Occlusion, 
					Emission, 
					Alpha);

				#ifdef _TRANSMISSION_ASE
				{
					float shadow = _TransmissionShadow;

					Light mainLight = GetMainLight( inputData.shadowCoord );
					float3 mainAtten = mainLight.color * mainLight.distanceAttenuation;
					mainAtten = lerp( mainAtten, mainAtten * mainLight.shadowAttenuation, shadow );
					half3 mainTransmission = max(0 , -dot(inputData.normalWS, mainLight.direction)) * mainAtten * Transmission;
					color.rgb += Albedo * mainTransmission;

					#ifdef _ADDITIONAL_LIGHTS
						int transPixelLightCount = GetAdditionalLightsCount();
						for (int i = 0; i < transPixelLightCount; ++i)
						{
							Light light = GetAdditionalLight(i, inputData.positionWS);
							float3 atten = light.color * light.distanceAttenuation;
							atten = lerp( atten, atten * light.shadowAttenuation, shadow );

							half3 transmission = max(0 , -dot(inputData.normalWS, light.direction)) * atten * Transmission;
							color.rgb += Albedo * transmission;
						}
					#endif
				}
				#endif

				#ifdef _TRANSLUCENCY_ASE
				{
					float shadow = _TransShadow;
					float normal = _TransNormal;
					float scattering = _TransScattering;
					float direct = _TransDirect;
					float ambient = _TransAmbient;
					float strength = _TransStrength;

					Light mainLight = GetMainLight( inputData.shadowCoord );
					float3 mainAtten = mainLight.color * mainLight.distanceAttenuation;
					mainAtten = lerp( mainAtten, mainAtten * mainLight.shadowAttenuation, shadow );

					half3 mainLightDir = mainLight.direction + inputData.normalWS * normal;
					half mainVdotL = pow( saturate( dot( inputData.viewDirectionWS, -mainLightDir ) ), scattering );
					half3 mainTranslucency = mainAtten * ( mainVdotL * direct + inputData.bakedGI * ambient ) * Translucency;
					color.rgb += Albedo * mainTranslucency * strength;

					#ifdef _ADDITIONAL_LIGHTS
						int transPixelLightCount = GetAdditionalLightsCount();
						for (int i = 0; i < transPixelLightCount; ++i)
						{
							Light light = GetAdditionalLight(i, inputData.positionWS);
							float3 atten = light.color * light.distanceAttenuation;
							atten = lerp( atten, atten * light.shadowAttenuation, shadow );

							half3 lightDir = light.direction + inputData.normalWS * normal;
							half VdotL = pow( saturate( dot( inputData.viewDirectionWS, -lightDir ) ), scattering );
							half3 translucency = atten * ( VdotL * direct + inputData.bakedGI * ambient ) * Translucency;
							color.rgb += Albedo * translucency * strength;
						}
					#endif
				}
				#endif

				#ifdef _REFRACTION_ASE
					float4 projScreenPos = ScreenPos / ScreenPos.w;
					float3 refractionOffset = ( RefractionIndex - 1.0 ) * mul( UNITY_MATRIX_V, float4( WorldNormal, 0 ) ).xyz * ( 1.0 - dot( WorldNormal, WorldViewDirection ) );
					projScreenPos.xy += refractionOffset.xy;
					float3 refraction = SHADERGRAPH_SAMPLE_SCENE_COLOR( projScreenPos.xy ) * RefractionColor;
					color.rgb = lerp( refraction, color.rgb, color.a );
					color.a = 1;
				#endif

				#ifdef ASE_FINAL_COLOR_ALPHA_MULTIPLY
					color.rgb *= color.a;
				#endif

				#ifdef ASE_FOG
					#ifdef TERRAIN_SPLAT_ADDPASS
						color.rgb = MixFogColor(color.rgb, half3( 0, 0, 0 ), IN.fogFactorAndVertexLight.x );
					#else
						color.rgb = MixFog(color.rgb, IN.fogFactorAndVertexLight.x);
					#endif
				#endif
				
				#ifdef ASE_DEPTH_WRITE_ON
					outputDepth = DepthValue;
				#endif

				return color;
			}

			ENDHLSL
		}

		
		Pass
		{
			
			Name "ShadowCaster"
			Tags { "LightMode"="ShadowCaster" }

			ZWrite On
			ZTest LEqual
			AlphaToMask Off

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile_instancing
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define _EMISSION
			#define _NORMALMAP 1
			#define ASE_SRP_VERSION 70301

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_SHADOWCASTER

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"

			

			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _DetailAlbedoMap_ST;
			float4 _EmissionColor;
			float4 _Color;
			float4 _BaseMap_ST;
			float4 _GlobalXYTilingXYZWOffsetXY;
			float4 _MetallicGlossMapMAHS_ST;
			float4 _ColorCubemap1;
			float4 _BumpMap_ST;
			float4 _DetailNormalMap_ST;
			float4 _DetailColor;
			float4 _EmissionMap_ST;
			float4 _DetailMask_ST;
			float4 _XColumnsYRowsZSpeedWStrartFrameNormal;
			float2 _OffsetFBDetailsNormal;
			float _UseEmissionFromMainProperties;
			float _RotationAngle2;
			float _TilingWave2;
			float _TranslationSpeed2;
			float _NormalWaveIntensity1;
			float _NormalWaveIntensity2;
			float _EmissionIntensity;
			float _VisualizeMask;
			float _BlurReflection;
			float _ReflectionIntensity;
			float _Metallic;
			float _Metallic2;
			float _Smoothness;
			float _SplashSpeed;
			float _Tiling;
			float _Size;
			float _Intensity;
			float _Glossiness2;
			float _UseMainNormalMapasNormalDirection;
			float _RotationAngle1;
			float _BlendMainNormal;
			float _TranslationSpeed1;
			float _Brightness;
			float _Parallax;
			float _Saturation;
			float _DetailBrightness;
			float _DuplicateTextureAtlasNormal;
			float _FlipBTilingNormal;
			float _IntensityScaleNormal1;
			float _ScaleFBDetailsNormal;
			float _FBDetailsNormal;
			float _TilingWave1;
			float _IntensityScaleNormal2;
			float _DetailSaturation;
			float _ExcludeMask;
			float _ContrastDetailMap;
			float _InvertMask;
			float _InvertABaseColor;
			float _SpreadDetailMap;
			float _IntensityMask;
			float _UseAoFromMainProperties;
			float _BumpScale;
			float _Distortion;
			float _AoIntensity;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			

			
			float3 _LightDirection;

			VertexOutput VertexFunction( VertexInput v )
			{
				VertexOutput o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO( o );

				
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif
				float3 normalWS = TransformObjectToWorldDir(v.ase_normal);

				float4 clipPos = TransformWorldToHClip( ApplyShadowBias( positionWS, normalWS, _LightDirection ) );

				#if UNITY_REVERSED_Z
					clipPos.z = min(clipPos.z, clipPos.w * UNITY_NEAR_CLIP_VALUE);
				#else
					clipPos.z = max(clipPos.z, clipPos.w * UNITY_NEAR_CLIP_VALUE);
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = clipPos;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				o.clipPos = clipPos;
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif

			half4 frag(	VertexOutput IN 
						#ifdef ASE_DEPTH_WRITE_ON
						,out float outputDepth : ASE_SV_DEPTH
						#endif
						 ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID( IN );
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );
				
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				
				float Alpha = 1;
				float AlphaClipThreshold = 0.5;
				float AlphaClipThresholdShadow = 0.5;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					#ifdef _ALPHATEST_SHADOW_ON
						clip(Alpha - AlphaClipThresholdShadow);
					#else
						clip(Alpha - AlphaClipThreshold);
					#endif
				#endif

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif
				#ifdef ASE_DEPTH_WRITE_ON
					outputDepth = DepthValue;
				#endif
				return 0;
			}

			ENDHLSL
		}

		
		Pass
		{
			
			Name "DepthOnly"
			Tags { "LightMode"="DepthOnly" }

			ZWrite On
			ColorMask 0
			AlphaToMask Off

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile_instancing
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define _EMISSION
			#define _NORMALMAP 1
			#define ASE_SRP_VERSION 70301

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_DEPTHONLY

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"

			

			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _DetailAlbedoMap_ST;
			float4 _EmissionColor;
			float4 _Color;
			float4 _BaseMap_ST;
			float4 _GlobalXYTilingXYZWOffsetXY;
			float4 _MetallicGlossMapMAHS_ST;
			float4 _ColorCubemap1;
			float4 _BumpMap_ST;
			float4 _DetailNormalMap_ST;
			float4 _DetailColor;
			float4 _EmissionMap_ST;
			float4 _DetailMask_ST;
			float4 _XColumnsYRowsZSpeedWStrartFrameNormal;
			float2 _OffsetFBDetailsNormal;
			float _UseEmissionFromMainProperties;
			float _RotationAngle2;
			float _TilingWave2;
			float _TranslationSpeed2;
			float _NormalWaveIntensity1;
			float _NormalWaveIntensity2;
			float _EmissionIntensity;
			float _VisualizeMask;
			float _BlurReflection;
			float _ReflectionIntensity;
			float _Metallic;
			float _Metallic2;
			float _Smoothness;
			float _SplashSpeed;
			float _Tiling;
			float _Size;
			float _Intensity;
			float _Glossiness2;
			float _UseMainNormalMapasNormalDirection;
			float _RotationAngle1;
			float _BlendMainNormal;
			float _TranslationSpeed1;
			float _Brightness;
			float _Parallax;
			float _Saturation;
			float _DetailBrightness;
			float _DuplicateTextureAtlasNormal;
			float _FlipBTilingNormal;
			float _IntensityScaleNormal1;
			float _ScaleFBDetailsNormal;
			float _FBDetailsNormal;
			float _TilingWave1;
			float _IntensityScaleNormal2;
			float _DetailSaturation;
			float _ExcludeMask;
			float _ContrastDetailMap;
			float _InvertMask;
			float _InvertABaseColor;
			float _SpreadDetailMap;
			float _IntensityMask;
			float _UseAoFromMainProperties;
			float _BumpScale;
			float _Distortion;
			float _AoIntensity;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			

			
			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;
				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float4 positionCS = TransformWorldToHClip( positionWS );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = positionCS;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				o.clipPos = positionCS;
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			#if defined(ASE_EARLY_Z_DEPTH_OPTIMIZE)
				#define ASE_SV_DEPTH SV_DepthLessEqual  
			#else
				#define ASE_SV_DEPTH SV_Depth
			#endif
			half4 frag(	VertexOutput IN 
						#ifdef ASE_DEPTH_WRITE_ON
						,out float outputDepth : ASE_SV_DEPTH
						#endif
						 ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				
				float Alpha = 1;
				float AlphaClipThreshold = 0.5;
				#ifdef ASE_DEPTH_WRITE_ON
				float DepthValue = 0;
				#endif

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				#ifdef LOD_FADE_CROSSFADE
					LODDitheringTransition( IN.clipPos.xyz, unity_LODFade.x );
				#endif
				#ifdef ASE_DEPTH_WRITE_ON
				outputDepth = DepthValue;
				#endif
				return 0;
			}
			ENDHLSL
		}

		
		Pass
		{
			
			Name "Meta"
			Tags { "LightMode"="Meta" }

			Cull Off

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile_instancing
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define _EMISSION
			#define _NORMALMAP 1
			#define ASE_SRP_VERSION 70301

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_META

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/MetaInput.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"

			#define ASE_NEEDS_VERT_NORMAL
			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma shader_feature_local _USEMAINALBEDOMAP_ON
			#pragma shader_feature_local _SOURCEMASK_DETAILMASK _SOURCEMASK_BASECOLORALPHA _SOURCEMASK_BOTH
			#pragma shader_feature_local _CHANNELSELECTIONMASK_RED _CHANNELSELECTIONMASK_GREEN _CHANNELSELECTIONMASK_BLUE _CHANNELSELECTIONMASK_ALPHA
			#pragma shader_feature_local _DETAILWAVE_ON
			#pragma shader_feature_local _MAINWAVE_ON


			#pragma shader_feature _ _SMOOTHNESS_TEXTURE_ALBEDO_CHANNEL_A

			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 texcoord1 : TEXCOORD1;
				float4 texcoord2 : TEXCOORD2;
				float4 ase_texcoord : TEXCOORD0;
				float4 ase_tangent : TANGENT;
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				float4 ase_texcoord2 : TEXCOORD2;
				float4 ase_texcoord3 : TEXCOORD3;
				float4 ase_texcoord4 : TEXCOORD4;
				float4 ase_texcoord5 : TEXCOORD5;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _DetailAlbedoMap_ST;
			float4 _EmissionColor;
			float4 _Color;
			float4 _BaseMap_ST;
			float4 _GlobalXYTilingXYZWOffsetXY;
			float4 _MetallicGlossMapMAHS_ST;
			float4 _ColorCubemap1;
			float4 _BumpMap_ST;
			float4 _DetailNormalMap_ST;
			float4 _DetailColor;
			float4 _EmissionMap_ST;
			float4 _DetailMask_ST;
			float4 _XColumnsYRowsZSpeedWStrartFrameNormal;
			float2 _OffsetFBDetailsNormal;
			float _UseEmissionFromMainProperties;
			float _RotationAngle2;
			float _TilingWave2;
			float _TranslationSpeed2;
			float _NormalWaveIntensity1;
			float _NormalWaveIntensity2;
			float _EmissionIntensity;
			float _VisualizeMask;
			float _BlurReflection;
			float _ReflectionIntensity;
			float _Metallic;
			float _Metallic2;
			float _Smoothness;
			float _SplashSpeed;
			float _Tiling;
			float _Size;
			float _Intensity;
			float _Glossiness2;
			float _UseMainNormalMapasNormalDirection;
			float _RotationAngle1;
			float _BlendMainNormal;
			float _TranslationSpeed1;
			float _Brightness;
			float _Parallax;
			float _Saturation;
			float _DetailBrightness;
			float _DuplicateTextureAtlasNormal;
			float _FlipBTilingNormal;
			float _IntensityScaleNormal1;
			float _ScaleFBDetailsNormal;
			float _FBDetailsNormal;
			float _TilingWave1;
			float _IntensityScaleNormal2;
			float _DetailSaturation;
			float _ExcludeMask;
			float _ContrastDetailMap;
			float _InvertMask;
			float _InvertABaseColor;
			float _SpreadDetailMap;
			float _IntensityMask;
			float _UseAoFromMainProperties;
			float _BumpScale;
			float _Distortion;
			float _AoIntensity;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			sampler2D _BaseMap;
			sampler2D _MetallicGlossMapMAHS;
			sampler2D _DetailAlbedoMap;
			sampler2D _TextureAtlasNormal;
			sampler2D _DetailMask;
			sampler2D _EmissionMap;
			samplerCUBE _Cubemap;
			sampler2D _DetailNormalMap;
			sampler2D _BumpMap;


			inline float2 ParallaxOffset( half h, half height, half3 viewDir )
			{
				h = h * height - height/2.0;
				float3 v = normalize( viewDir );
				v.z += 0.42;
				return h* (v.xy / v.z);
			}
			
			float4 CalculateContrast( float contrastValue, float4 colorTarget )
			{
				float t = 0.5 * ( 1.0 - contrastValue );
				return mul( float4x4( contrastValue,0,0,t, 0,contrastValue,0,t, 0,0,contrastValue,t, 0,0,0,1 ), colorTarget );
			}

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

				float3 ase_worldTangent = TransformObjectToWorldDir(v.ase_tangent.xyz);
				o.ase_texcoord3.xyz = ase_worldTangent;
				float3 ase_worldNormal = TransformObjectToWorldNormal(v.ase_normal);
				o.ase_texcoord4.xyz = ase_worldNormal;
				float ase_vertexTangentSign = v.ase_tangent.w * unity_WorldTransformParams.w;
				float3 ase_worldBitangent = cross( ase_worldNormal, ase_worldTangent ) * ase_vertexTangentSign;
				o.ase_texcoord5.xyz = ase_worldBitangent;
				
				o.ase_texcoord2.xyz = v.ase_texcoord.xyz;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord2.w = 0;
				o.ase_texcoord3.w = 0;
				o.ase_texcoord4.w = 0;
				o.ase_texcoord5.w = 0;
				
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif

				o.clipPos = MetaVertexPosition( v.vertex, v.texcoord1.xy, v.texcoord1.xy, unity_LightmapST, unity_DynamicLightmapST );
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = o.clipPos;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 texcoord1 : TEXCOORD1;
				float4 texcoord2 : TEXCOORD2;
				float4 ase_texcoord : TEXCOORD0;
				float4 ase_tangent : TANGENT;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.texcoord1 = v.texcoord1;
				o.texcoord2 = v.texcoord2;
				o.ase_texcoord = v.ase_texcoord;
				o.ase_tangent = v.ase_tangent;
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.texcoord1 = patch[0].texcoord1 * bary.x + patch[1].texcoord1 * bary.y + patch[2].texcoord1 * bary.z;
				o.texcoord2 = patch[0].texcoord2 * bary.x + patch[1].texcoord2 * bary.y + patch[2].texcoord2 * bary.z;
				o.ase_texcoord = patch[0].ase_texcoord * bary.x + patch[1].ase_texcoord * bary.y + patch[2].ase_texcoord * bary.z;
				o.ase_tangent = patch[0].ase_tangent * bary.x + patch[1].ase_tangent * bary.y + patch[2].ase_tangent * bary.z;
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			half4 frag(VertexOutput IN , FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID(IN);
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				float2 uv_BaseMap = IN.ase_texcoord2.xyz.xy * _BaseMap_ST.xy + _BaseMap_ST.zw;
				float2 break26_g854 = uv_BaseMap;
				float GlobalTilingX11 = ( _GlobalXYTilingXYZWOffsetXY.x - 1.0 );
				float GlobalTilingY8 = ( _GlobalXYTilingXYZWOffsetXY.y - 1.0 );
				float2 appendResult14_g854 = (float2(( break26_g854.x * GlobalTilingX11 ) , ( break26_g854.y * GlobalTilingY8 )));
				float GlobalOffsetX10 = _GlobalXYTilingXYZWOffsetXY.z;
				float GlobalOffsetY9 = _GlobalXYTilingXYZWOffsetXY.w;
				float2 appendResult13_g854 = (float2(( break26_g854.x + GlobalOffsetX10 ) , ( break26_g854.y + GlobalOffsetY9 )));
				float2 uv_MetallicGlossMapMAHS = IN.ase_texcoord2.xyz.xy * _MetallicGlossMapMAHS_ST.xy + _MetallicGlossMapMAHS_ST.zw;
				float2 break26_g831 = uv_MetallicGlossMapMAHS;
				float2 appendResult14_g831 = (float2(( break26_g831.x * GlobalTilingX11 ) , ( break26_g831.y * GlobalTilingY8 )));
				float2 appendResult13_g831 = (float2(( break26_g831.x + GlobalOffsetX10 ) , ( break26_g831.y + GlobalOffsetY9 )));
				float4 tex2DNode3_g830 = tex2D( _MetallicGlossMapMAHS, ( ( appendResult14_g831 + appendResult13_g831 ) + float2( 0,0 ) ) );
				float3 ase_worldTangent = IN.ase_texcoord3.xyz;
				float3 ase_worldNormal = IN.ase_texcoord4.xyz;
				float3 ase_worldBitangent = IN.ase_texcoord5.xyz;
				float3 tanToWorld0 = float3( ase_worldTangent.x, ase_worldBitangent.x, ase_worldNormal.x );
				float3 tanToWorld1 = float3( ase_worldTangent.y, ase_worldBitangent.y, ase_worldNormal.y );
				float3 tanToWorld2 = float3( ase_worldTangent.z, ase_worldBitangent.z, ase_worldNormal.z );
				float3 ase_worldViewDir = ( _WorldSpaceCameraPos.xyz - WorldPosition );
				ase_worldViewDir = normalize(ase_worldViewDir);
				float3 ase_tanViewDir =  tanToWorld0 * ase_worldViewDir.x + tanToWorld1 * ase_worldViewDir.y  + tanToWorld2 * ase_worldViewDir.z;
				ase_tanViewDir = normalize(ase_tanViewDir);
				float2 paralaxOffset38_g830 = ParallaxOffset( tex2DNode3_g830.b , _Parallax , ase_tanViewDir );
				float2 switchResult37_g830 = (((ase_vface>0)?(paralaxOffset38_g830):(0.0)));
				float2 Parallaxe494 = switchResult37_g830;
				float4 tex2DNode7_g853 = tex2D( _BaseMap, ( ( appendResult14_g854 + appendResult13_g854 ) + Parallaxe494 ) );
				float clampResult27_g853 = clamp( _Saturation , -1.0 , 100.0 );
				float3 desaturateInitialColor29_g853 = ( _Color * tex2DNode7_g853 ).rgb;
				float desaturateDot29_g853 = dot( desaturateInitialColor29_g853, float3( 0.299, 0.587, 0.114 ));
				float3 desaturateVar29_g853 = lerp( desaturateInitialColor29_g853, desaturateDot29_g853.xxx, -clampResult27_g853 );
				float4 temp_output_764_0 = CalculateContrast(_Brightness,float4( desaturateVar29_g853 , 0.0 ));
				float2 uv_DetailAlbedoMap = IN.ase_texcoord2.xyz.xy * _DetailAlbedoMap_ST.xy + _DetailAlbedoMap_ST.zw;
				float FlipBookTiling100_g852 = _FlipBTilingNormal;
				float2 temp_cast_2 = (FlipBookTiling100_g852).xx;
				float2 texCoord114_g852 = IN.ase_texcoord2.xyz.xy * temp_cast_2 + float2( 0,0 );
				float2 appendResult124_g852 = (float2(frac( texCoord114_g852.x ) , frac( texCoord114_g852.y )));
				float XColumns119_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.x;
				float YRows118_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.y;
				float ZSpeed121_g852 = ( ( _XColumnsYRowsZSpeedWStrartFrameNormal.x * _XColumnsYRowsZSpeedWStrartFrameNormal.y ) * _XColumnsYRowsZSpeedWStrartFrameNormal.z );
				float WStrartFrame122_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.w;
				// *** BEGIN Flipbook UV Animation vars ***
				// Total tiles of Flipbook Texture
				float fbtotaltiles130_g852 = XColumns119_g852 * YRows118_g852;
				// Offsets for cols and rows of Flipbook Texture
				float fbcolsoffset130_g852 = 1.0f / XColumns119_g852;
				float fbrowsoffset130_g852 = 1.0f / YRows118_g852;
				// Speed of animation
				float fbspeed130_g852 = _Time[ 1 ] * ZSpeed121_g852;
				// UV Tiling (col and row offset)
				float2 fbtiling130_g852 = float2(fbcolsoffset130_g852, fbrowsoffset130_g852);
				// UV Offset - calculate current tile linear index, and convert it to (X * coloffset, Y * rowoffset)
				// Calculate current tile linear index
				float fbcurrenttileindex130_g852 = round( fmod( fbspeed130_g852 + WStrartFrame122_g852, fbtotaltiles130_g852) );
				fbcurrenttileindex130_g852 += ( fbcurrenttileindex130_g852 < 0) ? fbtotaltiles130_g852 : 0;
				// Obtain Offset X coordinate from current tile linear index
				float fblinearindextox130_g852 = round ( fmod ( fbcurrenttileindex130_g852, XColumns119_g852 ) );
				// Multiply Offset X by coloffset
				float fboffsetx130_g852 = fblinearindextox130_g852 * fbcolsoffset130_g852;
				// Obtain Offset Y coordinate from current tile linear index
				float fblinearindextoy130_g852 = round( fmod( ( fbcurrenttileindex130_g852 - fblinearindextox130_g852 ) / XColumns119_g852, YRows118_g852 ) );
				// Reverse Y to get tiles from Top to Bottom
				fblinearindextoy130_g852 = (int)(YRows118_g852-1) - fblinearindextoy130_g852;
				// Multiply Offset Y by rowoffset
				float fboffsety130_g852 = fblinearindextoy130_g852 * fbrowsoffset130_g852;
				// UV Offset
				float2 fboffset130_g852 = float2(fboffsetx130_g852, fboffsety130_g852);
				// Flipbook UV
				half2 fbuv130_g852 = appendResult124_g852 * fbtiling130_g852 + fboffset130_g852;
				// *** END Flipbook UV Animation vars ***
				float3 unpack132_g852 = UnpackNormalScale( tex2D( _TextureAtlasNormal, fbuv130_g852 ), _IntensityScaleNormal1 );
				unpack132_g852.z = lerp( 1, unpack132_g852.z, saturate(_IntensityScaleNormal1) );
				float3 tex2DNode132_g852 = unpack132_g852;
				float2 temp_cast_3 = (( FlipBookTiling100_g852 / _ScaleFBDetailsNormal )).xx;
				float2 texCoord105_g852 = IN.ase_texcoord2.xyz.xy * temp_cast_3 + _OffsetFBDetailsNormal;
				float cos111_g852 = cos( _FBDetailsNormal );
				float sin111_g852 = sin( _FBDetailsNormal );
				float2 rotator111_g852 = mul( texCoord105_g852 - float2( 0.5,0.5 ) , float2x2( cos111_g852 , -sin111_g852 , sin111_g852 , cos111_g852 )) + float2( 0.5,0.5 );
				float2 break112_g852 = rotator111_g852;
				float2 appendResult123_g852 = (float2(frac( break112_g852.x ) , frac( break112_g852.y )));
				float fbtotaltiles129_g852 = XColumns119_g852 * YRows118_g852;
				float fbcolsoffset129_g852 = 1.0f / XColumns119_g852;
				float fbrowsoffset129_g852 = 1.0f / YRows118_g852;
				float fbspeed129_g852 = _Time[ 1 ] * ZSpeed121_g852;
				float2 fbtiling129_g852 = float2(fbcolsoffset129_g852, fbrowsoffset129_g852);
				float fbcurrenttileindex129_g852 = round( fmod( fbspeed129_g852 + WStrartFrame122_g852, fbtotaltiles129_g852) );
				fbcurrenttileindex129_g852 += ( fbcurrenttileindex129_g852 < 0) ? fbtotaltiles129_g852 : 0;
				float fblinearindextox129_g852 = round ( fmod ( fbcurrenttileindex129_g852, XColumns119_g852 ) );
				float fboffsetx129_g852 = fblinearindextox129_g852 * fbcolsoffset129_g852;
				float fblinearindextoy129_g852 = round( fmod( ( fbcurrenttileindex129_g852 - fblinearindextox129_g852 ) / XColumns119_g852, YRows118_g852 ) );
				fblinearindextoy129_g852 = (int)(YRows118_g852-1) - fblinearindextoy129_g852;
				float fboffsety129_g852 = fblinearindextoy129_g852 * fbrowsoffset129_g852;
				float2 fboffset129_g852 = float2(fboffsetx129_g852, fboffsety129_g852);
				half2 fbuv129_g852 = appendResult123_g852 * fbtiling129_g852 + fboffset129_g852;
				float3 unpack134_g852 = UnpackNormalScale( tex2D( _TextureAtlasNormal, fbuv129_g852 ), _IntensityScaleNormal2 );
				unpack134_g852.z = lerp( 1, unpack134_g852.z, saturate(_IntensityScaleNormal2) );
				float3 RainDotsNormal739 = (( _DuplicateTextureAtlasNormal )?( BlendNormal( tex2DNode132_g852 , unpack134_g852 ) ):( tex2DNode132_g852 ));
				float lerpResult777 = lerp( 0.0 , 0.9 , _Distortion);
				float2 temp_cast_5 = (lerpResult777).xx;
				float4 tex2DNode7_g872 = tex2D( _DetailAlbedoMap, ( uv_DetailAlbedoMap + ( RainDotsNormal739.xy * temp_cast_5 ) ) );
				float4 BaseColorRGB403 = temp_output_764_0;
				#ifdef _USEMAINALBEDOMAP_ON
				float4 staticSwitch38_g872 = BaseColorRGB403;
				#else
				float4 staticSwitch38_g872 = tex2DNode7_g872;
				#endif
				float clampResult27_g872 = clamp( _DetailSaturation , -1.0 , 100.0 );
				float3 desaturateInitialColor29_g872 = ( _DetailColor * staticSwitch38_g872 ).rgb;
				float desaturateDot29_g872 = dot( desaturateInitialColor29_g872, float3( 0.299, 0.587, 0.114 ));
				float3 desaturateVar29_g872 = lerp( desaturateInitialColor29_g872, desaturateDot29_g872.xxx, -clampResult27_g872 );
				float4 AlbedoDetail153 = CalculateContrast(_DetailBrightness,float4( desaturateVar29_g872 , 0.0 ));
				float2 uv_DetailMask = IN.ase_texcoord2.xyz.xy * _DetailMask_ST.xy + _DetailMask_ST.zw;
				float4 tex2DNode27_g871 = tex2D( _DetailMask, uv_DetailMask );
				#if defined(_CHANNELSELECTIONMASK_RED)
				float staticSwitch28_g871 = tex2DNode27_g871.r;
				#elif defined(_CHANNELSELECTIONMASK_GREEN)
				float staticSwitch28_g871 = tex2DNode27_g871.g;
				#elif defined(_CHANNELSELECTIONMASK_BLUE)
				float staticSwitch28_g871 = tex2DNode27_g871.b;
				#elif defined(_CHANNELSELECTIONMASK_ALPHA)
				float staticSwitch28_g871 = tex2DNode27_g871.a;
				#else
				float staticSwitch28_g871 = tex2DNode27_g871.r;
				#endif
				float BaseColorAlpha46 = (( _InvertABaseColor )?( ( 1.0 - tex2DNode7_g853.a ) ):( tex2DNode7_g853.a ));
				float temp_output_43_0_g871 = BaseColorAlpha46;
				#if defined(_SOURCEMASK_DETAILMASK)
				float staticSwitch45_g871 = (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 ));
				#elif defined(_SOURCEMASK_BASECOLORALPHA)
				float staticSwitch45_g871 = temp_output_43_0_g871;
				#elif defined(_SOURCEMASK_BOTH)
				float staticSwitch45_g871 = ( (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 )) + temp_output_43_0_g871 );
				#else
				float staticSwitch45_g871 = (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 ));
				#endif
				float4 temp_cast_10 = (( staticSwitch45_g871 + (-1.2 + (_SpreadDetailMap - 0.0) * (0.7 - -1.2) / (1.0 - 0.0)) )).xxxx;
				float4 temp_output_35_0_g871 = CalculateContrast(( _ContrastDetailMap + 1.0 ),temp_cast_10);
				float4 clampResult38_g871 = clamp( (( _ExcludeMask )?( ( temp_output_35_0_g871 * ( 1.0 - temp_output_43_0_g871 ) ) ):( temp_output_35_0_g871 )) , float4( 0,0,0,0 ) , float4( 1,1,1,0 ) );
				float4 Mask158 = ( clampResult38_g871 * _IntensityMask );
				float4 lerpResult365 = lerp( temp_output_764_0 , AlbedoDetail153 , Mask158);
				float4 Albedo26 = (( _VisualizeMask )?( Mask158 ):( lerpResult365 ));
				
				float2 uv_EmissionMap = IN.ase_texcoord2.xyz.xy * _EmissionMap_ST.xy + _EmissionMap_ST.zw;
				float2 break26_g895 = uv_EmissionMap;
				float2 appendResult14_g895 = (float2(( break26_g895.x * GlobalTilingX11 ) , ( break26_g895.y * GlobalTilingY8 )));
				float2 appendResult13_g895 = (float2(( break26_g895.x + GlobalOffsetX10 ) , ( break26_g895.y + GlobalOffsetY9 )));
				float4 temp_output_503_0 = ( _EmissionColor * tex2D( _EmissionMap, ( ( appendResult14_g895 + appendResult13_g895 ) + Parallaxe494 ) ) * _EmissionIntensity );
				float4 temp_cast_12 = (0.0).xxxx;
				float4 lerpResult394 = lerp( temp_output_503_0 , temp_cast_12 , Mask158);
				float2 temp_cast_13 = (_TranslationSpeed1).xx;
				float2 uv_DetailNormalMap = IN.ase_texcoord2.xyz.xy * _DetailNormalMap_ST.xy + _DetailNormalMap_ST.zw;
				float cos54_g889 = cos( radians( _RotationAngle1 ) );
				float sin54_g889 = sin( radians( _RotationAngle1 ) );
				float2 rotator54_g889 = mul( ( uv_DetailNormalMap * _TilingWave1 ) - float2( 0.5,0.5 ) , float2x2( cos54_g889 , -sin54_g889 , sin54_g889 , cos54_g889 )) + float2( 0.5,0.5 );
				float2 panner86_g889 = ( ( _TimeParameters.x * 0.05 ) * temp_cast_13 + rotator54_g889);
				float3 unpack107_g889 = UnpackNormalScale( tex2D( _DetailNormalMap, panner86_g889 ), _NormalWaveIntensity1 );
				unpack107_g889.z = lerp( 1, unpack107_g889.z, saturate(_NormalWaveIntensity1) );
				#ifdef _MAINWAVE_ON
				float3 staticSwitch109_g889 = unpack107_g889;
				#else
				float3 staticSwitch109_g889 = float3(0,0,1);
				#endif
				float2 temp_cast_14 = (_TranslationSpeed2).xx;
				float cos83_g889 = cos( radians( _RotationAngle2 ) );
				float sin83_g889 = sin( radians( _RotationAngle2 ) );
				float2 rotator83_g889 = mul( ( uv_DetailNormalMap * _TilingWave2 ) - float2( 0.5,0.5 ) , float2x2( cos83_g889 , -sin83_g889 , sin83_g889 , cos83_g889 )) + float2( 0.5,0.5 );
				float2 panner88_g889 = ( ( _TimeParameters.x * 0.05 ) * temp_cast_14 + rotator83_g889);
				float3 unpack108_g889 = UnpackNormalScale( tex2D( _DetailNormalMap, panner88_g889 ), _NormalWaveIntensity2 );
				unpack108_g889.z = lerp( 1, unpack108_g889.z, saturate(_NormalWaveIntensity2) );
				#ifdef _DETAILWAVE_ON
				float3 staticSwitch56_g889 = BlendNormal( staticSwitch109_g889 , unpack108_g889 );
				#else
				float3 staticSwitch56_g889 = staticSwitch109_g889;
				#endif
				float3 NormalPuddles459 = staticSwitch56_g889;
				float2 uv_BumpMap = IN.ase_texcoord2.xyz.xy * _BumpMap_ST.xy + _BumpMap_ST.zw;
				float2 break26_g893 = uv_BumpMap;
				float2 appendResult14_g893 = (float2(( break26_g893.x * GlobalTilingX11 ) , ( break26_g893.y * GlobalTilingY8 )));
				float2 appendResult13_g893 = (float2(( break26_g893.x + GlobalOffsetX10 ) , ( break26_g893.y + GlobalOffsetY9 )));
				float3 unpack4_g892 = UnpackNormalScale( tex2D( _BumpMap, ( ( appendResult14_g893 + appendResult13_g893 ) + Parallaxe494 ) ), _BumpScale );
				unpack4_g892.z = lerp( 1, unpack4_g892.z, saturate(_BumpScale) );
				float3 temp_output_495_0 = unpack4_g892;
				float3 temp_output_740_0 = BlendNormal( NormalPuddles459 , RainDotsNormal739 );
				float3 lerpResult137 = lerp( temp_output_495_0 , temp_output_740_0 , Mask158.rgb);
				float3 lerpResult205 = lerp( temp_output_495_0 , BlendNormal( temp_output_495_0 , temp_output_740_0 ) , Mask158.rgb);
				float3 Normal27 = (( _BlendMainNormal )?( lerpResult205 ):( lerpResult137 ));
				float3 worldRefl28_g888 = normalize( reflect( -ase_worldViewDir, float3( dot( tanToWorld0, (( _UseMainNormalMapasNormalDirection )?( Normal27 ):( NormalPuddles459 )) ), dot( tanToWorld1, (( _UseMainNormalMapasNormalDirection )?( Normal27 ):( NormalPuddles459 )) ), dot( tanToWorld2, (( _UseMainNormalMapasNormalDirection )?( Normal27 ):( NormalPuddles459 )) ) ) ) );
				float4 texCUBENode27_g888 = texCUBElod( _Cubemap, float4( worldRefl28_g888, _BlurReflection) );
				float clampResult39_g888 = clamp( _ReflectionIntensity , 0.0 , 100.0 );
				float4 Emission110 = ( (( _UseEmissionFromMainProperties )?( temp_output_503_0 ):( lerpResult394 )) + ( ( texCUBENode27_g888 * ( texCUBENode27_g888.a * clampResult39_g888 ) * _ColorCubemap1 ) * Mask158 ) );
				
				
				float3 Albedo = Albedo26.rgb;
				float3 Emission = Emission110.rgb;
				float Alpha = 1;
				float AlphaClipThreshold = 0.5;

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				MetaInput metaInput = (MetaInput)0;
				metaInput.Albedo = Albedo;
				metaInput.Emission = Emission;
				
				return MetaFragment(metaInput);
			}
			ENDHLSL
		}

		
		Pass
		{
			
			Name "Universal2D"
			Tags { "LightMode"="Universal2D" }

			Blend One Zero, One Zero
			ZWrite On
			ZTest LEqual
			Offset 0 , 0
			ColorMask RGBA

			HLSLPROGRAM
			
			#define _NORMAL_DROPOFF_TS 1
			#pragma multi_compile_instancing
			#pragma multi_compile _ LOD_FADE_CROSSFADE
			#pragma multi_compile_fog
			#define ASE_FOG 1
			#define _EMISSION
			#define _NORMALMAP 1
			#define ASE_SRP_VERSION 70301

			
			#pragma vertex vert
			#pragma fragment frag

			#define SHADERPASS_2D

			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/Color.hlsl"
			#include "Packages/com.unity.render-pipelines.core/ShaderLibrary/UnityInstancing.hlsl"
			#include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/ShaderGraphFunctions.hlsl"
			
			#define ASE_NEEDS_VERT_NORMAL
			#define ASE_NEEDS_FRAG_WORLD_POSITION
			#pragma shader_feature_local _USEMAINALBEDOMAP_ON
			#pragma shader_feature_local _SOURCEMASK_DETAILMASK _SOURCEMASK_BASECOLORALPHA _SOURCEMASK_BOTH
			#pragma shader_feature_local _CHANNELSELECTIONMASK_RED _CHANNELSELECTIONMASK_GREEN _CHANNELSELECTIONMASK_BLUE _CHANNELSELECTIONMASK_ALPHA


			#pragma shader_feature _ _SMOOTHNESS_TEXTURE_ALBEDO_CHANNEL_A

			struct VertexInput
			{
				float4 vertex : POSITION;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;
				float4 ase_tangent : TANGENT;
				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct VertexOutput
			{
				float4 clipPos : SV_POSITION;
				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 worldPos : TEXCOORD0;
				#endif
				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
				float4 shadowCoord : TEXCOORD1;
				#endif
				float4 ase_texcoord2 : TEXCOORD2;
				float4 ase_texcoord3 : TEXCOORD3;
				float4 ase_texcoord4 : TEXCOORD4;
				float4 ase_texcoord5 : TEXCOORD5;
				UNITY_VERTEX_INPUT_INSTANCE_ID
				UNITY_VERTEX_OUTPUT_STEREO
			};

			CBUFFER_START(UnityPerMaterial)
			float4 _DetailAlbedoMap_ST;
			float4 _EmissionColor;
			float4 _Color;
			float4 _BaseMap_ST;
			float4 _GlobalXYTilingXYZWOffsetXY;
			float4 _MetallicGlossMapMAHS_ST;
			float4 _ColorCubemap1;
			float4 _BumpMap_ST;
			float4 _DetailNormalMap_ST;
			float4 _DetailColor;
			float4 _EmissionMap_ST;
			float4 _DetailMask_ST;
			float4 _XColumnsYRowsZSpeedWStrartFrameNormal;
			float2 _OffsetFBDetailsNormal;
			float _UseEmissionFromMainProperties;
			float _RotationAngle2;
			float _TilingWave2;
			float _TranslationSpeed2;
			float _NormalWaveIntensity1;
			float _NormalWaveIntensity2;
			float _EmissionIntensity;
			float _VisualizeMask;
			float _BlurReflection;
			float _ReflectionIntensity;
			float _Metallic;
			float _Metallic2;
			float _Smoothness;
			float _SplashSpeed;
			float _Tiling;
			float _Size;
			float _Intensity;
			float _Glossiness2;
			float _UseMainNormalMapasNormalDirection;
			float _RotationAngle1;
			float _BlendMainNormal;
			float _TranslationSpeed1;
			float _Brightness;
			float _Parallax;
			float _Saturation;
			float _DetailBrightness;
			float _DuplicateTextureAtlasNormal;
			float _FlipBTilingNormal;
			float _IntensityScaleNormal1;
			float _ScaleFBDetailsNormal;
			float _FBDetailsNormal;
			float _TilingWave1;
			float _IntensityScaleNormal2;
			float _DetailSaturation;
			float _ExcludeMask;
			float _ContrastDetailMap;
			float _InvertMask;
			float _InvertABaseColor;
			float _SpreadDetailMap;
			float _IntensityMask;
			float _UseAoFromMainProperties;
			float _BumpScale;
			float _Distortion;
			float _AoIntensity;
			#ifdef _TRANSMISSION_ASE
				float _TransmissionShadow;
			#endif
			#ifdef _TRANSLUCENCY_ASE
				float _TransStrength;
				float _TransNormal;
				float _TransScattering;
				float _TransDirect;
				float _TransAmbient;
				float _TransShadow;
			#endif
			#ifdef TESSELLATION_ON
				float _TessPhongStrength;
				float _TessValue;
				float _TessMin;
				float _TessMax;
				float _TessEdgeLength;
				float _TessMaxDisp;
			#endif
			CBUFFER_END
			sampler2D _BaseMap;
			sampler2D _MetallicGlossMapMAHS;
			sampler2D _DetailAlbedoMap;
			sampler2D _TextureAtlasNormal;
			sampler2D _DetailMask;


			inline float2 ParallaxOffset( half h, half height, half3 viewDir )
			{
				h = h * height - height/2.0;
				float3 v = normalize( viewDir );
				v.z += 0.42;
				return h* (v.xy / v.z);
			}
			
			float4 CalculateContrast( float contrastValue, float4 colorTarget )
			{
				float t = 0.5 * ( 1.0 - contrastValue );
				return mul( float4x4( contrastValue,0,0,t, 0,contrastValue,0,t, 0,0,contrastValue,t, 0,0,0,1 ), colorTarget );
			}

			VertexOutput VertexFunction( VertexInput v  )
			{
				VertexOutput o = (VertexOutput)0;
				UNITY_SETUP_INSTANCE_ID( v );
				UNITY_TRANSFER_INSTANCE_ID( v, o );
				UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO( o );

				float3 ase_worldTangent = TransformObjectToWorldDir(v.ase_tangent.xyz);
				o.ase_texcoord3.xyz = ase_worldTangent;
				float3 ase_worldNormal = TransformObjectToWorldNormal(v.ase_normal);
				o.ase_texcoord4.xyz = ase_worldNormal;
				float ase_vertexTangentSign = v.ase_tangent.w * unity_WorldTransformParams.w;
				float3 ase_worldBitangent = cross( ase_worldNormal, ase_worldTangent ) * ase_vertexTangentSign;
				o.ase_texcoord5.xyz = ase_worldBitangent;
				
				o.ase_texcoord2.xy = v.ase_texcoord.xy;
				
				//setting value to unused interpolator channels and avoid initialization warnings
				o.ase_texcoord2.zw = 0;
				o.ase_texcoord3.w = 0;
				o.ase_texcoord4.w = 0;
				o.ase_texcoord5.w = 0;
				
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					float3 defaultVertexValue = v.vertex.xyz;
				#else
					float3 defaultVertexValue = float3(0, 0, 0);
				#endif
				float3 vertexValue = defaultVertexValue;
				#ifdef ASE_ABSOLUTE_VERTEX_POS
					v.vertex.xyz = vertexValue;
				#else
					v.vertex.xyz += vertexValue;
				#endif

				v.ase_normal = v.ase_normal;

				float3 positionWS = TransformObjectToWorld( v.vertex.xyz );
				float4 positionCS = TransformWorldToHClip( positionWS );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				o.worldPos = positionWS;
				#endif

				#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR) && defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					VertexPositionInputs vertexInput = (VertexPositionInputs)0;
					vertexInput.positionWS = positionWS;
					vertexInput.positionCS = positionCS;
					o.shadowCoord = GetShadowCoord( vertexInput );
				#endif

				o.clipPos = positionCS;
				return o;
			}

			#if defined(TESSELLATION_ON)
			struct VertexControl
			{
				float4 vertex : INTERNALTESSPOS;
				float3 ase_normal : NORMAL;
				float4 ase_texcoord : TEXCOORD0;
				float4 ase_tangent : TANGENT;

				UNITY_VERTEX_INPUT_INSTANCE_ID
			};

			struct TessellationFactors
			{
				float edge[3] : SV_TessFactor;
				float inside : SV_InsideTessFactor;
			};

			VertexControl vert ( VertexInput v )
			{
				VertexControl o;
				UNITY_SETUP_INSTANCE_ID(v);
				UNITY_TRANSFER_INSTANCE_ID(v, o);
				o.vertex = v.vertex;
				o.ase_normal = v.ase_normal;
				o.ase_texcoord = v.ase_texcoord;
				o.ase_tangent = v.ase_tangent;
				return o;
			}

			TessellationFactors TessellationFunction (InputPatch<VertexControl,3> v)
			{
				TessellationFactors o;
				float4 tf = 1;
				float tessValue = _TessValue; float tessMin = _TessMin; float tessMax = _TessMax;
				float edgeLength = _TessEdgeLength; float tessMaxDisp = _TessMaxDisp;
				#if defined(ASE_FIXED_TESSELLATION)
				tf = FixedTess( tessValue );
				#elif defined(ASE_DISTANCE_TESSELLATION)
				tf = DistanceBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, tessValue, tessMin, tessMax, GetObjectToWorldMatrix(), _WorldSpaceCameraPos );
				#elif defined(ASE_LENGTH_TESSELLATION)
				tf = EdgeLengthBasedTess(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams );
				#elif defined(ASE_LENGTH_CULL_TESSELLATION)
				tf = EdgeLengthBasedTessCull(v[0].vertex, v[1].vertex, v[2].vertex, edgeLength, tessMaxDisp, GetObjectToWorldMatrix(), _WorldSpaceCameraPos, _ScreenParams, unity_CameraWorldClipPlanes );
				#endif
				o.edge[0] = tf.x; o.edge[1] = tf.y; o.edge[2] = tf.z; o.inside = tf.w;
				return o;
			}

			[domain("tri")]
			[partitioning("fractional_odd")]
			[outputtopology("triangle_cw")]
			[patchconstantfunc("TessellationFunction")]
			[outputcontrolpoints(3)]
			VertexControl HullFunction(InputPatch<VertexControl, 3> patch, uint id : SV_OutputControlPointID)
			{
			   return patch[id];
			}

			[domain("tri")]
			VertexOutput DomainFunction(TessellationFactors factors, OutputPatch<VertexControl, 3> patch, float3 bary : SV_DomainLocation)
			{
				VertexInput o = (VertexInput) 0;
				o.vertex = patch[0].vertex * bary.x + patch[1].vertex * bary.y + patch[2].vertex * bary.z;
				o.ase_normal = patch[0].ase_normal * bary.x + patch[1].ase_normal * bary.y + patch[2].ase_normal * bary.z;
				o.ase_texcoord = patch[0].ase_texcoord * bary.x + patch[1].ase_texcoord * bary.y + patch[2].ase_texcoord * bary.z;
				o.ase_tangent = patch[0].ase_tangent * bary.x + patch[1].ase_tangent * bary.y + patch[2].ase_tangent * bary.z;
				#if defined(ASE_PHONG_TESSELLATION)
				float3 pp[3];
				for (int i = 0; i < 3; ++i)
					pp[i] = o.vertex.xyz - patch[i].ase_normal * (dot(o.vertex.xyz, patch[i].ase_normal) - dot(patch[i].vertex.xyz, patch[i].ase_normal));
				float phongStrength = _TessPhongStrength;
				o.vertex.xyz = phongStrength * (pp[0]*bary.x + pp[1]*bary.y + pp[2]*bary.z) + (1.0f-phongStrength) * o.vertex.xyz;
				#endif
				UNITY_TRANSFER_INSTANCE_ID(patch[0], o);
				return VertexFunction(o);
			}
			#else
			VertexOutput vert ( VertexInput v )
			{
				return VertexFunction( v );
			}
			#endif

			half4 frag(VertexOutput IN , FRONT_FACE_TYPE ase_vface : FRONT_FACE_SEMANTIC ) : SV_TARGET
			{
				UNITY_SETUP_INSTANCE_ID( IN );
				UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX( IN );

				#if defined(ASE_NEEDS_FRAG_WORLD_POSITION)
				float3 WorldPosition = IN.worldPos;
				#endif
				float4 ShadowCoords = float4( 0, 0, 0, 0 );

				#if defined(ASE_NEEDS_FRAG_SHADOWCOORDS)
					#if defined(REQUIRES_VERTEX_SHADOW_COORD_INTERPOLATOR)
						ShadowCoords = IN.shadowCoord;
					#elif defined(MAIN_LIGHT_CALCULATE_SHADOWS)
						ShadowCoords = TransformWorldToShadowCoord( WorldPosition );
					#endif
				#endif

				float2 uv_BaseMap = IN.ase_texcoord2.xy * _BaseMap_ST.xy + _BaseMap_ST.zw;
				float2 break26_g854 = uv_BaseMap;
				float GlobalTilingX11 = ( _GlobalXYTilingXYZWOffsetXY.x - 1.0 );
				float GlobalTilingY8 = ( _GlobalXYTilingXYZWOffsetXY.y - 1.0 );
				float2 appendResult14_g854 = (float2(( break26_g854.x * GlobalTilingX11 ) , ( break26_g854.y * GlobalTilingY8 )));
				float GlobalOffsetX10 = _GlobalXYTilingXYZWOffsetXY.z;
				float GlobalOffsetY9 = _GlobalXYTilingXYZWOffsetXY.w;
				float2 appendResult13_g854 = (float2(( break26_g854.x + GlobalOffsetX10 ) , ( break26_g854.y + GlobalOffsetY9 )));
				float2 uv_MetallicGlossMapMAHS = IN.ase_texcoord2.xy * _MetallicGlossMapMAHS_ST.xy + _MetallicGlossMapMAHS_ST.zw;
				float2 break26_g831 = uv_MetallicGlossMapMAHS;
				float2 appendResult14_g831 = (float2(( break26_g831.x * GlobalTilingX11 ) , ( break26_g831.y * GlobalTilingY8 )));
				float2 appendResult13_g831 = (float2(( break26_g831.x + GlobalOffsetX10 ) , ( break26_g831.y + GlobalOffsetY9 )));
				float4 tex2DNode3_g830 = tex2D( _MetallicGlossMapMAHS, ( ( appendResult14_g831 + appendResult13_g831 ) + float2( 0,0 ) ) );
				float3 ase_worldTangent = IN.ase_texcoord3.xyz;
				float3 ase_worldNormal = IN.ase_texcoord4.xyz;
				float3 ase_worldBitangent = IN.ase_texcoord5.xyz;
				float3 tanToWorld0 = float3( ase_worldTangent.x, ase_worldBitangent.x, ase_worldNormal.x );
				float3 tanToWorld1 = float3( ase_worldTangent.y, ase_worldBitangent.y, ase_worldNormal.y );
				float3 tanToWorld2 = float3( ase_worldTangent.z, ase_worldBitangent.z, ase_worldNormal.z );
				float3 ase_worldViewDir = ( _WorldSpaceCameraPos.xyz - WorldPosition );
				ase_worldViewDir = normalize(ase_worldViewDir);
				float3 ase_tanViewDir =  tanToWorld0 * ase_worldViewDir.x + tanToWorld1 * ase_worldViewDir.y  + tanToWorld2 * ase_worldViewDir.z;
				ase_tanViewDir = normalize(ase_tanViewDir);
				float2 paralaxOffset38_g830 = ParallaxOffset( tex2DNode3_g830.b , _Parallax , ase_tanViewDir );
				float2 switchResult37_g830 = (((ase_vface>0)?(paralaxOffset38_g830):(0.0)));
				float2 Parallaxe494 = switchResult37_g830;
				float4 tex2DNode7_g853 = tex2D( _BaseMap, ( ( appendResult14_g854 + appendResult13_g854 ) + Parallaxe494 ) );
				float clampResult27_g853 = clamp( _Saturation , -1.0 , 100.0 );
				float3 desaturateInitialColor29_g853 = ( _Color * tex2DNode7_g853 ).rgb;
				float desaturateDot29_g853 = dot( desaturateInitialColor29_g853, float3( 0.299, 0.587, 0.114 ));
				float3 desaturateVar29_g853 = lerp( desaturateInitialColor29_g853, desaturateDot29_g853.xxx, -clampResult27_g853 );
				float4 temp_output_764_0 = CalculateContrast(_Brightness,float4( desaturateVar29_g853 , 0.0 ));
				float2 uv_DetailAlbedoMap = IN.ase_texcoord2.xy * _DetailAlbedoMap_ST.xy + _DetailAlbedoMap_ST.zw;
				float FlipBookTiling100_g852 = _FlipBTilingNormal;
				float2 temp_cast_2 = (FlipBookTiling100_g852).xx;
				float2 texCoord114_g852 = IN.ase_texcoord2.xy * temp_cast_2 + float2( 0,0 );
				float2 appendResult124_g852 = (float2(frac( texCoord114_g852.x ) , frac( texCoord114_g852.y )));
				float XColumns119_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.x;
				float YRows118_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.y;
				float ZSpeed121_g852 = ( ( _XColumnsYRowsZSpeedWStrartFrameNormal.x * _XColumnsYRowsZSpeedWStrartFrameNormal.y ) * _XColumnsYRowsZSpeedWStrartFrameNormal.z );
				float WStrartFrame122_g852 = _XColumnsYRowsZSpeedWStrartFrameNormal.w;
				// *** BEGIN Flipbook UV Animation vars ***
				// Total tiles of Flipbook Texture
				float fbtotaltiles130_g852 = XColumns119_g852 * YRows118_g852;
				// Offsets for cols and rows of Flipbook Texture
				float fbcolsoffset130_g852 = 1.0f / XColumns119_g852;
				float fbrowsoffset130_g852 = 1.0f / YRows118_g852;
				// Speed of animation
				float fbspeed130_g852 = _Time[ 1 ] * ZSpeed121_g852;
				// UV Tiling (col and row offset)
				float2 fbtiling130_g852 = float2(fbcolsoffset130_g852, fbrowsoffset130_g852);
				// UV Offset - calculate current tile linear index, and convert it to (X * coloffset, Y * rowoffset)
				// Calculate current tile linear index
				float fbcurrenttileindex130_g852 = round( fmod( fbspeed130_g852 + WStrartFrame122_g852, fbtotaltiles130_g852) );
				fbcurrenttileindex130_g852 += ( fbcurrenttileindex130_g852 < 0) ? fbtotaltiles130_g852 : 0;
				// Obtain Offset X coordinate from current tile linear index
				float fblinearindextox130_g852 = round ( fmod ( fbcurrenttileindex130_g852, XColumns119_g852 ) );
				// Multiply Offset X by coloffset
				float fboffsetx130_g852 = fblinearindextox130_g852 * fbcolsoffset130_g852;
				// Obtain Offset Y coordinate from current tile linear index
				float fblinearindextoy130_g852 = round( fmod( ( fbcurrenttileindex130_g852 - fblinearindextox130_g852 ) / XColumns119_g852, YRows118_g852 ) );
				// Reverse Y to get tiles from Top to Bottom
				fblinearindextoy130_g852 = (int)(YRows118_g852-1) - fblinearindextoy130_g852;
				// Multiply Offset Y by rowoffset
				float fboffsety130_g852 = fblinearindextoy130_g852 * fbrowsoffset130_g852;
				// UV Offset
				float2 fboffset130_g852 = float2(fboffsetx130_g852, fboffsety130_g852);
				// Flipbook UV
				half2 fbuv130_g852 = appendResult124_g852 * fbtiling130_g852 + fboffset130_g852;
				// *** END Flipbook UV Animation vars ***
				float3 unpack132_g852 = UnpackNormalScale( tex2D( _TextureAtlasNormal, fbuv130_g852 ), _IntensityScaleNormal1 );
				unpack132_g852.z = lerp( 1, unpack132_g852.z, saturate(_IntensityScaleNormal1) );
				float3 tex2DNode132_g852 = unpack132_g852;
				float2 temp_cast_3 = (( FlipBookTiling100_g852 / _ScaleFBDetailsNormal )).xx;
				float2 texCoord105_g852 = IN.ase_texcoord2.xy * temp_cast_3 + _OffsetFBDetailsNormal;
				float cos111_g852 = cos( _FBDetailsNormal );
				float sin111_g852 = sin( _FBDetailsNormal );
				float2 rotator111_g852 = mul( texCoord105_g852 - float2( 0.5,0.5 ) , float2x2( cos111_g852 , -sin111_g852 , sin111_g852 , cos111_g852 )) + float2( 0.5,0.5 );
				float2 break112_g852 = rotator111_g852;
				float2 appendResult123_g852 = (float2(frac( break112_g852.x ) , frac( break112_g852.y )));
				float fbtotaltiles129_g852 = XColumns119_g852 * YRows118_g852;
				float fbcolsoffset129_g852 = 1.0f / XColumns119_g852;
				float fbrowsoffset129_g852 = 1.0f / YRows118_g852;
				float fbspeed129_g852 = _Time[ 1 ] * ZSpeed121_g852;
				float2 fbtiling129_g852 = float2(fbcolsoffset129_g852, fbrowsoffset129_g852);
				float fbcurrenttileindex129_g852 = round( fmod( fbspeed129_g852 + WStrartFrame122_g852, fbtotaltiles129_g852) );
				fbcurrenttileindex129_g852 += ( fbcurrenttileindex129_g852 < 0) ? fbtotaltiles129_g852 : 0;
				float fblinearindextox129_g852 = round ( fmod ( fbcurrenttileindex129_g852, XColumns119_g852 ) );
				float fboffsetx129_g852 = fblinearindextox129_g852 * fbcolsoffset129_g852;
				float fblinearindextoy129_g852 = round( fmod( ( fbcurrenttileindex129_g852 - fblinearindextox129_g852 ) / XColumns119_g852, YRows118_g852 ) );
				fblinearindextoy129_g852 = (int)(YRows118_g852-1) - fblinearindextoy129_g852;
				float fboffsety129_g852 = fblinearindextoy129_g852 * fbrowsoffset129_g852;
				float2 fboffset129_g852 = float2(fboffsetx129_g852, fboffsety129_g852);
				half2 fbuv129_g852 = appendResult123_g852 * fbtiling129_g852 + fboffset129_g852;
				float3 unpack134_g852 = UnpackNormalScale( tex2D( _TextureAtlasNormal, fbuv129_g852 ), _IntensityScaleNormal2 );
				unpack134_g852.z = lerp( 1, unpack134_g852.z, saturate(_IntensityScaleNormal2) );
				float3 RainDotsNormal739 = (( _DuplicateTextureAtlasNormal )?( BlendNormal( tex2DNode132_g852 , unpack134_g852 ) ):( tex2DNode132_g852 ));
				float lerpResult777 = lerp( 0.0 , 0.9 , _Distortion);
				float2 temp_cast_5 = (lerpResult777).xx;
				float4 tex2DNode7_g872 = tex2D( _DetailAlbedoMap, ( uv_DetailAlbedoMap + ( RainDotsNormal739.xy * temp_cast_5 ) ) );
				float4 BaseColorRGB403 = temp_output_764_0;
				#ifdef _USEMAINALBEDOMAP_ON
				float4 staticSwitch38_g872 = BaseColorRGB403;
				#else
				float4 staticSwitch38_g872 = tex2DNode7_g872;
				#endif
				float clampResult27_g872 = clamp( _DetailSaturation , -1.0 , 100.0 );
				float3 desaturateInitialColor29_g872 = ( _DetailColor * staticSwitch38_g872 ).rgb;
				float desaturateDot29_g872 = dot( desaturateInitialColor29_g872, float3( 0.299, 0.587, 0.114 ));
				float3 desaturateVar29_g872 = lerp( desaturateInitialColor29_g872, desaturateDot29_g872.xxx, -clampResult27_g872 );
				float4 AlbedoDetail153 = CalculateContrast(_DetailBrightness,float4( desaturateVar29_g872 , 0.0 ));
				float2 uv_DetailMask = IN.ase_texcoord2.xy * _DetailMask_ST.xy + _DetailMask_ST.zw;
				float4 tex2DNode27_g871 = tex2D( _DetailMask, uv_DetailMask );
				#if defined(_CHANNELSELECTIONMASK_RED)
				float staticSwitch28_g871 = tex2DNode27_g871.r;
				#elif defined(_CHANNELSELECTIONMASK_GREEN)
				float staticSwitch28_g871 = tex2DNode27_g871.g;
				#elif defined(_CHANNELSELECTIONMASK_BLUE)
				float staticSwitch28_g871 = tex2DNode27_g871.b;
				#elif defined(_CHANNELSELECTIONMASK_ALPHA)
				float staticSwitch28_g871 = tex2DNode27_g871.a;
				#else
				float staticSwitch28_g871 = tex2DNode27_g871.r;
				#endif
				float BaseColorAlpha46 = (( _InvertABaseColor )?( ( 1.0 - tex2DNode7_g853.a ) ):( tex2DNode7_g853.a ));
				float temp_output_43_0_g871 = BaseColorAlpha46;
				#if defined(_SOURCEMASK_DETAILMASK)
				float staticSwitch45_g871 = (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 ));
				#elif defined(_SOURCEMASK_BASECOLORALPHA)
				float staticSwitch45_g871 = temp_output_43_0_g871;
				#elif defined(_SOURCEMASK_BOTH)
				float staticSwitch45_g871 = ( (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 )) + temp_output_43_0_g871 );
				#else
				float staticSwitch45_g871 = (( _InvertMask )?( ( 1.0 - staticSwitch28_g871 ) ):( staticSwitch28_g871 ));
				#endif
				float4 temp_cast_10 = (( staticSwitch45_g871 + (-1.2 + (_SpreadDetailMap - 0.0) * (0.7 - -1.2) / (1.0 - 0.0)) )).xxxx;
				float4 temp_output_35_0_g871 = CalculateContrast(( _ContrastDetailMap + 1.0 ),temp_cast_10);
				float4 clampResult38_g871 = clamp( (( _ExcludeMask )?( ( temp_output_35_0_g871 * ( 1.0 - temp_output_43_0_g871 ) ) ):( temp_output_35_0_g871 )) , float4( 0,0,0,0 ) , float4( 1,1,1,0 ) );
				float4 Mask158 = ( clampResult38_g871 * _IntensityMask );
				float4 lerpResult365 = lerp( temp_output_764_0 , AlbedoDetail153 , Mask158);
				float4 Albedo26 = (( _VisualizeMask )?( Mask158 ):( lerpResult365 ));
				
				
				float3 Albedo = Albedo26.rgb;
				float Alpha = 1;
				float AlphaClipThreshold = 0.5;

				half4 color = half4( Albedo, Alpha );

				#ifdef _ALPHATEST_ON
					clip(Alpha - AlphaClipThreshold);
				#endif

				return color;
			}
			ENDHLSL
		}
		
	}
	CustomEditor "UnityEditor.ShaderGraph.PBRMasterGUI"
	Fallback "Hidden/InternalErrorShader"
	
}