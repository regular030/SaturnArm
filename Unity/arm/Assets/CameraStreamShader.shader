Shader "Custom/CameraStreamShader"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
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
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            sampler2D _MainTex;
            
            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }
            
            fixed4 frag (v2f i) : SV_Target
            {
                // Convert RGB565 to RGB888
                fixed4 col;
                uint pixel = tex2D(_MainTex, i.uv).r * 65535;
                col.r = ((pixel >> 11) & 0x1F) / 31.0;
                col.g = ((pixel >> 5) & 0x3F) / 63.0;
                col.b = (pixel & 0x1F) / 31.0;
                col.a = 1.0;
                return col;
            }
            ENDCG
        }
    }
}
