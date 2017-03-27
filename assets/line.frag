#version 150

in VertexData{
	vec2 mTexCoord;
	vec3 mColor;
} VertexIn;

out vec4 oColor;

void main(void)
{
	oColor.rgb = VertexIn.mColor;
	oColor.a = 1.0;
}