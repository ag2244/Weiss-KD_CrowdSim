#version 120 core
#extension GL_ARB_explicit_attrib_location : require
#extension GL_ARB_explicit_uniform_location : require
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 textureCoords;
uniform vec3 transPosition;

// declare an interface block; see 'Advanced GLSL' for what these are.
out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
	vec3 tPosition;
} vs_out;

void main()
{
    //gl_Position = projection * view * model* vec4(position+transPosition, 1.0);
	vs_out.FragPos = position+transPosition;
    vs_out.Normal = aNormal+transPosition;
    vs_out.TexCoords = textureCoords;
	vs_out.tPosition=transPosition;
    gl_Position = projection * view *model* vec4(position+transPosition, 1.0);
}
