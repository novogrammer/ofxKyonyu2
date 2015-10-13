#version 120

uniform float SizePerZ;

void main()
{
    gl_FrontColor=gl_Color;
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;
    gl_PointSize=gl_Vertex.z*SizePerZ/gl_Position.w;
}