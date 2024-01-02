# version 330
layout(location=0) in vec3 vertex;

uniform highp mat4 mvp_matrix;

void main(void)
{
    gl_Position = mvp_matrix * vec4(vertex, 1.0f);
}