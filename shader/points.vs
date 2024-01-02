# version 330
layout(location=0) in vec3 pos_vertex; 

uniform highp mat4 mvp_matrix;
uniform float Point_Size;

void main(void)
{
    gl_PointSize = Point_Size;
    gl_Position = mvp_matrix * vec4(pos_vertex, 1.0);
}