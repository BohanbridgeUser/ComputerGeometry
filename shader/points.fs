#version 330
uniform highp vec4 color;
out highp vec4 fragment_color;

void main(void) 
{
    fragment_color = color;
}
