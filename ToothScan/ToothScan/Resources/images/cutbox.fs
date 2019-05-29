#version 330 core
out vec4 FragColor;
uniform vec4 ourColor;
varying vec4 bColor;
void main()
{
	gl_FragColor  = bColor;//vec4(0.0f,0.6f,0.6f,0.1f);
}