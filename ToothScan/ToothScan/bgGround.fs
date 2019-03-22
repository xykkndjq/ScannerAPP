#version 330 core
out vec4 FragColor;
uniform vec4 ourColor;
void main()
{
	gl_FragColor  = ourColor;//vec4(0.0f,0.6f,0.6f,0.1f);
}