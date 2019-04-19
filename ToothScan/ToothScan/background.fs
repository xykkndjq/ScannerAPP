#version 330 core
uniform sampler2D texture;
varying mediump vec4 texc;
void main()
{
	gl_FragColor = texture2D(texture, texc.st);
}