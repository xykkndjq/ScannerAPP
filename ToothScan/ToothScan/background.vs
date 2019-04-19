attribute highp vec4 vertex;
attribute mediump vec4 texCoord;
varying mediump vec4 texc;

void main(void) {
	gl_Position = vertex;
	texc = texCoord;
}