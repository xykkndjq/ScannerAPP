attribute highp vec3 aPos;
attribute highp vec4 aColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform vec3 zoom;
uniform mat4 model2;
varying vec4 bColor;

void main(void) {
bColor = aColor;
vec3 bPos = vec3(aPos.x*zoom.x,aPos.y*zoom.y,aPos.z*zoom.z);
	gl_Position = projection * view * model * vec4(bPos, 1.0);
	//gl_Position = vec4(aPos * 10, 1.0);
}