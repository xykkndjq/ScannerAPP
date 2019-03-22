attribute highp vec3 aPos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

void main(void) {
	gl_Position = projection * view * model * vec4(aPos, 1.0);
	//gl_Position = vec4(aPos * 10, 1.0);
}