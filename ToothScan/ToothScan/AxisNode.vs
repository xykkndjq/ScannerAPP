attribute highp vec3 inPosition;
attribute highp vec3 inColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

varying highp vec3 passColor;

void main() {
	gl_Position = projection * view * model * vec4(inPosition, 1.0);
    passColor = inColor;
}