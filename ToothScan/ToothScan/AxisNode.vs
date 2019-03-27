attribute highp vec3 inPosition;
attribute highp vec3 inColor;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

varying highp vec3 passColor;

void main() {
	//gl_Position = projection * view * model * vec4(inPosition, 1.0);
	vec4 temppos = projection * view * model * vec4(inPosition, 1.0);
	gl_Position = temppos + vec4(temppos.w * -0.8 , temppos.w*-0.8,0,0);
    passColor = inColor;
}