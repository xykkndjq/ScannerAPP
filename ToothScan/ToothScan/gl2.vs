attribute highp vec3 aPos;
attribute highp vec3 aNormal;
attribute highp float aMateriala;

varying highp vec3 FragPos;
varying highp vec3 Normal;
varying highp float mflag;
varying highp float sflag;

uniform highp mat4 model;
uniform highp mat4 view;
uniform highp mat4 projection;
uniform highp vec3 screenPos;

void main()
{
	//mflag = aPos.w;
	mflag = aMateriala;
	sflag = -1;
    //FragPos = vec3(model * vec4(aPos.xyz, 1.0));
	FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;    
	vec4 p_1 = projection * view * vec4(FragPos, 1.0);    
	vec4 p_2 = p_1 + vec4(screenPos, 0.0);
	if(p_2.x>0)
	{
		sflag = 0.0;
	}
	else
	{
		sflag = 1.0;
	}
	gl_Position = p_2;
}