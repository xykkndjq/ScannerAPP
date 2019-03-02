attribute highp vec4 aPos;
attribute highp vec3 aNormal;

varying highp vec3 FragPos;
varying highp vec3 Normal;
varying highp float mflag;

uniform highp mat4 model;
uniform highp mat4 view;
uniform highp mat4 projection;

void main()
{
	mflag = aPos.w;
    FragPos = vec3(model * vec4(aPos.xyz, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;      
    gl_Position = projection * view * vec4(FragPos, 1.0);
	//if(aMaterial == 0)
	//	{mflag = 0;}
	//else
	//	{mflag = 1;}
	////mflag = (int)aMaterial;
}