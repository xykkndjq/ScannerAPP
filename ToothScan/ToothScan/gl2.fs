
struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
}; 

struct DirLight {
    vec3 direction;
	
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

struct PointLight {
    vec3 position;
    
    float constant;
    float linear;
    float quadratic;
	
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

struct SpotLight {
    vec3 position;
    vec3 direction;
    float cutOff;
    float outerCutOff;
  
    float constant;
    float linear;
    float quadratic;
  
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;       
};

#define NR_POINT_LIGHTS 4

varying highp vec3 FragPos;
varying highp vec3 Normal;
varying highp float mflag;
varying highp float sflag;

uniform highp vec3 viewPos;
uniform highp DirLight dirLight;
uniform highp PointLight pointLights[NR_POINT_LIGHTS];
uniform highp SpotLight spotLight;
uniform Material material1;
uniform Material material2;
uniform Material material3;

uniform mediump vec3 lightColor;
uniform mediump vec3 objectColor;

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir);
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir);

void main()
{    
// 属性
    vec3 norm = normalize(Normal);
    vec3 viewDir = normalize(viewPos - FragPos);
    
// 第一阶段：定向光照
    vec3 result = CalcDirLight(dirLight, -norm, viewDir);

// 第二阶段：点光源
   // for(int i = 0; i < NR_POINT_LIGHTS; i++)
       // result += CalcPointLight(pointLights[i], norm, FragPos, viewDir);   
		 
// 第三阶段：聚光
    //result += CalcSpotLight(spotLight, norm, FragPos, viewDir);  
	
	//result *= objectColor;  
    
    gl_FragColor = vec4(result, 1.0);
}

vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir)
{
	Material material;
	if(sflag==0)
	material = material3;
	else
	if(mflag==0)
		{material = material1;}
	else
		{material = material2;}

    vec3 lightDir = normalize(-light.direction);
   
    float diff = max(dot(normal, lightDir), 0.0);

    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

    vec3 ambient = light.ambient * lightColor*material.ambient;
    vec3 diffuse = light.diffuse * diff * lightColor*material.diffuse;
    vec3 specular = light.specular * spec * lightColor*material.specular;
    return (ambient + diffuse + specular);
}

vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
	Material material;
	if(sflag==0)
	material = material3;
	else
	if(mflag==0)
		{material = material1;}
	else
		{material = material2;}
    vec3 lightDir = normalize(light.position - fragPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    
    vec3 ambient = light.ambient * lightColor;
    vec3 diffuse = light.diffuse * diff * lightColor;
    vec3 specular = light.specular * spec * lightColor;
    ambient *= attenuation;
    diffuse *= attenuation;
    specular *= attenuation;
    return (ambient + diffuse + specular);
}

vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
	Material material;
	if(sflag==0)
	material = material3;
	else
	if(mflag==0)
		{material = material1;}
	else
		{material = material2;}
    vec3 lightDir = normalize(light.position - fragPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    

    float theta = dot(lightDir, normalize(-light.direction)); 
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);

    vec3 ambient = light.ambient * lightColor;
    vec3 diffuse = light.diffuse * diff * lightColor;
    vec3 specular = light.specular * spec * lightColor;
    ambient *= attenuation * intensity;
    diffuse *= attenuation * intensity;
    specular *= attenuation * intensity;
    return (ambient + diffuse + specular);
}