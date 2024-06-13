# version 410

in vec3 normal;
in vec4 worldPosition;
in vec2 st;

struct PointLight {
    vec3 position;
    vec3 color;
};

struct Material{
    vec3 diffuseColor;
    vec3 specularColor;
    float ka;
    float kd;
    float ks;
    float shininess;
};

uniform vec3 ambientLightColor; 
uniform vec3 cameraPosition;   
uniform Material material;

// FIXED LIGHT: Maybe add more lights
int numLights = 3;
PointLight pointLight[3] = PointLight[3](PointLight(vec3(3.25, 1.5, 2.5), vec3(1.0, 1.0, 1.0)),
                                         PointLight(vec3(0.75, 2.5, -2.5), vec3(1.0, 1.0, 1.0)),
                                         PointLight(vec3(-3.0, -5, 0.5), vec3(1.0, 1.0, 1.0)));

// FIXED MATERIAL:
vec3 color = vec3(1.0, 1.0, 1.0);
vec3 specularColor = vec3(1.0, 0.85, 0.85);
float ka = 0.3;
float kd = 0.5;
float ks = 0.8;
float shininess = 32.0;

out vec4 fragmentColor;

vec3 ComputeLambertian(const in vec3 dir_to_light, const in vec3 lightColor, const in vec3 normal, const in vec3 diffuseColor) {
        float nDotL = dot(normal, dir_to_light);         
        vec3 lambert = diffuseColor * lightColor * max (nDotL, 0.0); 
        return lambert;            
} 

vec3 ComputeSpecular(const in vec3 dir_to_light, const in vec3 dir_to_camera, const in vec3 lightColor, const in vec3 normal, const in vec3 specularColor, const in float shininess){
    vec3 r = 2 * dot(dir_to_light, normal) * normal - dir_to_light;

    float cosine = max(dot(dir_to_camera, r), 0.0); //if cosine is <0 then lock it to 0

    vec3 spec = lightColor * specularColor * pow(cosine, shininess); 
    return spec;
}

void main()                                                              
{
    vec3 ambient_color = material.ka*material.diffuseColor;
    vec3 diffuse_color = material.kd*material.diffuseColor;
    vec3 specular_color = material.ks*material.specularColor;

    vec3 myColor = ambientLightColor*ambient_color; 
    vec3 norm = normalize(normal);

    for (int i = 0; i < numLights; i++){
	   	vec3 dir_to_light = normalize(pointLight[i].position.xyz - worldPosition.xyz);
   		myColor += ComputeLambertian(dir_to_light, pointLight[i].color, norm, diffuse_color);

        vec3 dir_to_camera = normalize(cameraPosition - vec3(worldPosition));
        myColor += ComputeSpecular(dir_to_light, dir_to_camera, pointLight[i].color, norm, specular_color, material.shininess);
    }     

    fragmentColor = vec4(myColor, 1.0);
}