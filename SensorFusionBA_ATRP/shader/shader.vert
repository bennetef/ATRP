# version 410

layout(location=0) in vec3 position;
layout(location=1) in vec3 inNormal;
layout(location=2) in vec2 textureCoords;

uniform mat4 viewMatrix, projMatrix, modelMatrix;

out vec3 normal;
out vec4 worldPosition;
out vec2 st;

void main(void){
    st = textureCoords;
    gl_Position = projMatrix * viewMatrix * modelMatrix * vec4(position, 1.0);

    worldPosition = modelMatrix * vec4(position, 1.0);
    normal = normalize(transpose(inverse(mat3(modelMatrix))) * inNormal);
}