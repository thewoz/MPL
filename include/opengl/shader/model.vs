#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 texCoords;

// uniform variable
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform mat4 lightSpaceMatrix;

// output variable
out vec3 fragPos;
out vec3 fragNormal;
out vec2 fragTexCoord;

out vec3 fragPosLightSpace;

//*******************************
// vertex shader
//*******************************
void main() {
  
  // coordinata proiettiva del punto
  gl_Position = projection * view * model * vec4(position, 1.0f);
  
  // valori di usciti del fragment
  fragPos      = vec3(view * model * vec4(position, 1.0f));
  fragNormal   = vec3(normalize(view * model * vec4(normal, 0.0)));
  fragTexCoord = texCoords;

  // posizione in light space
  fragPosLightSpace = vec3(lightSpaceMatrix * vec4(position, 1.0));

}
