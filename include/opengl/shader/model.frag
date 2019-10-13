#version 330 core

/*****************************************************************************/
// Material
/*****************************************************************************/
struct Material {
  
  sampler2D diffuseTexture;
  sampler2D specularTexture;
  sampler2D ambientTexture;
  sampler2D emissimeTexture;
  sampler2D heightTexture;
  sampler2D normalsTexture;
  sampler2D shininessTexture;
  sampler2D opacityTexture;
  sampler2D displactemenTexture;
  sampler2D lightTexture;
  sampler2D reflectionTexture;
  
  bool haveDiffuseTexture;
  bool haveSpecularTexture;
  bool haveAmbientTexture;
  bool haveEmissimeTexture;
  bool haveHeightTexture;
  bool haveNormalsTexture;
  bool haveShininessTexture;
  bool haveOpacityTexture;
  bool haveDisplactemenTexture;
  bool haveLightTexture;
  bool haveReflectionTexture;
  
  vec3 emissiveColor;
  vec3 ambientColor;
  vec3 diffuseColor;
  vec3 specularColor;
  vec3 reflectiveColor;
  vec3 trasparentColor;
  
  float shininess;
  float refraction;
  float opacity;
  
};

/*****************************************************************************/
// Light
/*****************************************************************************/
struct Light {
  
  vec3 direction;
  vec3 position;

  vec3 ambient;
  vec3 diffuse;
  vec3 specular;
  
};

/*****************************************************************************/
// uniform variables
/*****************************************************************************/
uniform Material material;
uniform Light light;
//uniform vec3 cameraPosition;
uniform sampler2D shadowMap;
uniform mat4 depthBiasMVP;
uniform bool withShadow;

/*****************************************************************************/
// in variables
/*****************************************************************************/
in vec2 fragTexCoord;
in vec3 fragNormal;
in vec3 fragPos;
in vec3 fragPosLightSpace;

/*****************************************************************************/
// out variables
/*****************************************************************************/
out vec4 color;

/*****************************************************************************/
// Function prototypes
/*****************************************************************************/
float shadowCalculation(vec3 position, vec3 positionLight);

// Assume the monitor is calibrated to the sRGB color space
const float screenGamma = 1.0;

//https://lwjglgamedev.gitbooks.io/3d-game-development-with-lwjgl/content/chapter10/chapter10.html
/*****************************************************************************/
// main
/*****************************************************************************/
void main() {
  
  vec3 light_direction = normalize(light.position-fragPos);
  //vec3 light_direction = normalize(light.direction);

  float diffuseFactor = max(dot(light_direction, fragNormal), 0.0);
  
  // Specular shading
  vec3 camera_direction = normalize(-fragPos);
  vec3 reflected_light  = normalize(reflect(-camera_direction, fragNormal));
  float specularFactor = pow(max(dot(camera_direction, reflected_light), 0.0), material.shininess);
  
  // Get base colors
  vec3 ambientBaseColor  = (material.haveAmbientTexture)  ? vec3(texture(material.diffuseTexture,  fragTexCoord)) : material.ambientColor;
  vec3 diffuseBaseColor  = (material.haveDiffuseTexture)  ? vec3(texture(material.diffuseTexture,  fragTexCoord)) : material.diffuseColor;
  vec3 specularBaseColor = (material.haveSpecularTexture) ? vec3(texture(material.specularTexture, fragTexCoord)) : material.specularColor;
  
  // Combine results
  vec3 ambient  = light.ambient                   * ambientBaseColor;
  vec3 diffuse  = light.diffuse  * diffuseFactor  * diffuseBaseColor;
  vec3 specular = light.specular * specularFactor * specularBaseColor;
  
  if(withShadow) {
  
    float shadow = shadowCalculation(fragPos, fragPosLightSpace);
  
    //vec3 linearColor = (ambient + (1.0 - shadow) * (diffuse + specular));
    //color = vec4(linearColor, 1.0f);

    if(shadow == 1){
      
      color = vec4(1.0,0.0,0.0,1.0f);
      
    } else {
  
      vec3 linearColor = (ambient + (1.0 - shadow) * (diffuse + specular));
    
      color = vec4(linearColor, 1.0f);
      
    }
    
  } else  {
    
    vec3 linearColor = ambient + diffuse + specular;
    
    color = vec4(linearColor, 1.0f);
    
  }
  
}

/*****************************************************************************/
// shadowCalculation
/*****************************************************************************/
float shadowCalculation(vec3 position, vec3 positionLight) {
  
  float shadowFactor = 1.0;
  
  vec3 projCoords = position.xyz;

  // Transform from screen coordinates to texture coordinates
  projCoords = projCoords * 0.5 + 0.5;

  if(projCoords.z < texture(shadowMap, projCoords.xy).r) {
    // Current fragment is not in shade
    shadowFactor = 0;
  }
  
  if(position.z < positionLight.z){
    shadowFactor = 0;
  }
  
  return 1.0 - shadowFactor;
  
}
