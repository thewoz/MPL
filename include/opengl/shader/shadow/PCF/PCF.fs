#version 330 core

in vec4 FragPos;

uniform vec3 lightPos;
uniform float far;

void main() {
  
    float lightDistance = length(FragPos.xyz - lightPos);
    
    // map to [0;1] range by dividing by far_plane
    lightDistance = lightDistance / far;
    
    // write this as modified depth
    gl_FragDepth = lightDistance;
  
}
