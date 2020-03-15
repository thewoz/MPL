#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec4 in_color;

// uniform variable
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

//uniform mat4 mvp;

out vec4 frag_color;
out vec2 pointCoord;

void main() {
  
  gl_Position = projection * view * model * vec4(position, 1.0f);
  
  gl_PointSize = 30;
  
  pointCoord = gl_Position.xy / gl_Position.w;
  
  frag_color = in_color;

}

