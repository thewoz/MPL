#version 150

uniform vec3 color;
out vec4 out_color;

void main() {
  
  
  if(gl_Color.w<0.999) discard;
  
  vec3 N;
  N.xy = gl_TexCoord[0].xy*vec2(2.0, -2.0) + vec2(-1.0, 1.0);
  
  float mag = dot(N.xy, N.xy);
  
  if(mag > 1.0) discard;   // kill pixels outside circle
  
  N.z = sqrt(1.0-mag);
  
  vec3 lightDir = vec3(0.577, 0.577, 0.577);
  
  // calculate lighting
  float diffuse = max(0.0, dot(lightDir, N));
  
  gl_FragColor = gl_Color * diffuse;
  
  gl_FragDepth = gl_FragCoord.z;
  
  out_color = vec4(color, 1.0);
  
}
