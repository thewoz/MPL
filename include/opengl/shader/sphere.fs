#version 330 core

in vec4 frag_color;
in vec2 pointCoord;

out vec4 out_color;

  void main() {
    
    if(frag_color.w<0.999) discard;
    
    vec2 cxy = 2.0 * gl_PointCoord - 1.0;
    
    float r = dot(cxy, cxy);
    
    if(r > 1.0) discard;
    
    // calculate lighting
    vec3 pos = vec3(cxy.x,cxy.y,sqrt(1.0-r));
    vec3 lightDir = vec3(0.577, 0.577, 0.577);
    float diffuse = max(0.0, dot(lightDir, pos));
    
    //
    float alpha = 1.0;
    float delta = fwidth(r);
    alpha = 1.0 - smoothstep(1.0 - delta, 1.0 + delta, r);
    
    out_color = frag_color * alpha * diffuse;
    
}
