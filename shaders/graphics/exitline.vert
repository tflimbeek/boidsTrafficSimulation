layout(location=0) in vec2 aPos;

out vec2 fragTexCoord;

void main(){

    // Scale matrix
    mat4 sMat = mat4(
        scale,    0.0,    0.0,    0.0,
        0.0,    scale,    0.0,    0.0,
        0.0,    0.0,    scale,    0.0,
        0.0,    0.0,    0.0,    1.0
    );

    gl_Position = uViewProjection * sMat * vec4(aPos.xy, 0.0, 1.0);
    fragTexCoord = aPos.xy * 0.5 + 0.5;
}