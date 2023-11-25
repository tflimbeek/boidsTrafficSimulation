layout(location=0) in vec3 aPos;

void main(){

    posState_s posState = bPosState[gl_InstanceID];
    vec4 position = posState.pos;
    float rotation = posState.rot;

    movState_s movState = bMovState[gl_InstanceID];
    float angle = movState.angle;

    simState_s simState = bSimState[gl_InstanceID];
    uint col = simState.collided;

    // Translation matrix
    mat4 tMat1 = mat4(
        1.0,    0.0,    0.0,    0.0,
        0.0,    1.0,    0.0,    0.0,
        0.0,    0.0,    1.0,    0.0,
        position.x,    position.y,    position.z,    1.0
    );
    
    // Rotation matrix
    mat4 rMat1 = mat4(
        cos(angle),    -sin(angle),    0.0,    0.0,
        sin(angle),    cos(angle),    0.0,    0.0,
        0.0,    0.0,    1.0,    0.0,
        0.0,    0.0,    0.0,    1.0
    );
    mat4 rMat2 = mat4(
        cos(rotation),    -sin(rotation),    0.0,    0.0,
        sin(rotation),    cos(rotation),    0.0,    0.0,
        0.0,    0.0,    1.0,    0.0,
        0.0,    0.0,    0.0,    1.0
    );

    // Scale matrix
    mat4 sMat = mat4(
        scale,    0.0,    0.0,    0.0,
        0.0,    scale,    0.0,    0.0,
        0.0,    0.0,    scale,    0.0,
        0.0,    0.0,    0.0,    1.0
    );

    gl_Position = uViewProjection * tMat1 * sMat * rMat2 * vec4(aPos.xyz, 1.0)*(1-simState.collided);
}