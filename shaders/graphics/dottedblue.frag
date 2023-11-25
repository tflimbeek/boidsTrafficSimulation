layout(location=0) out vec4 fColor;

in vec2 fragTexCoord;
out vec4 fragColor;

uniform float dotSize = 2.0; // Size of dots
uniform float gapSize = 0.8; // Size of gaps
uniform vec4 dotColor = vec4(0.0, 0.0, 1.0, 1.0);

void main(){
    //fColor = vec4(0.0, 0.0, 1.0, 1.0);
    float centerDotDistance = length(fragTexCoord - vec2(0.5));

    if(mod(centerDotDistance/(dotSize+gapSize), 1.0) < 0.5){
        fColor = dotColor;
    } else {
        discard;
    }
}