#version 120 core
out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
	vec3 tPosition;
} fs_in;

uniform sampler2D floorTexture;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform bool blinn;

void main()
{           
    vec3 color = texture(floorTexture, fs_in.TexCoords).rgb;
    // ambient
    vec3 ambient = 0.05 * color;
    // diffuse
    vec3 lightDir = normalize(lightPos - fs_in.FragPos+fs_in.tPosition);
    vec3 normal = normalize(fs_in.Normal+fs_in.tPosition);
    float diff = max(dot(lightDir, normal), 0.0);
    vec3 diffuse = diff * color;
    // specular
    vec3 viewDir = normalize(viewPos - fs_in.FragPos+fs_in.tPosition);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = 0.0;
	const float kPi = 3.14159265;
	const float kShininess = 16.0;

	if( blinn )
	{
	const float kEnergyConservation = ( 8.0 + kShininess ) / ( 8.0 * kPi ); 
	   vec3 halfwayDir = normalize(lightDir + viewDir); 
	   spec = kEnergyConservation * pow(max(dot(normal, halfwayDir), 0.0), kShininess);
	}
	else
	{
	   const float kEnergyConservation = ( 2.0 + kShininess ) / ( 2.0 * kPi ); 
	   vec3 reflectDir = reflect(-lightDir, normal);
	   spec = kEnergyConservation * pow(max(dot(viewDir, reflectDir), 0.0), kShininess);
	}
    vec3 specular = vec3(0.3) * spec; // assuming bright white light color
    FragColor = vec4(ambient + diffuse + specular, 1.0);
}
