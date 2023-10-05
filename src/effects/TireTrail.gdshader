// NOTE: Shader automatically converted from Godot Engine 4.1.1.stable.mono's StandardMaterial3D.
shader_type spatial;
render_mode world_vertex_coords,blend_mix,depth_draw_opaque,cull_back,diffuse_burley,specular_schlick_ggx,unshaded;

// Fragment uniforms
uniform vec4 albedo : source_color;
uniform sampler2D texture_albedo : source_color,filter_linear_mipmap,repeat_enable;
uniform float roughness : hint_range(0,1);
uniform float specular;
uniform float metallic;
uniform vec3 uv1_scale;
uniform vec3 uv1_offset;

// Instance uniforms for vertex manipulation
const int point_count = 1000;
const int vertex_count = 2000;

uniform vec3[vertex_count] vertices;
uniform vec4[point_count] colors;
uniform int[point_count] visible;
uniform vec3 origin;
uniform vec4 end_color;
uniform int start;
uniform int end;
uniform int size;
uniform int max_size;


void vertex() {
	UV=UV*uv1_scale.xy+uv1_offset.xy;
	
	int point_offset = VERTEX_ID / 2;
	int point_idx = (end + point_offset) % max_size;
	int vertex_idx = point_idx * 2 + (VERTEX_ID % 2);
	int start_idx = start * 2 + (VERTEX_ID % 2);
	
	if (visible[point_idx] > 0) {
		float t = clamp(float(point_offset) / (float(size) - 1.0), 0, 1);
		COLOR = mix(colors[point_idx], end_color, 1.0 - t);
		VERTEX = vertices[vertex_idx];
	} 
	else {
		VERTEX = vertices[start_idx];
	}
}


void fragment() {
	vec2 base_uv = UV;
	vec4 albedo_tex = texture(texture_albedo,base_uv);
	albedo_tex *= COLOR;
	ALBEDO = albedo.rgb * albedo_tex.rgb;
	METALLIC = metallic;
	ROUGHNESS = roughness;
	SPECULAR = specular;
	ALPHA *= albedo.a * albedo_tex.a;
}
