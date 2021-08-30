static const char* mk_cpp_rsrc_hello_glsl___o_()
{
   return R"V0G0NP03TrY(#version 430

layout(local_size_x = 1, local_size_y = 1) in;
layout(rgba32f) uniform image2D img_output;

// ----------------------------------------------------------------------- index

void main()
{    
    // http://antongerdelan.net/opengl/compute.html
    
    // base pixel colour for image
    vec4 pixel = vec4(0.1, 1.0, 0.0, 1.0);
    // get index in global work group i.e x,y position
    ivec2 pixel_coords = ivec2(gl_GlobalInvocationID.xy);
  
    //
    // interesting stuff happens here later
    //
  
    // output to a specific pixel in the image
    imageStore(img_output, pixel_coords, pixel);
}

)V0G0NP03TrY";
}

static const char* mk_cpp_rsrc_voxel_consistency_glsl___o_()
{
   return R"V0G0NP03TrY(
#version 430
// #pragma optimize(off)

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

// -------------------------------------------------------------------- Textures

layout(binding = 1, rgba8ui) uniform uimage3D in_image;
//layout(binding = 2, rgba32f) readonly  uniform image3D  LAB_image;
layout(binding = 3, rgba32f) uniform image3D  dbg_output;

// ------------------------------------------------------------------- Constants
// This syntax is an extension handled by proprocessing.
// Helps get around GLSL limitations regarding arrays
const int MAX_CAMS = ${MAX_CAMS};
const int SAD_SIZE = ${SAD_SIZE};
const int WINDOW_SIZE = SAD_SIZE * SAD_SIZE;
const bool USE_LAB = false;

//uniform int    HALF_SAD;
const int HALF_SAD = (SAD_SIZE - 1) / 2;

// ------------------------------------------------------------------- Variables

uniform int    n_cams;
uniform int    width;
uniform int    height;
uniform mat3x3 K;
uniform mat3x3 K_inv;
uniform vec3   Cs[MAX_CAMS];     // Cam centers
uniform mat3x3 Rs[MAX_CAMS];     // Rotations
uniform mat3x3 Rs_inv[MAX_CAMS]; // Inverse rotations
uniform mat3x3 KRs[MAX_CAMS];    // Rotations
uniform mat4x3 Ps[MAX_CAMS];     // Camera matrices, 4 cols, 3 rows

uniform vec3   max_xyz;
uniform vec3   min_xyz;

uniform float  ray_step;         //

vec3   buf[2][2];//WINDOW_SIZE];

// --------------------------------------------------------- Projective Geometry

vec3 homgen(vec3 x) { return x / ((abs(x.z) > 1e-20) ? x.z : 1e-20); }

vec3 pix_to_eye(vec3 pix) { return normalize(K_inv * pix); }
vec3 pix_to_ray(int cam, vec3 pix) { return normalize(Rs_inv[cam]*K_inv*pix); }
vec3 world_to_pix(int cam, vec3 X) { return homgen(KRs[cam] * (X - Cs[cam])); }
vec3 world_to_eye(int cam, vec3 X) { return normalize(Rs[cam] * (X - Cs[cam]));}

bool in_front(int cam, vec3 X) { return world_to_eye(cam, X).z > 0.0; }

// ----------------------------------------------------------------- to LAB/Grey

vec3 to_LAB(vec3 k)
{
    // sRGB to XYZ conversion
    //------------------------
    const float R = k.x;
    const float G = k.y;
    const float B = k.z;
    
    const float r = (R <= 0.04045f) ? R/12.92f : pow((R+0.055f)/1.055f, 2.4f);
    const float g = (G <= 0.04045f) ? G/12.92f : pow((G+0.055f)/1.055f, 2.4f);
    const float b = (B <= 0.04045f) ? B/12.92f : pow((B+0.055f)/1.055f, 2.4f);  
    
    const vec3 XYZ = vec3(r*0.4124564f + g*0.3575761f + b*0.1804375f,
                          r*0.2126729f + g*0.7151522f + b*0.0721750f,
                          r*0.0193339f + g*0.1191920f + b*0.9503041f);
    
    // XYZ to LAB conversion
    //------------------------
    const float epsilon = 0.008856f;// actual CIE standard
    const float kappa   = 7.787f;   // actual CIE standard

    const float xr = XYZ.x / 0.950456f;  // reference white
    const float yr = XYZ.y / 1.0f;       // reference white
    const float zr = XYZ.z / 1.088754f;  // reference white

    const float c = 1.0f / 3.0f;

    const float fx = (xr > epsilon) ? pow(xr, c) : (kappa*xr + 16.0f)/116.0f;
    const float fy = (yr > epsilon) ? pow(yr, c) : (kappa*yr + 16.0f)/116.0f;
    const float fz = (zr > epsilon) ? pow(zr, c) : (kappa*zr + 16.0f)/116.0f;

    return vec3(116.0f*fy-16.0f, 500.0f*(fx-fy), 200.0f*(fy-fz));
}

float to_grey(vec3 k) { return 0.21f * k.x + 0.72f * k.y + 0.07f * k.z; }

// ------------------------------------------------------------------- CAD Model

bool is_in_CAD(vec3 X)
{
    if(X.x < min_xyz.x) return false;
    if(X.y < min_xyz.y) return false;
    if(X.z < min_xyz.z) return false;
    if(X.x > max_xyz.x) return false;
    if(X.y > max_xyz.y) return false;
    if(X.z > max_xyz.z) return false;
    return true;
}

// -------------------------------------------------------------- Loading Pixels
// @return [r, g, b] float. The image is uint8 [a, r, g, b]
vec3 load_pix(ivec3 coords) // x, y, layer
{
    const vec4 argb4 = imageLoad(in_image, coords);
    const vec3 rgb = vec3(argb4.y, argb4.z, argb4.w) / 255.0;
    return rgb;
}

vec3 load_LAB(ivec3 coords)
{
    return to_LAB(load_pix(coords));
}

float load_grey(ivec3 coords)
{
    return to_grey(load_pix(coords));
}

// Loads pixels into window
// @pos:      x, y, layer
// @buf_ind:  Must be 0 or 1
void load_window(ivec3 pos, int buf_ind) 
{
    int idx = 0;
    for(int dy = -HALF_SAD; dy <= HALF_SAD; ++dy)
        for(int dx = -HALF_SAD; dx <= HALF_SAD; ++dx)
            buf[buf_ind][idx++] = load_pix(ivec3(pos.x+dx, pos.y+dy, pos.z));
}

// @return TRUE iff pos has a valid SAD window
bool has_valid_window(ivec3 pos)
{
    return (pos.x >= HALF_SAD && pos.x < width  - HALF_SAD)
        && (pos.y >= HALF_SAD && pos.y < height - HALF_SAD);
}

// ------------------------------------------- Sum of Absolute Differences (SAD)
// Calculates sum-of-absolute-differences between currently loaded bufs
float SAD_with_grey()
{
    float sum = 0.0;
    for(int ind = 0; ind < WINDOW_SIZE; ++ind)
        sum += abs(to_grey(buf[0][ind]) - to_grey(buf[1][ind]));
    return sum;
}

float SAD_with_LAB()
{
    float sum = 0.0;
    for(int ind = 0; ind < WINDOW_SIZE; ++ind)
        sum += length(to_LAB(buf[0][ind]) - to_LAB(buf[1][ind]));
    return sum;
}

float SAD()
{
    return 0.0; // return USE_LAB ? SAD_with_LAB() : SAD_with_grey();
}

float SAD_with_grey(ivec3 p0, ivec3 p1)
{
    float sum = 0.0;
    for(int dy0 = -HALF_SAD; dy0 <= HALF_SAD; ++dy0)
        for(int dx0 = -HALF_SAD; dx0 <= HALF_SAD; ++dx0)
            for(int dy1 = -HALF_SAD; dy1 <= HALF_SAD; ++dy1)
                for(int dx1 = -HALF_SAD; dx1 <= HALF_SAD; ++dx1)
                    sum += abs(load_grey(ivec3(p0.x+dx0, p0.y+dy0, p0.z)) -
                               load_grey(ivec3(p1.x+dx1, p1.y+dy1, p1.z)));
    return sum;
}

// ------------------------------------------------------------------------ Main

void main()
{    
    // ===-=== Inputs ===-===
    //   + CAD bounds (xyz-min-max)
    //   + "Voxel" size: e.g., 2cm
    //   + Use SAD (or NCC)
    //   + Use SLIC-label (or fronto-parallel square)
    //   + SAD-window-size
    //   + Intrinsic: A        (an array (?))
    //   + Extrinsic: C, R     (an array)
    //   + Distortion maps     (one 3D image, readonly)
    //   + Vector3f LAB images (one 3D image, readonly)
    //   + SLIC labels         (one 3D image, readonly)

    // @ Invocation (x, y) => ray + set of distances without 3D bounds

    // @ Back project the voxel into each "other" image
    //   + Calculate the NCC (or SAD), optionally restrict to common slic-label
 
    // @ Keep the top four distances

    // ===-=== Outputs ===-===
    // Top-four distances for each image (one 3D image, readonly)
    // Debug texture

    // ===-=== On the CPU ===-===
    // Each super-pixel should be locally smooth; outlier removal

    const ivec3 pos = ivec3(gl_GlobalInvocationID.xyz);
    const int cam = pos.z;
    const vec3 C = Cs[cam];
    const vec3 pix = vec3(pos.x, pos.y, 1.0);
    const vec3 ray = pix_to_ray(cam, pix);

    vec4 dout = vec4(0.0, 0.0, 0.0, 0.0);
    dout.x = has_valid_window(pos) ? 1.0 : -1.0;
    // Load the current pixel into buffer 0

    
    if(has_valid_window(pos)) {

        // load_window(pos, 0);
        // load_window(pos, 1);
        
        float lambda = ray_step;
        vec3 X = C + lambda * ray;
        
        while(is_in_CAD(X)) {

            // //     // ivec3 o_pos = ivec3(50, 60, 2);
            for(int o_cam = 0; o_cam < MAX_CAMS; ++o_cam) {
                if(cam == o_cam) continue;
                vec3 o_pix = world_to_pix(o_cam, X);
                ivec3 o_pos = ivec3(round(o_pix.x), round(o_pix.y), o_cam);
                if(has_valid_window(o_pos)) {
                    //load_window(o_pos, 1);
                    dout.w += SAD_with_grey(pos, o_pos);
                }
            }

            lambda += ray_step;
            X = C + lambda * ray;
            dout.y += 1.0;
        }
    }
    
    imageStore(dbg_output, pos, dout);

    // const ivec3 in_coords = ivec3(gl_GlobalInvocationID.xyz);
    // imageStore(dbg_output, in_coords, dout);
    
    // vec4 argb4 = imageLoad(in_image, in_coords);
    // vec3 rgb = vec3(argb4.y, argb4.z, argb4.w) / 255.0;
    
    // vec3 x_pixel = vec3(in_coords.x, in_coords.y, 1.0);
    // vec3 eye = normalize(K_inv * x_pixel);
    // vec3 ray = Rs_inv[cam] * eye;
    // vec3 C = Cs[cam];

    // vec3 O;
    // float lambda = 0.0f;
    // while(true) {
    //     lambda += ray_step;
    //     vec3 X = C + lambda * ray;
    //     if(!is_in_CAD(X)) break;
    //     O = X;
    // }
    
    
    // // base pixel colour for image
    // vec4 dout = vec4(0.0, 0.4, 0.2, 0.0);
    // // get index in global work group i.e x,y position

    // // Let's project [3.092, 3.487, 0.740] into each camera
    // vec3 X = vec3(3.092, 3.487, 0.740);
    // X = vec3(-1.0, 0.0, 0.0);
    // vec3 x = KRs[cam] * (X - C);
    // x /= x.z;

    // // Lets go from 'x' to a ray
    // eye = normalize(K_inv * x);
    // ray = Rs_inv[cam] * eye;

    // float grey = to_grey(rgb);
    // vec3 lab = to_LAB(rgb);
    
    // dout.xyz = lab;
    // if(in_front(cam, X))
    //     dout.w = 1.0;
    // else
    //     dout.w = 0.0;
    // //in_front(cam, X) ? 1.0 : 0.0;
    // dout.w = pow(2.0, 1.0/3.0);

    // dout.xyz = world_to_pix(cam, O);
    // dout.w = in_front(cam, O) ? 1.0 : 0.0;

    // dout.x = Ps[cam][0][0];
    // dout.y = Ps[cam][1][0];
    // dout.z = Ps[cam][2][0];
    // dout.w = Ps[cam][3][0];

    // dout.x = K[0][0];
    // dout.y = K[0][1];
    // dout.z = K[0][2];
    // dout.w = K[1][0];


    // vec3 x_pixel = vec3(pixel_coords.x, pixel_coords.y, 1.0);
    // vec3 eye = K_inv * x_pixel;
    // eye /= eye.z;
    
    //
    // interesting stuff happens here later
    //
  
    // output to a specific pixel in the image
    // imageStore(dbg_output, in_coords, dout);
}

)V0G0NP03TrY";
}

const char* glsl_cstr(const char* fname)
{
   if(strcmp(fname, "hello.glsl") == 0) return mk_cpp_rsrc_hello_glsl___o_();
   if(strcmp(fname, "voxel-consistency.glsl") == 0)
      return mk_cpp_rsrc_voxel_consistency_glsl___o_();
   return nullptr;
}
