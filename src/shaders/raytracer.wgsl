const THREAD_COUNT = 16;
const RAY_TMIN = 0.0001;
const RAY_TMAX = 100.0;
const PI = 3.1415927f;
const FRAC_1_PI = 0.31830987f;
const FRAC_2_PI = 1.5707964f;

@group(0) @binding(0)  
  var<storage, read_write> fb : array<vec4f>;

@group(0) @binding(1)
  var<storage, read_write> rtfb : array<vec4f>;

@group(1) @binding(0)
  var<storage, read_write> uniforms : array<f32>;

@group(2) @binding(0)
  var<storage, read_write> spheresb : array<sphere>;

@group(2) @binding(1)
  var<storage, read_write> quadsb : array<quad>;

@group(2) @binding(2)
  var<storage, read_write> boxesb : array<box>;

@group(2) @binding(3)
  var<storage, read_write> trianglesb : array<triangle>;

@group(2) @binding(4)
  var<storage, read_write> meshb : array<mesh>;

struct ray {
  origin : vec3f,
  direction : vec3f,
};

struct sphere {
  transform : vec4f,
  color : vec4f,
  material : vec4f,
};

struct quad {
  Q : vec4f,
  u : vec4f,
  v : vec4f,
  color : vec4f,
  material : vec4f,
};

struct box {
  center : vec4f,
  radius : vec4f,
  rotation: vec4f,
  color : vec4f,
  material : vec4f,
};

struct triangle {
  v0 : vec4f,
  v1 : vec4f,
  v2 : vec4f,
};

struct mesh {
  transform : vec4f,
  scale : vec4f,
  rotation : vec4f,
  color : vec4f,
  material : vec4f,
  min : vec4f,
  max : vec4f,
  show_bb : f32,
  start : f32,
  end : f32,
};

struct material_behaviour {
  scatter : bool,
  direction : vec3f,
};

struct camera {
  origin : vec3f,
  lower_left_corner : vec3f,
  horizontal : vec3f,
  vertical : vec3f,
  u : vec3f,
  v : vec3f,
  w : vec3f,
  lens_radius : f32,
};

struct hit_record {
  t : f32,
  p : vec3f,
  normal : vec3f,
  object_color : vec4f,
  object_material : vec4f,
  frontface : bool,
  hit_anything : bool,
};

fn ray_at(r: ray, t: f32) -> vec3f
{
  return r.origin + t * r.direction;
}

fn get_ray(cam: camera, uv: vec2f, rng_state: ptr<function, u32>) -> ray
{
  var rd = cam.lens_radius * rng_next_vec3_in_unit_disk(rng_state);
  var offset = cam.u * rd.x + cam.v * rd.y;
  return ray(cam.origin + offset, normalize(cam.lower_left_corner + uv.x * cam.horizontal + uv.y * cam.vertical - cam.origin - offset));
}

fn get_camera(lookfrom: vec3f, lookat: vec3f, vup: vec3f, vfov: f32, aspect_ratio: f32, aperture: f32, focus_dist: f32) -> camera
{
  var camera = camera();
  camera.lens_radius = aperture / 2.0;

  var theta = degrees_to_radians(vfov);
  var h = tan(theta / 2.0);
  var w = aspect_ratio * h;

  camera.origin = lookfrom;
  camera.w = normalize(lookfrom - lookat);
  camera.u = normalize(cross(vup, camera.w));
  camera.v = cross(camera.u, camera.w);

  camera.lower_left_corner = camera.origin - w * focus_dist * camera.u - h * focus_dist * camera.v - focus_dist * camera.w;
  camera.horizontal = 2.0 * w * focus_dist * camera.u;
  camera.vertical = 2.0 * h * focus_dist * camera.v;

  return camera;
}

fn envoriment_color(direction: vec3f, color1: vec3f, color2: vec3f) -> vec3f
{
  var unit_direction = normalize(direction);
  var t = 0.5 * (unit_direction.y + 1.0);
  var col = (1.0 - t) * color1 + t * color2;

  var sun_direction = normalize(vec3(uniforms[13], uniforms[14], uniforms[15]));
  var sun_color = int_to_rgb(i32(uniforms[17]));
  var sun_intensity = uniforms[16];
  var sun_size = uniforms[18];

  var sun = clamp(dot(sun_direction, unit_direction), 0.0, 1.0);
  col += sun_color * max(0, (pow(sun, sun_size) * sun_intensity));

  return col;
}

fn check_ray_collision(r: ray, max: f32) -> hit_record
{
  var spheresCount = i32(uniforms[19]);
  var quadsCount = i32(uniforms[20]);
  var boxesCount = i32(uniforms[21]);
  var trianglesCount = i32(uniforms[22]);
  var meshCount = i32(uniforms[27]);

  var record = hit_record(RAY_TMAX, vec3f(0.0), vec3f(0.0), vec4f(0.0), vec4f(0.0), false, false);
  var closest = record;

  return closest;
}

fn lambertian(normal : vec3f, absorption: f32, random_sphere: vec3f, rng_state: ptr<function, u32>) -> material_behaviour
{
  return material_behaviour(true, vec3f(0.0));
}

fn metal(normal : vec3f, direction: vec3f, fuzz: f32, random_sphere: vec3f) -> material_behaviour
{
  return material_behaviour(false, vec3f(0.0));
}

fn dielectric(normal : vec3f, r_direction: vec3f, refraction_index: f32, frontface: bool, random_sphere: vec3f, fuzz: f32, rng_state: ptr<function, u32>) -> material_behaviour
{  
  return material_behaviour(false, vec3f(0.0));
}

fn emmisive(color: vec3f, light: f32) -> material_behaviour
{
  return material_behaviour(false, vec3f(0.0));
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// trace() percorre todos os buffers
fn trace(r: ray) -> vec3f {
    var closest = RAY_TMAX;
    var record = hit_record(0.0, vec3f(0.0), vec3f(0.0), vec4f(0.0), vec4f(0.0), false, false);

    let sphereCount = i32(uniforms[19]);
    for (var i = 0; i < sphereCount; i = i + 1) {
        var temp = record;
        hit_sphere(spheresb[i].transform.xyz, spheresb[i].transform.w, r, &temp, closest);
        if (temp.hit_anything && temp.t < closest) {
            // obter cor/material da esfera e normalizar se estiver em 0..255
            var col = spheresb[i].color.xyz;
            if (col.x > 1.5 || col.y > 1.5 || col.z > 1.5) {
                col = col / 255.0;
            }
            temp.object_color = vec4f(col, spheresb[i].color.w);
            temp.object_material = spheresb[i].material;
            closest = temp.t;
            record = temp;
        }
    }

    // (Opcional) adicionar interseções com quads/boxes/triangles/meshes aqui...

    if (record.hit_anything) {
        // iluminação simples: ambiente + difuso a partir da "sun direction" (uniforms[13..15])
        let n = normalize(record.normal);
        let light_dir = normalize(vec3f(uniforms[13], uniforms[14], uniforms[15]));
        let diff = max(dot(n, light_dir), 0.0);
        let base = record.object_color.xyz;
        let ambient = 0.12;
        let shaded = base * (ambient + (1.0 - ambient) * diff);
        return clamp(shaded, vec3f(0.0), vec3f(1.0));
    }

    // fundo via função de ambiente (mantém gradiente + sol)
    return envoriment_color(r.direction, vec3f(0.7, 0.8, 1.0), vec3f(1.0, 1.0, 1.0));
}

// ------------------------------------------------------------
// Render
@compute @workgroup_size(THREAD_COUNT, THREAD_COUNT, 1)
fn render(@builtin(global_invocation_id) id: vec3u) {
    let rez = uniforms[1];
    let fragCoord = vec2f(f32(id.x), f32(id.y));

    // normalize pixel coords [0,1] and flip Y so image isn't invertida
    let nx = fragCoord.x / rez;
    let ny = 1.0 - (fragCoord.y / rez);

    var r: ray;
    r.origin = vec3f(0.0, 0.0, 1.0);
    r.direction = normalize(vec3f(nx - 0.5, ny - 0.5, -1.0)); // Y flipped

    var color = vec3f(0.0, 0.0, 0.0);
    let samples_per_pixel = max(1, i32(uniforms[4]));
    for (var s = 0; s < samples_per_pixel; s = s + 1) {
        color = color + trace(r);
    }
    color = color / f32(samples_per_pixel);

    // clamp then gamma-correct (simple gamma 2.0 using sqrt)
    color = clamp(color, vec3f(0.0), vec3f(1.0));
    let color_gamma = vec3f(sqrt(color.x), sqrt(color.y), sqrt(color.z));

    var color_out = vec4f(color_gamma, 1.0);
    let map_fb = id.y * u32(rez) + id.x;
    let should_accumulate = uniforms[3];

    if (should_accumulate > 0.0) {
        // blend prev and current (prev assumed stored in same gamma space)
        let prev_rgb = rtfb[map_fb].xyz;
        let blended = prev_rgb * 0.5 + color_out.xyz * 0.5;
        color_out = vec4f(blended, color_out.w);
    }

    rtfb[map_fb] = color_out;
    fb[map_fb] = color_out;
}