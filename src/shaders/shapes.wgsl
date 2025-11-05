fn hit_sphere(center: vec3f, radius: f32, r: ray, rec: ptr<function, hit_record>, max_t: f32) {
    let oc = r.origin - center;
    let a = dot(r.direction, r.direction);
    let half_b = dot(oc, r.direction);
    let c = dot(oc, oc) - radius * radius;
    let discriminant = half_b * half_b - a * c;
    if (discriminant <= 0.0) {
        return;
    }

  var oc = r.origin - center;
  var a = dot(r.direction, r.direction);
  var half_b = dot(oc, r.direction);
  var c = dot(oc, oc) - radius * radius;

  var discriminant = half_b * half_b - a * c;
  if (discriminant < 0.0)
  {
    record.hit_anything = false;
    return;
  }

  var sqrtd = sqrt(discriminant);
  var root = (-half_b - sqrtd) / a;
  if (root < RAY_TMIN || root > max)
  {
    root = (-half_b + sqrtd) / a;
    if (root < RAY_TMIN || root > max)
    {
      record.hit_anything = false;
      return;
    }
  }

  record.t = root;
  record.p = ray_at(r, record.t);

  var outward_normal = (record.p - center) / radius;
  var front = dot(r.direction, outward_normal) < 0.0;
  record.frontface = front;
  if (front)
  {
    record.normal = outward_normal;
  }
  else
  {
    record.normal = -outward_normal;
  }

  record.hit_anything = true;
}

fn hit_quad(r: ray, Q: vec4f, u: vec4f, v: vec4f, record: ptr<function, hit_record>, max: f32) -> bool
{
    // Calculate the normal of the quad
    var n = cross(u.xyz, v.xyz);
    var normal = normalize(n);
    
    // Calculate the plane constant D
    var D = dot(normal, Q.xyz);
    
    // Calculate denominator
    var denom = dot(normal, r.direction);
    
    // Check if ray is parallel to the quad
    if (abs(denom) < RAY_TMIN)
    {
        return false;
    }
    
    // Calculate t (distance along ray)
    var t = (D - dot(normal, r.origin)) / denom;
    
    // Check if intersection is within valid range
    if (t < RAY_TMIN || t > max)
    {
        return false;
    }
    
    // Calculate intersection point
    var p = ray_at(r, t);
    
    // Calculate planar hitpoint coordinates
    var planar_hitpt = p - Q.xyz;
    
    // Calculate w vector for bounds checking
    var w = n / dot(n, n);
    var alpha = dot(w, cross(planar_hitpt, v.xyz));
    var beta = dot(w, cross(u.xyz, planar_hitpt));
    
    // Check if point is inside the quad bounds
    if (alpha < 0.0 || alpha > 1.0 || beta < 0.0 || beta > 1.0)
    {
        return false;
    }
    
    // Set hit record
    record.t = t;
    record.p = p;
    record.hit_anything = true;
    
    // Determine front face
    var front_face = dot(r.direction, normal) < 0.0;
    record.frontface = front_face;
    
    if (front_face)
    {
        record.normal = normal;
    }
    else
    {
        record.normal = -normal;
    }
    
    return true;
}

fn hit_triangle(r: ray, v0: vec3f, v1: vec3f, v2: vec3f, record: ptr<function, hit_record>, max: f32)
{
  var v1v0 = v1 - v0;
  var v2v0 = v2 - v0;
  var rov0 = r.origin - v0;

  var n = cross(v1v0, v2v0);
  var q = cross(rov0, r.direction);

  var d = 1.0 / dot(r.direction, n);

  var u = d * dot(-q, v2v0);
  var v = d * dot(q, v1v0);
  var t = d * dot(-n, rov0);

  if (u < 0.0 || u > 1.0 || v < 0.0 || (u + v) > 1.0)
  {
    record.hit_anything = false;
    return;
  }

  if (t < RAY_TMIN || t > max)
  {
    record.hit_anything = false;
    return;
  }

  record.t = t;
  record.p = ray_at(r, t);
  record.normal = normalize(n);
  record.hit_anything = true;
}

fn hit_box(r: ray, center: vec3f, rad: vec3f, rotation: vec3f, record: ptr<function, hit_record>, t_max: f32) -> bool
{
    // Convert Euler angles to quaternion
    var quat = quaternion_from_euler(rotation);
    var quat_inv = q_inverse(quat);
    
    // Transform ray to box's local space
    var local_origin = rotate_vector(r.origin - center, quat_inv);
    var local_direction = rotate_vector(r.direction, quat_inv);
    
    // Create local ray
    var local_ray = ray(local_origin, local_direction);
    
    // Calculate intersection using AABB method
    var m = 1.0 / local_ray.direction;
    var n = m * local_ray.origin;
    var k = abs(m) * rad;
    
    var t1 = -n - k;
    var t2 = -n + k;
    
    var tN = max(max(t1.x, t1.y), t1.z);
    var tF = min(min(t2.x, t2.y), t2.z);
    
    // Check if there's an intersection
    if (tN > tF || tF < RAY_TMIN)
    {
        return false;
    }
    
    // Choose the closest intersection
    var t = tN;
    if (tN < RAY_TMIN)
    {
        t = tF;
    }
    
    // Check if intersection is within valid range
    if (t < RAY_TMIN || t > t_max)
    {
        return false;
    }
    
    // Calculate intersection point in local space
    var local_p = ray_at(local_ray, t);
    
    // Calculate normal in local space
    var local_normal = vec3f(0.0);
    var eps = 0.0001;
    
    if (abs(local_p.x - rad.x) < eps)
    {
        local_normal = vec3f(1.0, 0.0, 0.0);
    }
    else if (abs(local_p.x + rad.x) < eps)
    {
        local_normal = vec3f(-1.0, 0.0, 0.0);
    }
    else if (abs(local_p.y - rad.y) < eps)
    {
        local_normal = vec3f(0.0, 1.0, 0.0);
    }
    else if (abs(local_p.y + rad.y) < eps)
    {
        local_normal = vec3f(0.0, -1.0, 0.0);
    }
    else if (abs(local_p.z - rad.z) < eps)
    {
        local_normal = vec3f(0.0, 0.0, 1.0);
    }
    else
    {
        local_normal = vec3f(0.0, 0.0, -1.0);
    }
    
    // Transform normal back to world space
    var world_normal = normalize(rotate_vector(local_normal, quat));
    
    // Set hit record
    record.t = t;
    record.p = ray_at(r, t);
    record.hit_anything = true;
    
    // Determine front face
    var front_face = dot(r.direction, world_normal) < 0.0;
    record.frontface = front_face;
    
    if (front_face)
    {
        record.normal = world_normal;
    }
    else
    {
        record.normal = -world_normal;
    }
    
    return true;
}