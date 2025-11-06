fn hit_sphere(center: vec3f, radius: f32, r: ray, record: ptr<function, hit_record>, max: f32)
{

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
  // If no significant rotation, use simple AABB
  if (length(rotation) < 0.01) {
    var m = 1.0 / r.direction;
    var n = m * (r.origin - center);
    var k = abs(m) * rad;

    var t1 = -n - k;
    var t2 = -n + k;

    var tN = max(max(t1.x, t1.y), t1.z);
    var tF = min(min(t2.x, t2.y), t2.z);

    if (tN > tF || tF < 0.0)
    {
      record.hit_anything = false;
      return false;
    }

    var t = tN;
    if (t < RAY_TMIN || t > t_max)
    {
      record.hit_anything = false;
      return false;
    }

    record.t = t;
    record.p = ray_at(r, t);
    record.normal = -sign(r.direction) * step(t1.yzx, t1.xyz) * step(t1.zxy, t1.xyz);
    record.hit_anything = true;
    return true;
  }

  // For rotated boxes, transform ray to local space
  var quat = quaternion_from_euler(rotation);
  var quat_inv = q_inverse(quat);
  
  // Transform ray to box's local space
  var local_origin = rotate_vector(r.origin - center, quat_inv);
  var local_direction = normalize(rotate_vector(r.direction, quat_inv));
  
  // Perform AABB intersection in local space with safety checks
  var safe_direction = local_direction;
  if (abs(safe_direction.x) < 0.0001) { safe_direction.x = 0.0001; }
  if (abs(safe_direction.y) < 0.0001) { safe_direction.y = 0.0001; }
  if (abs(safe_direction.z) < 0.0001) { safe_direction.z = 0.0001; }
  
  var m = 1.0 / safe_direction;
  var n = m * local_origin;
  var k = abs(m) * rad;
  
  var t1 = -n - k;
  var t2 = -n + k;
  
  var tN = max(max(t1.x, t1.y), t1.z);
  var tF = min(min(t2.x, t2.y), t2.z);
  
  if (tN > tF || tF < 0.0)
  {
    record.hit_anything = false;
    return false;
  }
  
  var t = tN;
  if (t < RAY_TMIN || t > t_max)
  {
    record.hit_anything = false;
    return false;
  }
  
  // Calculate intersection point and normal in local space
  var local_p = local_origin + t * local_direction;
  var local_normal = -sign(safe_direction) * step(t1.yzx, t1.xyz) * step(t1.zxy, t1.xyz);
  
  // Transform back to world space
  record.t = t;
  record.p = rotate_vector(local_p, quat) + center;
  record.normal = normalize(rotate_vector(local_normal, quat));
  record.hit_anything = true;
  
  return true;
}