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

fn hit_cylinder(r: ray, center: vec3f, radius: f32, height: f32, rotation: vec3f, record: ptr<function, hit_record>, t_max: f32) -> bool
{
    // Transform ray to cylinder's local space
    var quat = quaternion_from_euler(rotation);
    var quat_inv = q_inverse(quat);
    
    var local_origin = rotate_vector(r.origin - center, quat_inv);
    var local_direction = normalize(rotate_vector(r.direction, quat_inv));
    
    // Cylinder is aligned with Y axis in local space
    // Test intersection with infinite cylinder first
    var a = local_direction.x * local_direction.x + local_direction.z * local_direction.z;
    var b = 2.0 * (local_origin.x * local_direction.x + local_origin.z * local_direction.z);
    var c = local_origin.x * local_origin.x + local_origin.z * local_origin.z - radius * radius;
    
    var discriminant = b * b - 4.0 * a * c;
    
    if (discriminant < 0.0 || a < 0.0001)
    {
        return false;
    }
    
    var sqrt_disc = sqrt(discriminant);
    var t = (-b - sqrt_disc) / (2.0 * a);
    
    // Check if first intersection is valid
    if (t < RAY_TMIN || t > t_max)
    {
        t = (-b + sqrt_disc) / (2.0 * a);
        if (t < RAY_TMIN || t > t_max)
        {
            return false;
        }
    }
    
    // Calculate hit point in local space
    var local_p = local_origin + t * local_direction;
    
    // Check if hit is within cylinder height
    var half_height = height * 0.5;
    
    // Check intersection with caps first
    var hit_cap = false;
    var cap_t = t_max;
    var cap_normal = vec3f(0.0);
    
    // Bottom cap (y = -half_height)
    if (abs(local_direction.y) > 0.0001)
    {
        var t_bottom = (-half_height - local_origin.y) / local_direction.y;
        if (t_bottom > RAY_TMIN && t_bottom < t_max)
        {
            var p_bottom = local_origin + t_bottom * local_direction;
            var dist_from_axis = sqrt(p_bottom.x * p_bottom.x + p_bottom.z * p_bottom.z);
            
            if (dist_from_axis <= radius)
            {
                hit_cap = true;
                cap_t = t_bottom;
                cap_normal = vec3f(0.0, -1.0, 0.0);
            }
        }
        
        // Top cap (y = half_height)
        var t_top = (half_height - local_origin.y) / local_direction.y;
        if (t_top > RAY_TMIN && t_top < t_max)
        {
            var p_top = local_origin + t_top * local_direction;
            var dist_from_axis = sqrt(p_top.x * p_top.x + p_top.z * p_top.z);
            
            if (dist_from_axis <= radius && t_top < cap_t)
            {
                hit_cap = true;
                cap_t = t_top;
                cap_normal = vec3f(0.0, 1.0, 0.0);
            }
        }
    }
    
    // Check side intersection
    var hit_side = false;
    var side_t = t_max;
    var side_normal = vec3f(0.0);
    
    if (local_p.y >= -half_height && local_p.y <= half_height)
    {
        hit_side = true;
        side_t = t;
        side_normal = normalize(vec3f(local_p.x, 0.0, local_p.z));
    }
    else
    {
        // Try the other intersection
        t = (-b + sqrt_disc) / (2.0 * a);
        if (t > RAY_TMIN && t < t_max)
        {
            local_p = local_origin + t * local_direction;
            if (local_p.y >= -half_height && local_p.y <= half_height)
            {
                hit_side = true;
                side_t = t;
                side_normal = normalize(vec3f(local_p.x, 0.0, local_p.z));
            }
        }
    }
    
    // Choose closest hit
    var final_t = t_max;
    var local_normal = vec3f(0.0);
    
    if (hit_side && side_t < final_t)
    {
        final_t = side_t;
        local_normal = side_normal;
        local_p = local_origin + final_t * local_direction;
    }
    
    if (hit_cap && cap_t < final_t)
    {
        final_t = cap_t;
        local_normal = cap_normal;
        local_p = local_origin + final_t * local_direction;
    }
    
    if (final_t >= t_max)
    {
        return false;
    }
    
    // Transform back to world space
    var world_normal = normalize(rotate_vector(local_normal, quat));
    var world_p = rotate_vector(local_p, quat) + center;
    
    record.t = final_t;
    record.p = world_p;
    record.hit_anything = true;
    
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