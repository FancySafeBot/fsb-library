
#include "fsb_geometry.h"

namespace fsb
{

static void
set_collision_object_ray(const Vec3& position_start, const Vec3& direction, CollisionObject& object)
{
}
static void
set_collision_object_plane(const Vec3& position, const Vec3& normal, CollisionObject& object)
{
}
static void
set_collision_object_sphere(const Vec3& position, real_t radius, CollisionObject& object)
{
}
static void set_collision_object_capsule(
    const Vec3& position_start, const Vec3& position_end, real_t radius, CollisionObject& object)
{
}

static CollisionObject transform_object(const Transform& pose, const CollisionObject& object)
{
    CollisionObject result = object;

    return result;
}

static CollisionError compute_collision(
    const CollisionObject& object_a, const CollisionObject& object_b, CollisionResult& collision)
{
    CollisionError err = CollisionError::SUCCESS;

    if (object_a.type == CollisionObjectType::RAY && object_b.type == CollisionObjectType::RAY)
    {
        return collision_ray_ray(object_a, object_b);
    }
    if (object_a.type == CollisionObjectType::RAY && object_b.type == CollisionObjectType::PLANE)
    {
        return collision_ray_plane(object_a, object_b);
    }

    return err;
}

}
