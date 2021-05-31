#pragma once
/* AG2244: NEW INCLUDES */
#include <freeglut_std.h>
#include <freeglut_ext.h>
#include <quaternion_geometric.hpp>
/* AG2244: END NEW INCLUDES */
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
/* Global variables */

char title[] = "3D Shapes";
float pi = 3.14159265359;
// settings
const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 600;


struct vec3 {
    float x;
    float y;
    float z;
    vec3() : x(0), y(0), z(0) {}
    vec3(float sx,float sy,float sz) : x(sx), y(sy), z(sz) {}
    float dot(vec3 v) { return x * v.x + y * v.y + z * v.z; }
    vec3 cross(vec3 v) { return vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    // scalar product
    friend vec3 operator*(float s, vec3 v) { return vec3(s * v.x, s * v.y, s * v.z); }
    friend vec3 operator+(vec3 a, vec3 v) { return vec3(a.x+v.x, a.y+ v.y, a.z + v.z); }
    vec3 normalize() {
        float invLength = 1.0f / sqrtf(x * x + y * y + z * z);
        x *= invLength;
        y *= invLength;
        z *= invLength;
        return *this;
    }
};

enum PathFindingMode {
    simple,
    objectTracing
};

enum Direction {
    left,
    right,
    up,
    down
};

enum PlatformType {
    box,
    sphere,
    prism
};

vec3 cross_vec3(vec3 a, vec3 b);
float angleBetweenTwoVec3(vec3 a, vec3 b);
float dot_vec3(vec3 a, vec3 b);
vec3 norm_vec3(vec3 vec);
float length_vec3(vec3 a);
vec3 rotate_around_axis_vec3(vec3 v, vec3 r, float a);
vec3 qRotate(glm::vec3 point, float angle, glm::vec3 axis);

bool pointObsticle(vec3 point, int obsticleId);
bool particleObsticle(vec3 pos, int obsticleId,char t);
vec3 getClosestPointToSphere(vec3 spherePos, float sphereRad, vec3 point);

template <typename T>
bool LineAABBIntersection(T aabbBox, vec3 v0, vec3 v1);
template <typename T>
bool LineSphereIntersection(T sphere, vec3 v0, vec3 v1);

template <typename T>
vec3 closestPointOnAABB(vec3 p, T b);

template <typename T>
vec3 closestPointOnSphere(vec3 p, T b);

template <typename T>
T getPlatform(int id);

template <typename T>
T getParticle(int id);

void drawLine(float x1, float y1, float z1, float x2, float y2, float z2);