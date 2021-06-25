#pragma once
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include "glew.h"//glew extention for shaders
#include "freeglut.h"  // GLUT, include glu.h and gl.h
#include <string.h>
#include <math.h>
#include "main.h"
#include "Shader.h"
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <vector>
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
    vec3(float sx, float sy, float sz) : x(sx), y(sy), z(sz) {}
    float dot(vec3 v) { return x * v.x + y * v.y + z * v.z; }
    vec3 cross(vec3 v) { return vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    float length() { return sqrt((x * x) + (y * y) + (z * z)); }
    // scalar product
    friend vec3 operator*(float s, vec3 v) { return vec3(s * v.x, s * v.y, s * v.z); }
    friend vec3 operator+(vec3 a, vec3 v) { return vec3(a.x + v.x, a.y + v.y, a.z + v.z); }
    friend vec3 operator-(vec3 a, vec3 v) { return vec3(a.x - v.x, a.y - v.y, a.z - v.z); }
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

enum AntStates {
    Static,
    FollowLeader,
    FollowDestination,
    FollowClosestBridge,
    Wandering
};

vec3 cross_vec3(vec3 a, vec3 b);
float angleBetweenTwoVec3(vec3 a, vec3 b);
float dot_vec3(vec3 a, vec3 b);
vec3 sub_vec3(vec3 a, vec3 b);
vec3 scale_vec3(vec3 a, float b);
vec3 norm_vec3(vec3 vec);
float length_vec3(vec3 a);
vec3 rotate_around_axis_vec3(vec3 v, vec3 r, float a);
vec3 qRotate(glm::vec3 point, float angle, glm::vec3 axis);

bool pointObsticle(vec3 point, int obsticleId);
bool particleObsticle(vec3 pos, int obsticleId, char t);
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

template <typename T>
vec3 get_cube_normal_from_position(T cube, vec3 pos);
vec3 get_platform_normal_from_position(int id, vec3 pos);
vec3 get_ant_normal_from_position(int id, vec3 pos);

void drawLine(float x1, float y1, float z1, float x2, float y2, float z2);

/* 
======================
MOVED FROM mainv14.cpp
======================
*/




float distance(float x1, float y1, float z1, float x2, float y2, float z2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
}

float randomNumber(float Min, float Max)
{
	return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
}

//in simulation
float particle_radii = .02;
float time_delta = 1 / 64.;
float particle_distance = particle_radii * 2.5;
int num_of_particles = 100;
float gravity = -0.01;
bool reachedGoal = false;
const float mui_static = 1.f;    // 0.00023; //0.021;
const float mui_kinematic = 0.8f; // 0.00017; //0.02;
bool bridge_completed = false;
bool can_freeze = true;

//platform class
class Platform {
public:
	float x, y, z, l, w, h, r;
	int id;
	PlatformType tp;
	Platform() {

	};
	Platform(float sx, float sy, float sz, PlatformType t, float sw, float sh, float sl, int sid) {
		x = sx;
		y = sy;
		z = sz;
		tp = t;
		l = sl;
		w = sw;
		h = sh;
		id = sid;
	};
	Platform(float sx, float sy, float sz, PlatformType t, float sr, int sid) {
		x = sx;
		y = sy;
		z = sz;
		tp = t;
		r = sr;
		id = sid;
	};
};

Direction getRandomDir() {
	Direction dir = static_cast<Direction>(rand() % 4);
	return dir;
}
//particle
class Particle {
public:
	float x = 0;
	float y = 0;
	float z = 0;
	int id = -1;
	float vx = 0;
	float vy = 0;
	float vz = 0;
	float pforwardx = 0;
	float pforwardy = 0;
	float pforwardz = 0;
	float px = 0;
	float py = 0;
	float pz = 0;
	float r = particle_radii;
	float mass = 1;
	float inv_mass = 1 / mass;
	float speed = .05;
	int timer = 0;
	int maxTimer = 700;
	int pftTimer = 0;
	bool wandering = false;
	int wanderTimer = 700;
	vec3 randomPoint = { 0,0,0 };
	vec3 destination = { 0,0,0 };
	vec3 up = { 0,1,0 };
	vec3 right = { 0,0,1 };
	vec3 forward = { 1,0,0 };
	vec3 projforward = { 0,0,0 };
	PathFindingMode pfm = PathFindingMode::simple;
	bool foundDest = false;
	bool collision = false;
	int obsticleCollisionId = -1;
	char collWith;//p for plat, a for ant
	std::unordered_map<char, std::vector<int>> obsticleCollisionIds;
	int attachedId = -1;
	char attachedType;
	bool startObjectTrace = false;
	bool foundClearPoint = false;
	vec3 contactPoint;
	vec3 closestPoint; //used for following destination. Get as close as possible. if hit point become static
	Direction dir;
	bool isStatic = false;
	bool isSleep = false;
	vec3 timerPosition = { 0,0,0 };
	vec3 pftStartPosition = { 0,0,0 };
	bool follow = false;
	bool scout = false;
	int followId = -1;
	AntStates state;
	Particle() {
	};

	Particle(float s_x, float s_y, float s_z, int s_id) {
		x = s_x;
		y = s_y;
		z = s_z;
		id = s_id;
	};

	void update() {

		inv_mass = 1 / mass;
		//freeze timer
		if (timer == 0) {
			timerPosition = { x,y,z };
		}

		switch (state) {
		case(Static): {
			isStatic = true;
			mass = 2000;
			vx = 0;
			vy = 0;
			vz = 0;
			if (attachedType == 'p') {
				//vy = -gravity * time_delta;
			}
		}break;
		case(FollowLeader): {
			if (followId == -1) {
				if (wandering == true) {
					if (timer > wanderTimer) {
						wandering = false;
						timer = -1;
					}
				}
				else {
					if (timer > maxTimer) {
						if (isStatic == false) {
							if (distance(x, y, z, timerPosition.x, timerPosition.y, timerPosition.z) < r * 1.1) {
								//random number. if above 50% we wander else we static
								float randn = ((float)rand() / (RAND_MAX));
								if (randn > .6)
									state = Static;
								else {
									state = Wandering;
									randomPoint = { randomNumber(-2,2),randomNumber(-2,2),randomNumber(-2,2) };
									printf("wander time");
								}
							}
							timer = -1;
						}

					}
				}
			}
			else {
				updateFollow();
			}
		}break;
		case(FollowDestination): {
			if (follow == false) {
				closestPoint = getContactPoint();

				if (distance(x, y, z, destination.x, destination.y, destination.z) < r * 1.5 ||
					distance(x, y, z, closestPoint.x, closestPoint.y, closestPoint.z) < r * 1.5) {
					//isStatic = true;
					state = Static;
				}
				if (timer > maxTimer) {
					if (isStatic == false) {
						if (distance(x, y, z, timerPosition.x, timerPosition.y, timerPosition.z) < r * 1.1) {
							//isStatic = true;
							if (speed >= .3) {
								//isStatic = true;
								state = Static;
								speed = .05;
								printf("static Time");
							}
							else {
								speed += .05;
							}


						}

					}
					timer = -1;
				}
			}
			else {
				if (timer > maxTimer) {
					if (isStatic == false) {
						if (distance(x, y, z, timerPosition.x, timerPosition.y, timerPosition.z) < r * 1.1) {
							//isStatic = true;
							state = Static;
							printf("static Time");
						}

					}
					timer = -1;
				}
			}
		}break;
		case(FollowClosestBridge): {

		}break;
		case(Wandering): {
			wandering = true;
			//when im done wandering i go to either the closest bridge or the destination. Whatever is closer
			if (wandering == true) {
				if (timer > wanderTimer) {
					wandering = false;
					//random here
					state = FollowDestination;
					timer = -1;
				}
			}
		}break;
		}
		if (state != Static) {
			//check if im in object tracing mode
				//if i can make a straight uninterupted line to the goal switch back to simple movements
			if (pfm == PathFindingMode::objectTracing) {
				bool ObsticleInPath = checkLineOfSight();
				//printf("line of sight checked");
				if (ObsticleInPath == true) {
					calculatePredictedMovement();
					pftTimer++;
					//printf("path blocked");
				}
				else {
					pfm = PathFindingMode::simple;
					//reset stuff here
					startObjectTrace = false;
					//printf("back to simple");
					pftTimer = 0;
				}
				//printf("objectTracing mode");
			}
			else {
				calculateMovement();
				//printf("simple mode ");
			}
		}
		checkGoal();
		update_rotations();
		timer++;
	};

	void updateFollow() {
		Particle p = getParticle<Particle>(followId);
		destination = { p.x,p.y,p.z };
		if (p.isStatic && distance(x, y, z, destination.x, destination.y, destination.z) < r * 2.0) {
			destination = p.destination;
			//follow = false;
			followId = -1;
		}
		//timeout for following
		if (timer > 1000) {
			if (distance(x, y, z, timerPosition.x, timerPosition.y, timerPosition.z) < r * 1.1) {
				//stop following and go to destination
				destination = p.destination;
				//follow = false;
				followId = -1;
			}
			timer = -1;
		}
		//destination = p.destination;
	}

	void checkGoal() {
		if (distance(x, y, z, destination.x, destination.y, destination.z) <= r * 2) {
			reachedGoal = true;
		}
	}

	void checkAheadFor() {

	}

	void calculateMovement() {
		vx = 0;
		vy = 0;
		vz = 0;
		vec3 vel;
		if (scout == true)
			vel = sub_vec3(destination, { x,y,z });
		else {
			if (wandering == true)
				vel = randomPoint;
			else
				vel = sub_vec3(destination, { x,y,z });
		}
		vel = vel.normalize();
		vel = scale_vec3(vel, speed * .99 * time_delta);
		vx = vel.x;
		vy = vel.y;
		vz = vel.z;

	}

	vec3 getContactPoint() {
		if (collWith == 'p') {
			Platform plat = getPlatform<Platform>(obsticleCollisionId);
			if (plat.tp == box) {
				return closestPointOnAABB(destination, plat);
			}
			else if (plat.tp == sphere) {
				return closestPointOnSphere(destination, plat);
			}

		}
		else if (collWith == 'a') {
			Particle p = getParticle<Particle>(obsticleCollisionId);
			return getClosestPointToSphere(destination, r, { p.x,p.y,p.z });
		}
		return{ 0,0,0 };
	}

	void calculatePredictedMovement() {
		if (startObjectTrace == false) {
			float predx = 0;
			float predy = 0;
			float predz = 0;
			calculateMovement();
			//printf("should only see this once");
			startObjectTrace = true;
			//find the contact point
			//set contact point as dest
			predx = x + vx;
			predy = y + vy;
			predz = z + vz;
			if (obsticleCollisionId != -1) {
				contactPoint = getContactPoint();
				dir = getRandomDir();
			}
		}

		//update contact point
		if (obsticleCollisionId != -1) {
			if (collWith == 'p') {
				//printf("checking p collision");
				Platform plat = getPlatform<Platform>(obsticleCollisionId);
				if (plat.tp == box) {
					contactPoint = closestPointOnAABB(destination, plat);
				}
				else if (plat.tp == sphere) {
					contactPoint = closestPointOnSphere(destination, plat);
					//printf("going sphere");
				}
			}
			else if (collWith == 'a') {
				//printf("checking a collision");
				contactPoint = getClosestPointToSphere(destination, r, { x,y,z });
			}
		}

		//check if reached the free point
		if (distance(x, y, z, contactPoint.x, contactPoint.y, contactPoint.z) <= r * 2) {
			pfm = PathFindingMode::simple;
			//printf("founddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
			startObjectTrace = false;
			return;
		}

		//check if we havent moved from a position in a while to change the direction of our movement
		if (pftTimer > 200) {
			if (distance(x, y, z, pftStartPosition.x, pftStartPosition.y, pftStartPosition.z) < r * 1.3) {
				dir = getRandomDir();
			}
			pftTimer = 0;
			pftStartPosition = { x,y,z };
		}

		//rotate either left/right chose left
		float predx = 0;
		float predy = 0;
		float predz = 0;


		if (dir == Direction::left) {
			//printf("going left");
			//check left
			bool freeSpaceLeft = false;
			//rotate it right
			glm::vec3 dir = glm::vec3(vx, vy, vz);
			//dir=glm::normalize(dir);
			vec3 rotated = qRotate(dir, -.01, glm::vec3(up.x, up.y, up.z));
			//printf("right rotation %f,%f,%f", rotated.x, rotated.y, rotated.z);
			float tempvx = rotated.x;
			float tempvy = 0;//rotated.y;
			float tempvz = rotated.z;
			predx = x + tempvx;
			predy = y + tempvy;
			predz = z + tempvz;

			if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
				//printf("current pos %f,%f,%f velocity %f,%f,%f ,predicted pos %f,%f,%f free space found ", x, y, z, tempvx, tempvy, tempvz, predx, predy, predz);
				freeSpaceLeft = true;
			}
			if (freeSpaceLeft == true) {
				//change velocity
				vx = tempvx;
				vy = 0;//tempvy;
				vz = tempvz;
				//printf("pathfound ");
			}

			if (freeSpaceLeft == false) {
				//check forward
				predx = x + vx;
				predy = y + vy;
				predz = z + vz;
				bool canMoveForward = false;
				//printf("no free space");
				if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == true) {
					//printf("no pass");
					bool pathfound = false;
					if (pathfound == false) {
						//rotate it left						
						//printf("bool is %i", compr);
						if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
							//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
							pathfound = true;
							//printf("forward pathfound");
							canMoveForward = true;
						}
						else {
							//printf("vy :%f ", vy);
							Platform plat = getPlatform<Platform>(obsticleCollisionId);
							bool contact = false;
							if (plat.tp == box) {
								contact = LineAABBIntersection(plat, { predx,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							else if (plat.tp == sphere) {
								contact = LineSphereIntersection(plat, { predx ,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							if (contact == false) {
								//printf("Path clear");
								return;
							}
							//printf("rotating ");
							float angle = .01; //angle in radians
							vec3 rotated = qRotate(glm::vec3(vx, vy, vz), angle, glm::vec3(up.x, up.y, up.z));
							vx = rotated.x;
							vy = 0;//rotated.y;
							vz = rotated.z;
						}
					}
				}
			}
		}
		else if (dir == Direction::right) {
			//printf("going right");
			//check right
			bool freeSpaceRight = false;
			//rotate it right
			glm::vec3 dir = glm::vec3(vx, vy, vz);
			//dir=glm::normalize(dir);
			vec3 rotated = qRotate(dir, -.01, glm::vec3(up.x, up.y, up.z));
			//printf("right rotation %f,%f,%f", rotated.x, rotated.y, rotated.z);
			float tempvx = rotated.x;
			float tempvy = 0;//rotated.y;
			float tempvz = rotated.z;
			predx = x + tempvx;
			predy = y + tempvy;
			predz = z + tempvz;

			if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
				//printf("current pos %f,%f,%f velocity %f,%f,%f ,predicted pos %f,%f,%f free space found ", x, y, z, tempvx, tempvy, tempvz, predx, predy, predz);
				freeSpaceRight = true;
			}
			if (freeSpaceRight == true) {
				//change velocity
				vx = tempvx;
				vy = tempvy;
				vz = tempvz;
				//printf("pathfound ");
			}

			if (freeSpaceRight == false) {
				//check forward
				predx = x + vx;
				predy = y + vy;
				predz = z + vz;
				bool canMoveForward = false;
				//printf("no free space");
				if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == true) {
					//printf("no pass");
					bool pathfound = false;
					if (pathfound == false) {
						//rotate it left						
						//printf("bool is %i", compr);
						if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
							//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
							pathfound = true;
							//printf("forward pathfound");
							canMoveForward = true;
						}
						else {
							//printf("vy :%f ", vy);
							Platform plat = getPlatform<Platform>(obsticleCollisionId);
							bool contact = false;
							if (plat.tp == box) {
								contact = LineAABBIntersection(plat, { predx,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							else if (plat.tp == sphere) {
								contact = LineSphereIntersection(plat, { predx ,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							if (contact == false) {
								//printf("Path clear");
								return;
							}
							//printf("rotating ");
							float angle = -.01; //angle in radians
							vec3 rotated = qRotate(glm::vec3(vx, vy, vz), angle, glm::vec3(up.x, up.y, up.z));
							vx = rotated.x;
							vy = 0;//rotated.y;
							vz = rotated.z;
						}
					}
				}
			}
		}

		else if (dir == Direction::up) {
			//printf("going up ");
			//check right
			bool freeSpaceUp = false;
			//rotate it right
			glm::vec3 dir = glm::vec3(vx, vy, vz);
			//dir=glm::normalize(dir);
			vec3 rotated = qRotate(dir, -.01f, glm::vec3(right.x, right.y, right.z));
			//printf("right rotation %f,%f,%f", rotated.x, rotated.y, rotated.z);
			float tempvx = rotated.x;
			float tempvy = rotated.y;
			float tempvz = rotated.z;
			predx = x + tempvx;
			predy = y + tempvy;
			predz = z + tempvz;

			if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
				//printf("current pos %f,%f,%f velocity %f,%f,%f ,predicted pos %f,%f,%f free space found ", x, y, z, tempvx, tempvy, tempvz, predx, predy, predz);
				freeSpaceUp = true;
			}
			if (freeSpaceUp == true) {
				//change velocity
				vx = tempvx;
				vy = tempvy;
				vz = tempvz;

				//printf("pathfound ");
			}

			if (freeSpaceUp == false) {
				//check forward
				predx = x + vx;
				predy = y + vy;
				predz = z + vz;
				bool canMoveForward = false;
				//printf("no free space");
				if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == true) {
					//printf("no pass");
					bool pathfound = false;
					if (pathfound == false) {
						//rotate it left						
						//printf("bool is %i", compr);
						if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
							//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
							pathfound = true;
							//printf("forward pathfound");
							canMoveForward = true;
						}
						else {
							//printf("vy :%f ", vy);
							Platform plat = getPlatform<Platform>(obsticleCollisionId);
							bool contact = false;
							if (plat.tp == box) {
								contact = LineAABBIntersection(plat, { predx,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							else if (plat.tp == sphere) {
								contact = LineSphereIntersection(plat, { predx ,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							if (contact == false) {
								//printf("Path clear");
								return;
							}
							//printf("rotating ");
							float angle = .01; //angle in radians
							vec3 rotated = qRotate(glm::vec3(vx, vy, vz), angle, glm::vec3(right.x, right.y, right.z));
							vx = rotated.x;
							vy = rotated.y;
							vz = rotated.z;
						}
					}
				}
			}
		}
		else if (dir == Direction::down) {
			//printf("going up ");
			//check right
			bool freeSpaceUp = false;
			//rotate it right
			glm::vec3 dir = glm::vec3(vx, vy, vz);
			//dir=glm::normalize(dir);
			vec3 rotated = qRotate(dir, .01f, glm::vec3(right.x, right.y, right.z));
			//printf("right rotation %f,%f,%f", rotated.x, rotated.y, rotated.z);
			float tempvx = rotated.x;
			float tempvy = rotated.y;
			float tempvz = rotated.z;
			predx = x + tempvx;
			predy = y + tempvy;
			predz = z + tempvz;

			if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
				//printf("current pos %f,%f,%f velocity %f,%f,%f ,predicted pos %f,%f,%f free space found ", x, y, z, tempvx, tempvy, tempvz, predx, predy, predz);
				freeSpaceUp = true;
			}
			if (freeSpaceUp == true) {
				//change velocity
				vx = tempvx;
				vy = tempvy;
				vz = tempvz;

				//printf("pathfound ");
			}

			if (freeSpaceUp == false) {
				//check forward
				predx = x + vx;
				predy = y + vy;
				predz = z + vz;
				bool canMoveForward = false;
				//printf("no free space");
				if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == true) {
					//printf("no pass");
					bool pathfound = false;
					if (pathfound == false) {
						//rotate it left						
						//printf("bool is %i", compr);
						if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
							//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
							pathfound = true;
							//printf("forward pathfound");
							canMoveForward = true;
						}
						else {
							//printf("vy :%f ", vy);
							Platform plat = getPlatform<Platform>(obsticleCollisionId);
							bool contact = false;
							if (plat.tp == box) {
								contact = LineAABBIntersection(plat, { predx,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							else if (plat.tp == sphere) {
								contact = LineSphereIntersection(plat, { predx ,predy,predz }, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							if (contact == false) {
								//printf("Path clear");
								return;
							}
							//printf("rotating ");
							float angle = -.01; //angle in radians
							vec3 rotated = qRotate(glm::vec3(vx, vy, vz), angle, glm::vec3(right.x, right.y, right.z));
							vx = rotated.x;
							vy = rotated.y;
							vz = rotated.z;
						}
					}
				}
			}
		}
		//vx = vx * speed * .99 * time_delta;
		//vy = vy * speed * .99 * time_delta;
		//vz = vz * speed * .99 * time_delta;
	}

	void update_rotations() {
		vec3 dir = { vx,vy,vz };
		float angle = angleBetweenTwoVec3(forward, dir);
		vec3 axis = cross_vec3(forward, dir);
		axis = axis.normalize();
		if (angle == 0)
			return;

		if (dot_vec3(forward, dir) / (forward.length() * dir.length()) == -1) {
			axis = up;
		}

		forward = qRotate(glm::vec3(forward.x, forward.y, forward.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		right = qRotate(glm::vec3(right.x, right.y, right.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		up = qRotate(glm::vec3(up.x, up.y, up.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		//printf("quat is %f,%f,%f ", r.x, r.y, r.z);
		//if (collWith == 'p') {
		handleSurfaceNormal();
		//return everything again
//		}
		projforward = up;
		pforwardx = forward.x * speed * .99 * time_delta;
		pforwardy = forward.y * speed * .99 * time_delta;
		pforwardz = forward.z * speed * .99 * time_delta;

	}

	void handleSurfaceNormal() {
		//printf("here");
		if (collWith == 'p') {
			//printf("checking p collision");
			Platform plat = getPlatform<Platform>(obsticleCollisionId);
			vec3 newUp = get_platform_normal_from_position(obsticleCollisionId, { x,y,z });
			float angle = angleBetweenTwoVec3(up, newUp);
			vec3 axis = cross_vec3(up, newUp);
			axis = axis.normalize();
			if (angle == 0) {
				//printf("same");
				return;
			}
			//up = newUp;
			forward = qRotate(glm::vec3(forward.x, forward.y, forward.z), angle, glm::vec3(axis.x, axis.y, axis.z));
			right = qRotate(glm::vec3(right.x, right.y, right.z), angle, glm::vec3(axis.x, axis.y, axis.z));
			up = qRotate(glm::vec3(up.x, up.y, up.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		}
		else if (collWith == 'a') {
			vec3 newUp = get_ant_normal_from_position(obsticleCollisionId, { x,y,z });
			float angle = angleBetweenTwoVec3(up, newUp);
			vec3 axis = cross_vec3(up, newUp);
			axis = axis.normalize();
			if (angle == 0) {
				//printf("same");
				return;
			}
			//up = newUp;
			forward = qRotate(glm::vec3(forward.x, forward.y, forward.z), angle, glm::vec3(axis.x, axis.y, axis.z));
			right = qRotate(glm::vec3(right.x, right.y, right.z), angle, glm::vec3(axis.x, axis.y, axis.z));
			up = qRotate(glm::vec3(up.x, up.y, up.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		}
	}

	void handleObsticle(int id, char t) {
		//collision = true;
		//check line of sight first. if this obsticle is blocking then we do the pfm
		obsticleCollisionId = id;
		collWith = t;


		if (pfm != PathFindingMode::objectTracing && checkLineOfSight()) {
			pfm = PathFindingMode::objectTracing;
		}

	}

	void addToObsticleList(int id, char t) {
		obsticleCollisionIds[t].push_back(id);
		//printf("size of obsticles is %i", obsticleCollisionIds[t].size());
	}

	bool checkLineOfSight() {
		bool contact = false;

		//printf("size of obsticles is %i", obsticleCollisionIds['p'].size());
		for (std::pair<char, std::vector<int>> element : obsticleCollisionIds)
		{
			contact = false;
			//printf("first element is %c", element.first[0]);
			if (element.second.size() > 0) {

				if (element.first == 'p') {
					//printf("platform check");
					for (int i = 0; i < element.second.size(); i++) {
						Platform plat = getPlatform<Platform>(element.second[i]);
						if (plat.tp == box) {
							contact = LineAABBIntersection(plat, { x,y,z }, destination);
							//printf("we got contact %i at id:%i", contact,id);
						}
						else if (plat.tp == sphere) {
							contact = LineSphereIntersection(plat, { x,y,z }, destination);
						}
						if (contact == true) {
							//printf("Path Blocked");
							return contact;
						}
					}
				}
				else if (element.first == 'a') {
					//printf("particle check");
					for (int i = 0; i < element.second.size(); i++) {
						Particle p = getParticle<Particle>(element.second[i]);
						contact = LineSphereIntersection(p, { x,y,z }, destination);
						if (contact == true) {
							//printf("Path Blocked");
							return contact;
						}
					}
				}

			}
		}

		return contact;
	}

	void clearObsticleList() {
		//obsticleCollisionIds.clear();
		for (std::pair<char, std::vector<int>> element : obsticleCollisionIds)
		{
			obsticleCollisionIds[element.first] = {};
		}
	}

	void setMovementType(PathFindingMode m) {
		pfm = m;
	}

	void setDestination(vec3 dest) {
		destination = dest;
		foundDest = false;
		calculateMovement();
		//update_rotations();
	};

	void setSleep(int n) {
		if (n == 1) {
			isSleep = true;
			isStatic = true;
		}
		else {
			isSleep = false;
			isStatic = false;
		}
	}
};