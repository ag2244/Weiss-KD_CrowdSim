/*
 * OGL01Shape3D.cpp: 3D Shapes
 */
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <glew.h>//glew extention for shaders
#include <freeglut.h>  // GLUT, include glu.h and gl.h
#include <string.h>
#include <math.h>
#include "main.h"
#include "Shader.h"
#include <memory>
#include <unordered_map>
#include <algorithm>
#include <vector>

#define EPS 1e-9
 // camera
//Shader tmp = { 0 };
//Shader* ourShader=&tmp;
GLuint TexID1, TexID2;      // Handles to our textures

vec3 cameraPos = { -2.0f, 1.0f, -1.0f };
vec3 cameraFront = { 0.0f, 0.0f, -1.0f };
vec3 cameraUp = { 0.0f, 1.0f, 0.0f };

vec3 glob_up = { 0.0f,1.0f,0.0f };

bool firstMouse = true;
float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch = 0.0f;
float lastX = 800.0f / 2.0;
float lastY = 600.0 / 2.0;
float fov = 45.0f;
bool debug = true;

int frame = 0;

//in simulation
float particle_radii = .02;
float time_delta = 1 / 64.;
float particle_distance = particle_radii * 2.5;
int num_of_particles = 100;
float gravity = -0.01;
bool reachedGoal = false;
const float mui_static = 1.f;    // 0.00023; //0.021;
const float mui_kinematic = 0.8f; // 0.00017; //0.02;

#define OUT_PATH "frames/frame"

GLuint texture[2];


struct Image {

	unsigned long sizeX;

	unsigned long sizeY;

	char* data;

};

typedef struct Image Image;


#define checkImageWidth 64

#define checkImageHeight 64


GLubyte checkImage[checkImageWidth][checkImageHeight][3];

void makeCheckImage(void) {

	int i, j, c;

	for (i = 0; i < checkImageWidth; i++) {

		for (j = 0; j < checkImageHeight; j++) {

			c = ((((i & 0x8) == 0) ^ ((j & 0x8) == 0))) * 255;

			checkImage[i][j][0] = (GLubyte)c;

			checkImage[i][j][1] = (GLubyte)c;

			checkImage[i][j][2] = (GLubyte)c;

		}

	}

}



int ImageLoad(char* filename, Image* image) {

	FILE* file;

	unsigned long size; // size of the image in bytes.

	unsigned long i; // standard counter.

	unsigned short int planes; // number of planes in image (must be 1)

	unsigned short int bpp; // number of bits per pixel (must be 24)

	char temp; // temporary color storage for bgr-rgb conversion.

	// make sure the file is there.

	if ((file = fopen(filename, "rb")) == NULL) {

		printf("File Not Found : %s\n", filename);

		return 0;

	}

	// seek through the bmp header, up to the width/height:

	fseek(file, 18, SEEK_CUR);

	// read the width

	if ((i = fread(&image->sizeX, 4, 1, file)) != 1) {

		printf("Error reading width from %s.\n", filename);

		return 0;

	}

	//printf("Width of %s: %lu\n", filename, image->sizeX);

	// read the height

	if ((i = fread(&image->sizeY, 4, 1, file)) != 1) {

		printf("Error reading height from %s.\n", filename);

		return 0;

	}

	//printf("Height of %s: %lu\n", filename, image->sizeY);

	// calculate the size (assuming 24 bits or 3 bytes per pixel).

	size = image->sizeX * image->sizeY * 3;

	// read the planes

	if ((fread(&planes, 2, 1, file)) != 1) {

		printf("Error reading planes from %s.\n", filename);

		return 0;

	}

	if (planes != 1) {

		printf("Planes from %s is not 1: %u\n", filename, planes);

		return 0;

	}

	// read the bitsperpixel

	if ((i = fread(&bpp, 2, 1, file)) != 1) {

		printf("Error reading bpp from %s.\n", filename);

		return 0;

	}

	if (bpp != 24) {

		printf("Bpp from %s is not 24: %u\n", filename, bpp);

		return 0;

	}

	// seek past the rest of the bitmap header.

	fseek(file, 24, SEEK_CUR);

	// read the data.

	image->data = (char*)malloc(size);

	if (image->data == NULL) {

		printf("Error allocating memory for color-corrected image data");

		return 0;

	}

	if ((i = fread(image->data, size, 1, file)) != 1) {

		printf("Error reading image data from %s.\n", filename);

		return 0;

	}

	for (i = 0; i < size; i += 3) { // reverse all of the colors. (bgr -> rgb)

		temp = image->data[i];

		image->data[i] = image->data[i + 2];

		image->data[i + 2] = temp;

	}

	// we're done.

	return 1;

}


Image* loadTexture(std::string filename) {

	Image* image1 = {};

	// allocate space for texture

	image1 = (Image*)malloc(sizeof(Image));

	if (image1 == NULL) {

		printf("Error allocating space for image");

		exit(0);

	}
	std::string str = filename;
	char* writable = new char[str.size() + 1];
	std::copy(str.begin(), str.end(), writable);
	writable[str.size()] = '\0';
	if (!ImageLoad(writable, image1)) {

		exit(1);

	}

	return image1;

}

void setCameraPosition(float x, float y, float z) {
	cameraPos.x = x;
	cameraPos.y = y;
	cameraPos.z = z;
}


float distance(float x1, float y1, float z1, float x2, float y2, float z2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
}

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
	Direction dir = static_cast<Direction>(rand() % Direction::down);
	return dir;
}

std::vector<Platform>platforms;
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
	float px = 0;
	float py = 0;
	float pz = 0;
	float r = particle_radii;
	float mass = 1;
	float inv_mass = 1/mass;
	float speed = .02;
	int timer = 0;
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
	Direction dir;
	bool isStatic = false;
	vec3 timerPosition = { 0,0,0 };
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
		else if (timer > 200) {
			if (isStatic == false) {
				if (distance(x, y, z, timerPosition.x, timerPosition.y, timerPosition.z) < r * 1.1) {
					isStatic = true;
				}
				
			}
			timer = -1;
		}

		if (isStatic == false) {
			mass = 1;
			//check if im in object tracing mode
			//if i can make a straight uninterupted line to the goal switch back to simple movements
			if (pfm == PathFindingMode::objectTracing) {
				bool ObsticleInPath = checkLineOfSight();
				//printf("line of sight checked");
				if (ObsticleInPath == true) {
					calculatePredictedMovement();
					//printf("path blocked");
				}
				else {
					pfm = PathFindingMode::simple;
					//reset stuff here
					startObjectTrace = false;
					//printf("back to simple");
				}
				//printf("objectTracing mode");
			}
			else {
				calculateMovement();
				//printf("simple mode ");
			}
		}
		else {
			
			mass = 2000;
			vx = 0;
			vy = 0;
			vz = 0;
			if (attachedType == 'p') {
				vy = -gravity * time_delta;
			}
			if (timer > 200) {
				//printf("check up");
				bool coll = checkLineOfSight();
				if (coll == false) {
					isStatic = false;
					timer = -1;
					//printf("can move");
				}
			}
			
		}
		checkGoal();
		update_rotations();
		timer++;
	};

	void checkGoal() {
		if (distance(x, y, z, destination.x, destination.y, destination.z) <= r*2) {
			reachedGoal = true;
		}
	}

	void calculateMovement() {
		vx = 0;
		vy = 0;//gravity *speed* .99 * time_delta; maybe turn gravity on when on bridge
		vz = 0;
		if (x > destination.x)
			vx = -1.0f * speed * .99 * time_delta;
		else if (x < destination.x)
			vx = 1.0f * speed * .99 * time_delta;
		if (y > destination.y)
			vy = (-1.0f * speed) * .99* time_delta;
		else if (y < destination.y)
			vy = (1.0f *speed )* .99* time_delta;
		if (z > destination.z)
			vz = -1.0f * speed * .99 * time_delta;
		else if (z < destination.z)
			vz = 1.0f * speed * .99 * time_delta;
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
				if (collWith == 'p') {
					//printf("checking p collision");
					Platform plat = getPlatform<Platform>(obsticleCollisionId);
					if (plat.tp == box) {
						contactPoint = closestPointOnAABB(destination, plat);
					}
					else if (plat.tp == sphere) {
						contactPoint = closestPointOnSphere(destination, plat);
					}
					dir = Direction::up;// getRandomDir();
				}
				else if (collWith == 'a') {
					//printf("checking a collision");
					contactPoint = getClosestPointToSphere(destination, r, { x,y,z });
					dir = Direction::up;
				}
			}
			//// Direction::up;
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
		if (distance(x, y, z, contactPoint.x, contactPoint.y, contactPoint.z) <= r*2) {
			pfm = PathFindingMode::simple;
			//printf("founddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
			startObjectTrace = false;
			return;
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
							bool contact=false;
							if (plat.tp == box) {
								contact = LineAABBIntersection(plat, { predx,predy,predz }, { predx + vx *200,predy + vy *200,predz + vz*200 });
							}
							else if (plat.tp == sphere) {
								contact = LineSphereIntersection(plat, { predx ,predy,predz}, { predx + vx * 200,predy + vy * 200,predz + vz * 200 });
							}
							if (contact == false) {
								//printf("Path clear");
								return ;
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
		
	}

	void update_rotations() {
		vec3 dir = { vx,vy,vz };
		float angle = angleBetweenTwoVec3(forward, dir);
		vec3 axis = cross_vec3(forward, dir);
		axis = axis.normalize();
		if (angle == 0)
			return;
		//printf("velocity %f,%f,%f angle %f ", vx, vy, vz, angle);
		forward = qRotate(glm::vec3(forward.x, forward.y, forward.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		right = qRotate(glm::vec3(right.x, right.y, right.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		up = qRotate(glm::vec3(up.x, up.y, up.z), angle, glm::vec3(axis.x, axis.y, axis.z));
		//printf("quat is %f,%f,%f ", r.x, r.y, r.z);
		projforward = up;

	}
	void handleObsticle(int id, char t) {
		//collision = true;
		//check line of sight first. if this obsticle is blocking then we do the pfm
		obsticleCollisionId = id;
		collWith = t;

		
		if (pfm != PathFindingMode::objectTracing&& checkLineOfSight()) {
			pfm = PathFindingMode::objectTracing;
			//printf("pfm time");
		}

	}
	void addToObsticleList(int id,char t) {
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
		update_rotations();
	};
};

std::vector<Particle> particles;

/******************
* KD TREE ADDITIONS  
******************/

class KD_Tree
{

private:

	//Node for a particle
	class Node_Particle
	{
	public:

		size_t begin; //beginning node number
		size_t end; //ending node number

		size_t left; //left node number
		size_t right; //right node number

		int k;
		std::vector<float> k_vals;
		
		std::vector<float> k_max;
		std::vector<float> k_min;
	};

	explicit KD_Tree(); //explicit KdTree(RVOSimulator *sim);

	~KD_Tree();

	void buildParticleTree();

	void buildParticleTreeRecursive(size_t begin, size_t end, size_t node, int k);

	void traverseParticleTree();

	std::vector<Particle*> kd_particles;
	std::vector<Node_Particle> kd_particleTree;

	static const size_t MAX_LEAF_SIZE = 10;

	friend class Particle;
};

KD_Tree::~KD_Tree()
{

}

void KD_Tree::buildParticleTree()
{
	//Add each additional particle to the tree, and update k vals
	for (size_t i = 0; i < particles.size(); i++)
	{
		if (kd_particles.size() <= i)
		{
			kd_particles.push_back(&particles[i]);
		}

		kd_particleTree[i].k_vals[0] = particles[i].x;
		kd_particleTree[i].k_vals[0] = particles[i].y;
		kd_particleTree[i].k_vals[0] = particles[i].z;
	}

	//If there are more particles than the size of the particles tree
	if (kd_particles.size() < particles.size())
	{
		kd_particleTree.resize((kd_particles.size() * 2) - 1);
	}

	if (!kd_particles.empty())
	{
		//Build the particle tree, with beginning = 0, node = first node (0) and ending = last node (the size of the tree)
		buildParticleTreeRecursive(0, kd_particles.size(), 0, 0);
	}
}

void KD_Tree::buildParticleTreeRecursive(size_t begin, size_t end, size_t node, int k)
{
	//Reset k if it is too much
	if (k >= 3) { k = 0; }

	kd_particleTree[node].begin = begin; //Index of beginning node
	kd_particleTree[node].end = end; //Index of ending node

	kd_particleTree[node].k_min[0] = kd_particleTree[node].k_max[0] = kd_particles[begin]->x;
	kd_particleTree[node].k_min[1] = kd_particleTree[node].k_max[1] = kd_particles[begin]->y;
	kd_particleTree[node].k_min[2] = kd_particleTree[node].k_max[2] = kd_particles[begin]->z;

	//Get max X, min X, max Y and min Y values of all the nodes between beginning and end
	for (size_t i = begin + 1; i < end; ++i) 
	{
		//max X is the maximum X value of all the nodes between beginning and end, min X is similar
		kd_particleTree[node].k_max[0] = std::max(kd_particleTree[node].k_max[0], kd_particles[i]->x);
		kd_particleTree[node].k_min[0] = std::min(kd_particleTree[node].k_min[0], kd_particles[i]->x);

		//max Y is the maximum Y value of all the nodes between beginning and end, minimum Y is similar
		kd_particleTree[node].k_max[1] = std::max(kd_particleTree[node].k_max[1], kd_particles[i]->y);
		kd_particleTree[node].k_min[1] = std::min(kd_particleTree[node].k_min[1], kd_particles[i]->y);

		//max Y is the maximum Y value of all the nodes between beginning and end, minimum Y is similar
		kd_particleTree[node].k_max[2] = std::max(kd_particleTree[node].k_max[2], kd_particles[i]->z);
		kd_particleTree[node].k_min[2] = std::min(kd_particleTree[node].k_min[2], kd_particles[i]->z);
	}

	//If the space between the ending and beginning nodes is bigger than the size of the leaf
	if (end - begin > MAX_LEAF_SIZE)
	{
		
		//splitValue = median x if isVertical, median y if not
		const float splitValue = 0.5f * (kd_particleTree[node].k_max[0] + kd_particleTree[node].k_min[0]);

		size_t left = begin;
		size_t right = end;

		while (left < right)
		{
			//Increment left while:
			//left < right and: 
			//particle at index 'left''s x is less than the average x (if isVertical), 
			//or its y is less than the average y 
			while ((left < right) && (kd_particleTree[node].k_vals[left] < splitValue))
				left++;

			//Decrement right while:
			//left < right and: 
			//particle at index 'right - 1''s x is greater than/equal to the average x (if isVertical), 
			//or its y is greater than/equal to the average y 
			while ((right > left) && (kd_particleTree[node].k_vals[right - 1] >= splitValue))
				right--;

			//If left still < right, swap the particles at index left and right, increment left, decrement right.
			if (left < right)
			{
				std::swap(kd_particles[left], kd_particles[right]);
				left++;
				right--;
			}

			//increment both left and right if left == the beginning index
			if (left == begin)
			{
				left++;
				right++;
			}

			//node's left should be the next index
			kd_particleTree[node].left = node + 1;
			//node's right should be node plus 
			kd_particleTree[node].right = node + 2 * (left - begin);

			buildParticleTreeRecursive(begin, left, kd_particleTree[node].left, k + 1);
			buildParticleTreeRecursive(left, end, kd_particleTree[node].right, k + 1);
		}
	}
	
}

void traverseParticleTree()
{

}

/**********************
* END KD TREE ADDITIONS
**********************/

template<typename T>
T getParticle(int id)
{
	return particles[id];
}

/* Initialize OpenGL Graphics */
void initGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
	glClearDepth(1.0f);                   // Set background depth to farthest
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess[] = { 50.0 };
	GLfloat light_position[] = { 0.0, 0.20, 0.0, 0.0 };
	GLfloat light_position2[] = { -2.0, 0.20, 0.0, 0.0 };
	//glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position2);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

}

//draw cube
void drawCube(float x, float y, float z, float w, float h, float l) {
	glDisable(GL_LIGHTING);
	glPushMatrix();
	glTranslatef(x,y,z);  // Move right and into the screen

	glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads

	glVertex3f(w, h, -l);
	//glTexCoord2f(1, 0);
	//ourShader->setVec2("aTexCoords", glm::vec2(1, 0));
	//glVertexAttrib2f(texcoord_index, 1, 0);
	glVertex3f(-w, h, -l);
	//glTexCoord2f(0, 0);
	//ourShader->setVec2("aTexCoords", glm::vec2(0, 0));
	//glVertexAttrib2f(texcoord_index, 0, 0);
	glVertex3f(-w, h, l);
	//glTexCoord2f(0, 1);
	//ourShader->setVec2("aTexCoords", glm::vec2(0, 0));
	//glVertexAttrib2f(texcoord_index, 0, 1);
	glVertex3f(w, h, l);
	//glDisable(GL_TEXTURE_2D);
	// Bottom face (y = -1.0f)
	glColor3f(1.0f, 1.0f, 1.0f);      // Orange
	glVertex3f(w, -h, l);
	glVertex3f(-w, -h, l);
	glVertex3f(-w, -h, -l);
	glVertex3f(w, -h, -l);

	// Front face  (z = 1.0f)
	glColor3f(0.0f, 0.0f, 1.0f);      // Red
	//glVertexAttrib2f(texcoord_index, 1, 1);
	glVertex3f(w, h, l);
	//glVertexAttrib2f(texcoord_index, 1, 0);
	glVertex3f(-w, h, l);
	//glVertexAttrib2f(texcoord_index, 0, 0);
	glVertex3f(-w, -h, l);
	//glVertexAttrib2f(texcoord_index, 0, 1);
	glVertex3f(w, -h, l);

	// Back face (z = -1.0f)
	glColor3f(0.0f, 0.0f, 1.0f);      // Yellow
	glVertex3f(w, -h, -l);
	glVertex3f(-w, -h, -l);
	glVertex3f(-w, h, -l);
	glVertex3f(w, h, -l);

	// Left face (x = -1.0f)
	glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	glVertex3f(-w, h, l);
	glVertex3f(-w, h, -l);
	glVertex3f(-w, -h, -l);
	glVertex3f(-w, -h, l);

	// Right face (x = 1.0f)
	glColor3f(0.0f, 0.0f, 1.0f);      // Magenta
	glVertex3f(w, h, -l);
	glVertex3f(w, h, l);
	glVertex3f(w, -h, l);
	glVertex3f(w, -h, -l);
	glColor3f(1.0f, 1.0f, 1.0f);

	glEnd();  // End of drawing color-cube
	glPopMatrix();
	//glBindTexture(GL_TEXTURE_2D, 0);
	glEnable(GL_LIGHTING);
}


void drawSphere(float x, float y, float z, float r) {
	
	GLUquadric* quad = gluNewQuadric();
	glPushMatrix();
	//glBindTexture(GL_TEXTURE_2D, texture[0]);
	//glActiveTexture(GL_TEXTURE0);
	//glEnable(GL_TEXTURE_2D);
	glTranslatef(x, y,z);
	gluSphere(quad, r, 10, 10);
	glPopMatrix();
	glEnd();
	//glDisable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, 0);
}

void drawLine(float x1, float y1, float z1, float x2, float y2, float z2) {
	glPushMatrix();
	//ourShader->setVec3("transPosition", glm::vec3(x1, y1, z1));
	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 1.0f);

	glVertex3f(x1, y1, z1);
	//ourShader->setVec3("transPosition", glm::vec3(x2, y2, z2));
	glVertex3f(x2, y2, z2);
	glEnd();
	glPopMatrix();
}

//vector and math
vec3 add_vec3(vec3 a, vec3 b) {
	vec3 newv;
	newv.x = a.x + b.x;
	newv.y = a.y + b.y;
	newv.z = a.z + b.z;

	return newv;
}

vec3 sub_vec3(vec3 a, vec3 b) {
	vec3 newv;
	newv.x = a.x - b.x;
	newv.y = a.y - b.y;
	newv.z = a.z - b.z;

	return newv;
}
float radians(double degree)
{
	return (degree * (pi / 180));
}

vec3 norm_vec3(vec3 vec)
{
	float len = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
	vec3 newv = { 0,0,0 };
	if (len != 0.0f)
	{
		newv.x = vec.x / len;
		newv.y = vec.y / len;
		newv.z = vec.z / len;
	}
	return newv;
}

vec3 cross(vec3 a, vec3 b) {
	vec3 newv;
	newv.x = a.y * b.z - a.z * b.y;
	newv.y = a.z * b.x - a.x * b.z;
	newv.z = a.x * b.y - a.y * b.x;

	return newv;
}

vec3 scale_vec3(vec3 a, float b) {
	vec3 newv;
	newv.x = a.x * b;
	newv.y = a.y * b;
	newv.z = a.z * b;

	return newv;
}

vec3 abs_vec3(vec3 a) {
	vec3 newv;
	newv.x = abs(a.x);
	newv.y = abs(a.y);
	newv.z = abs(a.z);
	return newv;
}

float length_vec3(vec3 a) {
	return sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}

vec3 cross_vec3(vec3 a, vec3 b) {
	vec3 newv;
	newv.x = a.y * b.z - a.z * b.y;
	newv.y = -(a.x * b.z - a.z * b.x);
	newv.z = a.x * b.y - a.y * b.x;
	return newv;
}

float dot_vec3(vec3 a, vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float dist_vec3(vec3 a, vec3 b) {
	return distance(a.x, a.y, a.z, b.x, b.y, b.z);
}

float angleBetweenTwoVec3(vec3 a, vec3 b) {
	float angle = acos((dot_vec3(a, b)) / (length_vec3(a) * length_vec3(b)));

	vec3 vec = cross_vec3(a, b);
	vec = norm_vec3(vec);
	if (vec.z == 1) {
		//angle *= -1;
		//printf("reverse");
	}
	if (vec.z > 0 && vec.z < 1) {
		//angle *= -1;

		//printf("reverse");
	}
	if (isnan(angle) || isinf(angle))
		angle = 0;
	return angle;
}

vec3 rotate_around_axis_vec3(vec3 v, vec3 r, float a)
{
	float deg = a * 180 / pi;

	vec3 q = (1 - cos(a)) * v.dot(r) * r + cos(a) * v + sin(a) * r.cross(v);
	//vec3 q = add_vec3(add_vec3(scale_vec3(r, (1 - cos(a)) * (dot_vec3(v, r))), scale_vec3(v, cos(a))), scale_vec3(cross_vec3(r, v), sin(a)));
	return q;
}

vec3 qRotate(glm::vec3 point, float angle, glm::vec3 axis)
{
	//Create quaternion.
	//glm::quat orientation = glm::quat(rotate);
	glm::quat orientation = glm::angleAxis(angle, axis);
	//Normalize
	orientation = glm::normalize(orientation);

	/*Method 1 - pure vector quaternion.*/
	//Create the 'pure' vector quaternion
	glm::quat pure = glm::quat(0.0, point.x, point.y, point.z);
	glm::quat qpq = orientation * pure * (glm::conjugate(orientation));
	//Form the result
	//glm::vec3 result = glm::vec3(qpq.x, qpq.y, qpq.z);
	//printf("\nResult1 = %f %f %f", result.x, result.y, result.z);

	/*Method 2 - just apply the orientation to the point.*/
	glm::vec3 result = orientation * point;
	if (isnan(result.x)) {
		return {point.x,point.y,point.z};
		printf("nanana");
	}
	//printf("\nResult2 = %f %f %f", alpha.x, alpha.y, alpha.z);
	return { result.x, result.y, result.z };
}
/* Handler for window-repaint event. Called back when the window first appears and
   whenever the window needs to be re-painted. */
void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	glMatrixMode(GL_MODELVIEW);
	//gluPerspective(45.0, SCR_WIDTH / SCR_HEIGHT, 0.1, 100.0);
	glLoadIdentity();
	gluLookAt(cameraPos.x, cameraPos.y, cameraPos.z, cameraPos.x + cameraFront.x, cameraPos.y + cameraFront.y, cameraPos.z + cameraFront.z, cameraUp.x, cameraUp.y, cameraUp.z);
	// Render a color-cube consisting of 6 quads with different colors

	//draw platforms
	glm::vec3 lightPos(-1.0f, .20f, -1.0f);
	//ourShader->use();
	//ourShader->setVec3("lightPos", lightPos);
	//ourShader->setVec3("viewPos", glm::vec3(cameraPos.x, cameraPos.y, cameraPos.z));

	glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
	glm::mat4 view = glm::lookAt(glm::vec3(cameraPos.x, cameraPos.y, cameraPos.z), glm::vec3(cameraPos.x + cameraFront.x, cameraPos.y + cameraFront.y, cameraPos.z + cameraFront.z), glm::vec3(cameraUp.x, cameraUp.y, cameraUp.z));
	//ourShader->setMat4("projection", projection);
	//ourShader->setMat4("view", view);

	glm::mat4 model = glm::mat4(1.0f);
	//ourShader->setMat4("model", model);
	//ourShader->setInt("blinn", false);

	for (int i = 0; i < platforms.size(); i++) {
		if (platforms[i].tp == box)
			drawCube(platforms[i].x, platforms[i].y, platforms[i].z, platforms[i].w, platforms[i].h, platforms[i].l);
		else if (platforms[i].tp == sphere)
			drawSphere(platforms[i].x, platforms[i].y, platforms[i].z, platforms[i].r);
	}

	//light source
	//drawSphere(lightPos.x, lightPos.y, lightPos.z, .1);
	//Draw Particles
	//glDisable(GL_LIGHTING);
	glColor3f(1.0f, 0.0f, 0.0f);
	//drawSphere(particles[0].destination.x, particles[0].destination.y, particles[0].destination.z, particles[0].r);
	//glEnable(GL_LIGHTING);
	for (int i = 0; i < particles.size(); i++) {
		
		//if (particles[i].startObjectTrace == true)
			//drawSphere(particles[0].contactPoint.x, particles[0].contactPoint.y, particles[0].contactPoint.z, particles[0].r);
		if (debug == true) {
			if (particles[i].isStatic) {
				//glDisable(GL_LIGHTING);
				glColor3f(0.0f, 1.0f, 0.0f);
				drawSphere(particles[i].x, particles[i].y, particles[i].z, particles[i].r);
				//glEnable(GL_LIGHTING);
			}
			else {
				if(i<50)
					glColor3f(1.0f, 0.0f, 1.0f);
				else
					glColor3f(0.6f, 0.2f, 1.0f);
				drawSphere(particles[i].x, particles[i].y, particles[i].z, particles[i].r);
			}
			//drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x+ particles[i].vx*1000, particles[i].y+ particles[i].vy*1000, particles[i].z+ particles[i].vz*1000);
			//drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x + particles[i].forward.x * 2, particles[i].y + particles[i].forward.y * 2, particles[i].z + particles[i].forward.z * 2);
			//drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x + particles[i].right.x * 2, particles[i].y + particles[i].right.y * 2, particles[i].z + particles[i].right.z * 2);
			//drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x + particles[i].up.x * 100, particles[i].y + particles[i].up.y * 100, particles[i].z + particles[i].up.z * 100);
			//glDisable(GL_LIGHTING);
			//glColor3f(1.0f, 0.0f, 0.0f);
			//drawSphere(particles[i].destination.x, particles[i].destination.y, particles[i].destination.z, particles[i].r);
			//drawSphere(particles[i].contactPoint.x, particles[i].contactPoint.y, particles[i].contactPoint.z, particles[i].r);
			//glEnable(GL_LIGHTING);
		}
		else {
			glColor3f(1.0f, 0.0f, 1.0f);
			drawSphere(particles[i].x, particles[i].y, particles[i].z, particles[i].r);
		}
	}
	//updateCamera();
	glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}

/* Handler for window re-size event. Called back when the window first appears and
   whenever the window is re-sized with its new width and height */
void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
   // Compute aspect ratio of the new window
	if (height == 0) height = 1;                // To prevent divide by 0
	GLfloat aspect = (GLfloat)width / (GLfloat)height;

	// Set the viewport to cover the new window
	glViewport(0, 0, width, height);

	// Set the aspect ratio of the clipping volume to match the viewport
	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();             // Reset
	// Enable perspective projection with fovy, aspect, zNear and zFar
	gluPerspective(45.0f, aspect, 0.1f, 100.0f);
}


void mouse(int button, int state, int screenx, int screeny) {
	if (firstMouse)
	{
		lastX = screenx;
		lastY = screeny;
		firstMouse = false;
	}

	float xoffset = screenx - lastX;
	float yoffset = lastY - screeny; // reversed since y-coordinates go from bottom to top
	lastX = screenx;
	lastY = screeny;

	float sensitivity = 0.1f; // change this value to your liking
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	yaw += xoffset;
	pitch += yoffset;

	// make sure that when pitch is out of bounds, screen doesn't get flipped
	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	vec3 front;
	front.x = cos(radians(yaw)) * cos(radians(pitch));
	front.y = sin(radians(pitch));
	front.z = sin(radians(yaw)) * cos(radians(pitch));
	cameraFront = norm_vec3(front);
	printf("new cam coord %f,%f,%f", cameraPos.x, cameraPos.y, cameraPos.z);

}

void mousePassive(int screenx, int screeny) {

	if (firstMouse)
	{
		lastX = screenx;
		lastY = screeny;
		firstMouse = false;
	}


	float xoffset = screenx - lastX;
	float yoffset = lastY - screeny; // reversed since y-coordinates go from bottom to top
	lastX = screenx;
	lastY = screeny;

	float sensitivity = 0.1f; // change this value to your liking
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	yaw += xoffset;
	pitch += yoffset;

	// make sure that when pitch is out of bounds, screen doesn't get flipped
	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	vec3 front;
	front.x = cos(radians(yaw)) * cos(radians(pitch));
	front.y = sin(radians(pitch));
	front.z = sin(radians(yaw)) * cos(radians(pitch));
	cameraFront = norm_vec3(front);
	//printf("camera move");
}

void processSpecialKeys(int key, int x, int y) {

	float camera_speed = 0.1f;
	vec3 cross_cam;
	vec3 ups;
	switch (key) {
	case GLUT_KEY_LEFT:
		cross_cam = cross(cameraFront, cameraUp);
		cross_cam = norm_vec3(cross_cam);
		cross_cam = scale_vec3(cross_cam, -camera_speed);
		cameraPos = add_vec3(cameraPos, cross_cam);
		break;
	case GLUT_KEY_RIGHT:
		cross_cam = cross(cameraFront, cameraUp);
		cross_cam = norm_vec3(cross_cam);
		cross_cam = scale_vec3(cross_cam, camera_speed);
		cameraPos = add_vec3(cameraPos, cross_cam);
		break;
	case GLUT_KEY_UP:
		ups = scale_vec3(cameraFront, camera_speed);
		cameraPos = add_vec3(cameraPos, ups);
		break;
	case GLUT_KEY_DOWN:
		ups = scale_vec3(cameraFront, -camera_speed);
		cameraPos = add_vec3(cameraPos, ups);
		break;
	}
}
float angle = 0;
void keyboard(unsigned char key, int x, int y) {
	if (key == 27)
		exit(0);
	else if (key == 'd') {
		debug = !debug;
	}
}
//collision
bool pointRect(vec3 point, Platform box) {
	float minX = box.x - box.w;
	float minY = box.y - box.h;
	float minZ = box.z - box.l;
	float maxX = box.x + box.w;
	float maxY = box.y + box.h;
	float maxZ = box.z + box.l;

	return (point.x >= minX && point.x <= maxX) &&
		(point.y >= minY && point.y <= maxY) &&
		(point.z >= minZ && point.z <= maxZ);
}

bool sphereRect(Particle sphere, Platform box) {
	// get box closest point to sphere center by clamping
	float minX = box.x - box.w;
	float minY = box.y - box.h;
	float minZ = box.z - box.l;
	float maxX = box.x + box.w;
	float maxY = box.y + box.h;
	float maxZ = box.z + box.l;

	float x = std::max(minX, std::min(sphere.x, maxX));
	float y = std::max(minY, std::min(sphere.y, maxY));
	float z = std::max(minZ, std::min(sphere.z, maxZ));

	// this is the same as isPointInsideSphere
	float distance = sqrt((x - sphere.x) * (x - sphere.x) +
		(y - sphere.y) * (y - sphere.y) +
		(z - sphere.z) * (z - sphere.z));

	return distance < sphere.r;
}


template <typename T>
bool sphereSphere(Particle a, T b) {
	float distance = sqrt((a.x - b.x) * (a.x - b.x) +
		(a.y - b.y) * (a.y - b.y) +
		(a.z - b.z) * (a.z - b.z));
	return distance < (a.r + b.r);
}

bool pointObsticle(vec3 point, int obsticleId) {
	Platform plat = getPlatform<Platform>(obsticleId);
	if (plat.tp == box) {
		return pointRect(point, plat);
	}
	return false;

}

bool particleObsticle(vec3 pos, int obsticleId, char t) {
	if (t == 'p') {
		Platform obst = getPlatform<Platform>(obsticleId);
		if (obst.tp == box) {
			Particle part = Particle(pos.x, pos.y, pos.z, -1);
			return sphereRect(part, obst);
		}
		else if (obst.tp == sphere) {
			Particle part = Particle(pos.x, pos.y, pos.z, -1);
			return sphereSphere(part, obst);
		}
	}
	//else if(t=='a') {
		//Particle obst = getParticle<Particle>(obsticleId);
		//Particle part = Particle(pos.x, pos.y, pos.z, -1);
		//return sphereSphere(part, obst);
	//}
	return false;
}

//ray box collision from https://github.com/BSVino/MathForGameDevelopers/blob/line-box-intersection/math/collision.cpp
bool ClipLine(int d, Platform aabbBox, vec3 v0, vec3 v1, float f_low, float f_high)
{
	// f_low and f_high are the results from all clipping so far. We'll write our results back out to those parameters.

	// f_dim_low and f_dim_high are the results we're calculating for this current dimension.
	float f_dim_low, f_dim_high;

	float minX = aabbBox.x - aabbBox.w;
	float minY = aabbBox.y - aabbBox.h;
	float minZ = aabbBox.z - aabbBox.l;
	float maxX = aabbBox.x + aabbBox.w;
	float maxY = aabbBox.y + aabbBox.h;
	float maxZ = aabbBox.z + aabbBox.l;

	// Find the point of intersection in this dimension only as a fraction of the total vector http://youtu.be/USjbg5QXk3g?t=3m12s
	if (d == 0) {
		f_dim_low = (minX - v0.x) / (v1.x - v0.x);
		f_dim_high = (maxX - v0.x) / (v1.x - v0.x);
	}
	if (d == 1) {
		f_dim_low = (minY - v0.y) / (v1.y - v0.y);
		f_dim_high = (maxY - v0.y) / (v1.y - v0.y);
	}
	if (d == 2) {
		f_dim_low = (minZ - v0.z) / (v1.z - v0.z);
		f_dim_high = (maxZ - v0.z) / (v1.z - v0.z);
	}
	// Make sure low is less than high
	if (f_dim_high < f_dim_low)
		std::swap(f_dim_high, f_dim_low);

	// If this dimension's high is less than the low we got then we definitely missed. http://youtu.be/USjbg5QXk3g?t=7m16s
	if (f_dim_high < f_low)
		return false;

	// Likewise if the low is less than the high.
	if (f_dim_low > f_high)
		return false;

	// Add the clip from this dimension to the previous results http://youtu.be/USjbg5QXk3g?t=5m32s
	f_low = std::max(f_dim_low, f_low);
	f_high = std::min(f_dim_high, f_high);

	if (f_low > f_high)
		return false;

	return true;
}

// Find the intersection of a line from v0 to v1 and an axis-aligned bounding box http://www.youtube.com/watch?v=USjbg5QXk3g
template<typename T>
bool LineAABBIntersection(T aabbBox, vec3 v0, vec3 v1)
{
	float f_low = 0;
	float f_high = 1;

	if (!ClipLine(0, aabbBox, v0, v1, f_low, f_high))
		return false;

	if (!ClipLine(1, aabbBox, v0, v1, f_low, f_high))
		return false;

	if (!ClipLine(2, aabbBox, v0, v1, f_low, f_high))
		return false;

	return true;
}

template <typename T>
bool LineSphereIntersection(T sphere, vec3 v0, vec3 v1) {
	float a, b, c;
	float bb4ac;
	vec3 dp;

	dp.x = v1.x - v0.x;
	dp.y = v1.y - v0.y;
	dp.z = v1.z - v0.z;
	a = dp.x * dp.x + dp.y * dp.y + dp.z * dp.z;
	b = 2 * (dp.x * (v0.x - sphere.x) + dp.y * (v0.y - sphere.y) + dp.z * (v0.z - sphere.z));
	c = sphere.x * sphere.x + sphere.y * sphere.y + sphere.z * sphere.z;
	c += v0.x * v0.x + v0.y * v0.y + v0.z * v0.z;
	c -= 2 * (sphere.x * v0.x + sphere.y * v0.y + sphere.z * v0.z);
	c -= sphere.r * sphere.r;
	bb4ac = b * b - 4 * a * c;
	if (abs(a) < EPS || bb4ac < 0) {
		return false ;
	}

	return true;
}

void update_particles() {
	for (int i = 0; i < particles.size(); i++) {
		particles[i].update();
	}
}

void check_particle_platform_collision() {
	for (int i = 0; i < particles.size(); i++) {
		particles[i].clearObsticleList();
		bool coll = false;
		int collctr = 0;
		for (int j = 0; j < platforms.size(); j++) {
			if (platforms[j].tp == box) {
				coll = sphereRect(particles[i], platforms[j]);
			}
			else if (platforms[j].tp == sphere) {
				vec3 pos;
				coll = sphereSphere(particles[i], platforms[j]);
			}
			if (coll) {
				particles[i].addToObsticleList(j, 'p');
				//if(collctr==0)
				particles[i].handleObsticle(j, 'p');
				
				//printf("collided with %i",j);
				collctr++;
				//break;
			}
		}
	}
}

void check_particle_particle_collision() {
	for (int i = 0; i < particles.size(); i++) {
		//particles[i].clearObsticleList();
		bool coll = false;
		int collctr = 0;
		for (int j = 0; j < particles.size(); j++) {
			if (j == i )
				continue;
			coll = sphereSphere(particles[i], particles[j]);
			if (coll) {
				particles[i].addToObsticleList(j, 'a');
				if (particles[j].isStatic == true) {
				particles[i].handleObsticle(j, 'a');
				}
				
				//printf("collided with %i",j);
				collctr++;
				//break;
			}
		}
	}
}

float sdBox(vec3 p, vec3 b)
{
	vec3 q = sub_vec3(abs_vec3(p), b);
	//printf("sub %f,%f,%f", q.x, q.y, q.z);


	return length_vec3({ std::max(q.x,0.0f),std::max(q.y,0.0f),std::max(q.z,0.0f) }) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
}

float udBox(vec3 p, vec3 b)
{
	//printf("bounds %f,%f,%f ", b.x, b.y, b.z);
	vec3 absp = abs_vec3(p);
	vec3 value = sub_vec3(absp, b);
	


	if (value.x < 0.00) {
		value.x = 0.00;
	}

	if (value.y < 0.00) {
		value.y = 0.00;
	}

	if (value.z < 0.00) {
		value.z = 0.00;
	}

	return length_vec3(value);
}

float sdSphere(vec3 p, float s)
{
	return length_vec3(p) - s;
}

template<typename T>
vec3 closestPointOnAABB(vec3 p, T b)
{
	vec3 newv = { p.x,p.y,p.z };
	if (p.x > b.x + b.w) {
		newv.x = b.x + b.w;
	}
	else if (p.x < b.x - b.w) {
		newv.x = b.x - b.w;
	}

	if (p.y > b.y + b.h) {
		newv.y = b.y + b.h;
	}
	else if (p.y < b.y - b.h) {
		newv.y = b.y - b.h;
	}

	if (p.z > b.z + b.l) {
		newv.z = b.z + b.l;
	}
	else if (p.z < b.z - b.l) {
		newv.z = b.z - b.l;
	}


	return newv;
}
template<typename T>
vec3 closestPointOnSphere(vec3 p, T b) {
	// First, get a vetor from the sphere to the point
	vec3 spherePos = vec3{ b.x,b.y,b.z };
	vec3 sphereToPoint = sub_vec3(p, spherePos);
	// Normalize that vector
	sphereToPoint = sphereToPoint.normalize();
	// Adjust it's length to point to edge of sphere
	sphereToPoint = scale_vec3(sphereToPoint, b.r);
	// Translate into world space
	vec3 worldPoint = spherePos + sphereToPoint;
	// Return new point
	return worldPoint;
}

template<typename T>
T getPlatform(int id)
{
	return platforms[id];
}

vec3 getClosestPointToSphere(vec3 spherePos, float sphereRad, vec3 point) {
	// First, get a vetor from the sphere to the point
	vec3 sphereToPoint = sub_vec3(point, spherePos);
	// Normalize that vector
	sphereToPoint = norm_vec3(sphereToPoint);
	// Adjust it's length to point to edge of sphere
	sphereToPoint = scale_vec3(sphereToPoint, sphereRad);
	// Translate into world space
	vec3 worldPoint = add_vec3(spherePos, sphereToPoint);
	// Return new point
	return worldPoint;
}

vec3 getClosestPointToObsticle(int id, vec3 v) {
	Platform o = getPlatform<Platform>(id);
	if (o.tp == box)
		return closestPointOnAABB(v, o);
	else if (o.tp == sphere)
		return closestPointOnSphere(v, o);
	return { 0,0,0 };
}

std::pair<int, float> getClosestPlatform(vec3 pos) {
	float closest = 99999;
	int id = -1;
	for (int i = 0; i < platforms.size(); i++) {
		
		float dis = 0;
		if (platforms[i].tp == box) {
			vec3 p = { platforms[i].w,platforms[i].h,platforms[i].l };	
			dis = udBox({ pos.x - platforms[i].x,pos.y - platforms[i].y,pos.z - platforms[i].z }, p);
		}
		else if (platforms[i].tp == sphere)
			dis = sdSphere({ pos.x - platforms[i].x,pos.y - platforms[i].y,pos.z - platforms[i].z }, platforms[i].r);
		//printf("dis %f,%i", dis, platforms[i].id);
		if (dis < closest) {
			closest = dis;
			id = platforms[i].id;
		}
	}
	return std::make_pair(id, closest);
}
std::pair<int, float> getClosestStaticAnt(vec3 pos, int pId) {
	float closest = 99999;
	int id = -1;
	for (int i = 0; i < particles.size(); i++) {
		if (i == pId)
			continue;
		if (particles[i].isStatic) {
			float dis = 0;
			dis = sdSphere({ pos.x - particles[i].px,pos.y - particles[i].py,pos.z - particles[i].pz }, particles[i].r);
			if (dis < closest) {
				closest = dis;
				id = particles[i].id;
			}
		}
	}
	return std::make_pair(id, closest);
}

std::pair<int, char>  getClosestObsticle(vec3 pos, int id) {
	std::pair<int, float> staticAnt = getClosestStaticAnt(pos, id);
	std::pair<int, float> platform = getClosestPlatform(pos);

	std::pair<int, char> closest;
	if (staticAnt.second < platform.second) {
		closest.first = staticAnt.first;
		closest.second = 'a';
	}
	else {
		closest.first = platform.first;
		closest.second = 'p';
	}
	return closest;

}

//May reuse this function
void clamp_particle_to_point(int i, vec3 point) {
	vec3 oldPosition = { particles[i].px,particles[i].py,particles[i].pz };
	vec3 closestToSphere = getClosestPointToSphere({ particles[i].px,particles[i].py,particles[i].pz }, particles[i].r - .0001, point);
	vec3 newPosition = { point.x,point.y,point.z };
	vec3 diff = sub_vec3(newPosition, oldPosition);
	closestToSphere = add_vec3(closestToSphere, diff);
	vec3 disFromClosestToNew = sub_vec3(newPosition, closestToSphere);
	newPosition = add_vec3(newPosition, disFromClosestToNew);

	//printf("closest point is %f,%f,%f", c.x, c.y, c.z);
	particles[i].px = newPosition.x;
	particles[i].py = newPosition.y;
	particles[i].pz = newPosition.z;
}

//gets closest obsticle and attaches ant to it
void resolveClosestSurface() {
	for (int i = 0; i < particles.size(); i++) {
		if (particles[i].isStatic) {
			int closestId = particles[i].attachedId;
			char closestType = particles[i].attachedType;
			vec3 closestToObsticle;
			if (closestType == 'p') {
				closestToObsticle = getClosestPointToObsticle(closestId, { particles[i].px,particles[i].py,particles[i].pz });
			}
			else if (closestType == 'a') {
				closestToObsticle = getClosestPointToSphere({ particles[closestId].x,particles[closestId].y,particles[closestId].z }, particles[i].r, { particles[i].px, particles[i].py, particles[i].pz });
			}
			clamp_particle_to_point(i, closestToObsticle);
		}
		else {
			std::pair<int, char> closestObst = getClosestObsticle({ particles[i].x,particles[i].y,particles[i].z }, i);
			int closestId = closestObst.first;
			if (closestId == -1) {
				printf("negative id %c. position %f,%f,%f. Id: %i. ", closestObst.second, particles[i].px, particles[i].py, particles[i].pz, i);
				continue;
			}
			vec3 closestToObsticle;
			if (closestObst.second == 'p') {
				closestToObsticle = getClosestPointToObsticle(closestId, { particles[i].px,particles[i].py,particles[i].pz });
			}
			else if (closestObst.second == 'a')
				closestToObsticle = getClosestPointToSphere({ particles[closestId].x,particles[closestId].y,particles[closestId].z }, particles[i].r, { particles[i].px, particles[i].py, particles[i].pz });

			particles[i].attachedId = closestId;
			particles[i].attachedType = closestObst.second;
			clamp_particle_to_point(i, closestToObsticle);
		}
	}
}


void resolve_ant_platform_collision() {
	for (int i = 0; i < particles.size(); i++) {
		for (std::pair<char, std::vector<int>> element : particles[i].obsticleCollisionIds)
		{
			if (element.second.size() > 0) {
				vec3 closestToObsticle = {9999,9999,9999};
				if (element.first == 'p') {
					for (int j = 0; j < element.second.size(); j++) {
						vec3 dist = getClosestPointToObsticle(element.second[j], { particles[i].x,particles[i].y,particles[i].z });
						closestToObsticle = (distance(particles[i].x, particles[i].y, particles[i].z, dist.x, dist.y, dist.z) < distance(particles[i].x, particles[i].y, particles[i].z, closestToObsticle.x, closestToObsticle.y, closestToObsticle.z)) ? dist:closestToObsticle ;
					}
					
				}
				else if (element.first == 'a') {
					for (int j = 0; j < element.second.size(); j++) {
						closestToObsticle = getClosestPointToSphere({ particles[element.second[j]].x,particles[element.second[j]].y,particles[element.second[j]].z }, particles[i].r, { particles[i].x, particles[i].y, particles[i].z });
					}
				}

				clamp_particle_to_point(i, closestToObsticle);
			}
		}
	}
}

float* axis_aligned_cube_collision_constraint(Particle p1,Platform cube) {
	static float output[3];
	//get closest point to cube and subtract from position to get normal
	//normalize result
	//deltas will be the distance from (sphere radius - (sphere center-closest point) in x,y,z)
	//value will be scaled by normal to get distance to translate the sphere
	vec3 closestPoint=closestPointOnAABB({p1.px,p1.py,p1.pz}, cube);
	vec3 norm = sub_vec3({ p1.px,p1.py,p1.pz },closestPoint).normalize();
	float dist = p1.r - distance(p1.px, p1.py, p1.pz, closestPoint.x, closestPoint.y, closestPoint.z);
	norm = scale_vec3(norm, dist);
	output[0] = norm.x;
	output[1] = norm.y;
	output[2] = norm.z;
	return output;
}

void resolve_collisions_with_static_objects() {
	for (int i = 0; i < particles.size(); i++) {
		if (!particles[i].isStatic) {
			continue;
		}
		for (int j = 0; j < platforms.size(); j++) {
			if (platforms[j].tp != box)
				continue;
			if (sphereRect(particles[i], platforms[j])) {
				float* deltas = axis_aligned_cube_collision_constraint(particles[i], platforms[j]);
				particles[i].px += deltas[0];
				particles[i].py += deltas[1];
				particles[i].pz += deltas[2];
				//printf("collision deltas %f,%f,%f", deltas[0], deltas[1], deltas[2]);
			}
		}
	}
}
float* collision_constraint(Particle particle1, Particle particle2) {
	static float corrections[6];
	float correction_x1 = 0.0;
	float correction_y1 = 0.0;
	float correction_z1 = 0.0;
	float correction_x2 = 0.0;
	float correction_y2 = 0.0;
	float correction_z2 = 0.0;

	float px1 = particle1.px;
	float py1 = particle1.py;
	float pz1 = particle1.pz;
	float px2 = particle2.px;
	float py2 = particle2.py;
	float pz2 = particle2.pz;

	float currdist = distance(px1, py1,pz1, px2, py2,pz2);
	if (currdist == 0)
		return corrections;
	float dist_diff = currdist - (particle1.r + particle2.r);

	if (dist_diff < 0) {

		float coef1 = -particle1.inv_mass / (particle1.inv_mass + particle2.inv_mass);
		float coef2 = particle2.inv_mass / (particle1.inv_mass + particle2.inv_mass);
		float coef = .5f*dist_diff / currdist;
		correction_x1 = coef1 * coef * (px1 - px2);
		correction_y1 = coef1 * coef * (py1 - py2);
		correction_z1 = coef1 * coef * (pz1 - pz2);
		correction_x2 = coef2 * coef * (px1 - px2);
		correction_y2 = coef2 * coef * (py1 - py2);
		correction_z2 = coef2 * coef * (pz1 - pz2);
	}

	corrections[0] = correction_x1;
	corrections[1] = correction_y1;
	corrections[2] = correction_z1;
	corrections[3] = correction_x2;
	corrections[4] = correction_y2;
	corrections[5] = correction_z2;

	return corrections;
}

vec3 project_on_vector(vec3 a, vec3 b_normalized) {
	vec3 out;
	float d = dot_vec3(a, b_normalized);
	out.x = b_normalized.x * d;
	out.y = b_normalized.y * d;
	out.z = b_normalized.z * d;

	return out;
}

float* friction_constraint(Particle p1, Particle p2) {
	// we don't want to use the bad old values
	static float corrections[6];
	corrections[0] = 0.0;
	corrections[1] = 0.0;
	corrections[2] = 0.0;
	corrections[3] = 0.0;
	corrections[4] = 0.0;
	corrections[5] = 0.0;

	float friction_constraint_stiffness = .2;

	vec3 out;
	float w_i_coef = p1.inv_mass / (p1.inv_mass + p2.inv_mass);
	float w_j_coef = -p2.inv_mass / (p1.inv_mass + p2.inv_mass);
	vec3 contact_normal;
	vec3 tangential_displacement;
	vec3 x_pred_w_delta1;
	vec3 x_pred_w_delta2;
	float collision_margin = (p1.r + p2.r) * 1.01f;
	float d = dist_vec3({ p1.px,p1.py,p1.pz }, { p2.px,p2.py,p2.pz });
	float f = d - collision_margin;
	if (f < 0) {
		contact_normal.x = 0.;
		contact_normal.y = 0.;
		contact_normal.z = 0.;
		tangential_displacement.x = 0.;
		tangential_displacement.y = 0.;
		tangential_displacement.z = 0.;
		x_pred_w_delta1.x = 0.;
		x_pred_w_delta1.y = 0.;
		x_pred_w_delta1.z = 0.;
		x_pred_w_delta2.x = 0.;
		x_pred_w_delta2.y = 0.;
		x_pred_w_delta2.z = 0.;
		out.x = 0.;
		out.y = 0.;
		out.z = 0.;
		contact_normal.x = (p1.px - p2.px) / d;
		contact_normal.y = (p1.py - p2.py) / d;
		contact_normal.z = (p1.pz - p2.pz) / d;
		corrections[0] = -w_i_coef * contact_normal.x * f;
		corrections[1] = -w_i_coef * contact_normal.y * f;
		corrections[2] = -w_i_coef * contact_normal.z * f;
		corrections[3] = -w_j_coef * contact_normal.x * f;
		corrections[4] = -w_j_coef * contact_normal.y * f;
		corrections[5] = -w_j_coef * contact_normal.z * f;
		x_pred_w_delta1.x = corrections[0] + p1.px;
		x_pred_w_delta1.y = corrections[1] + p1.py;
		x_pred_w_delta1.z = corrections[2] + p1.pz;
		x_pred_w_delta2.x = corrections[3] + p2.px;
		x_pred_w_delta2.y = corrections[4] + p2.py;
		x_pred_w_delta2.z = corrections[5] + p2.pz;
		float n_norm = dist_vec3(x_pred_w_delta1, x_pred_w_delta2);
		contact_normal.y = (x_pred_w_delta1.x - x_pred_w_delta2.x) / n_norm;
		contact_normal.x = -(x_pred_w_delta1.y - x_pred_w_delta2.y) / n_norm;
		contact_normal.z = -(x_pred_w_delta1.z - x_pred_w_delta2.z) / n_norm;
		// tangential_displacement.x = x_pred_w_delta1.x-x_pred_w_delta2.x;
		// tangential_displacement.y = x_pred_w_delta1.y-x_pred_w_delta2.y;
		// Above might be wrong
		// should be
		tangential_displacement.x = x_pred_w_delta1.x - p1.x -
			(x_pred_w_delta2.x - p2.x);
		tangential_displacement.y = x_pred_w_delta1.y - p1.y -
			(x_pred_w_delta2.y - p2.y);
		tangential_displacement.z = x_pred_w_delta1.z - p1.z -
			(x_pred_w_delta2.z - p2.z);
		out=project_on_vector(tangential_displacement, contact_normal);
		float out_norm = length_vec3(out);
		if (out_norm >= mui_static * d) {
			float coef = std::min(1.f, mui_kinematic * d / out_norm);
			out.x *= coef;
			out.y *= coef;
			out.z *= coef;
		}
		corrections[0] += -out.x * w_i_coef;
		corrections[1] += -out.y * w_i_coef;
		corrections[2] += -out.z * w_i_coef;
		corrections[3] += -out.x * w_j_coef;
		corrections[4] += -out.y * w_j_coef;
		corrections[5] += -out.z * w_j_coef;
	}
	//else {
	//	vec3 x_i = { p1.x,p1.y,p1.z };
	//	vec3 x_j = { p2.x,p2.y,p2.z };
	//	const float dist = dist_vec3({ p1.px,p1.py,p1.pz }, { p2.px,p2.py,p2.pz });
	//	float radius_sq = powf(p1.r + p2.r,2);
	//	if (dist < p1.r+p2.r) {
	//		radius_sq = (p1.r + p2.r - dist) * (p1.r + p2.r - dist);
	//	}
	//	const float v_ix = (p1.px - x_i.x) / time_delta;
	//	const float v_jx = (p2.px - x_j.x) / time_delta;
	//	const float v_x = v_ix - v_jx;
	//	const float v_iy = (p1.py - x_i.y) / time_delta;
	//	const float v_jy = (p2.py - x_j.y) / time_delta;
	//	const float v_y = v_iy - v_jy;
	//	const float v_iz = (p1.pz - x_i.z) / time_delta;
	//	const float v_jz = (p2.pz - x_j.z) / time_delta;
	//	const float v_z = v_iz - v_jz;
	//	float x0 = x_i.x - x_j.x;
	//	float y0 = x_i.y - x_j.y;
	//	float z0 = x_i.z - x_j.z;
	//	float v_sq = v_x * v_x + v_y * v_y+v_z*v_z;
	//	float x0_sq = x0 * x0;
	//	float y0_sq = y0 * y0;
	//	float z0_sq = z0 * z0;
	//	float x_sq = x0_sq + y0_sq+z0_sq;
	//	float a = v_sq;
	//	float b = -v_x * x0 - v_y * y0 - v_z * z0;
	//	float b_sq = b * b;
	//	float c = x_sq - radius_sq;
	//	float d_sq = b_sq - a * c;
	//	if (d_sq > 0 && (a < -EPS || a > EPS)) {
	//		float d = sqrtf(d_sq);
	//		float tao = (b - d) / a;
	//		float tao_alt = (b + d) / a;
	//		// pick the min solution that is > 0
	//		tao = tao_alt < tao&& tao_alt > 0 ? tao_alt : tao;
	//		// need to consider +- sign perhaps?
	//		if (tao > 0) {
	//			// const float min_tao_init=b/v_sq;
	//			const float min_tao =
	//				tao + time_delta; // min_tao_init;//(min_tao_init+tao)/2;
	//			const float x_i_min = x_i.x + min_tao * v_ix;
	//			const float y_i_min = x_i.y + min_tao * v_iy;
	//			const float z_i_min = x_i.z + min_tao * v_iz;
	//			const float x_j_min = x_j.x + min_tao * v_jx;
	//			const float y_j_min = x_j.y + min_tao * v_jy;
	//			const float z_j_min = x_j.z + min_tao * v_jz;
	//			float min_tao_dist = sqrtf((x_i_min - x_j_min) * (x_i_min - x_j_min) +
	//				(y_i_min - y_j_min) * (y_i_min - y_j_min) +
	//				(z_i_min - z_j_min) * (z_i_min - z_j_min));
	//			float d = min_tao_dist;
	//			float f = d - collision_margin;
	//			if (f < 0 && d > EPS) {
	//				const float clamp_tao = exp(-min_tao * min_tao / 5.);
	//				const float k = friction_constraint_stiffness; // 0.25;
	//				contact_normal.x = 0.;
	//				contact_normal.y = 0.;
	//				contact_normal.z = 0.;
	//				tangential_displacement.x = 0.;
	//				tangential_displacement.y = 0.;
	//				tangential_displacement.z = 0.;
	//				x_pred_w_delta1.x = 0.;
	//				x_pred_w_delta1.y = 0.;
	//				x_pred_w_delta1.z = 0.;
	//				x_pred_w_delta2.x = 0.;
	//				x_pred_w_delta2.y = 0.;
	//				x_pred_w_delta2.z = 0.;
	//				out.x = 0.;
	//				out.y = 0.;
	//				out.z = 0.;
	//				contact_normal.x = (x_i_min - x_j_min) / d;
	//				contact_normal.y = (y_i_min - y_j_min) / d;
	//				contact_normal.z = (z_i_min - z_j_min) / d;
	//				corrections[0] = -k * clamp_tao * w_i_coef * contact_normal.x * f;
	//				corrections[1] = -k * clamp_tao * w_i_coef * contact_normal.y * f;
	//				corrections[2] = -k * clamp_tao * w_i_coef * contact_normal.z * f;
	//				corrections[3] = -k * clamp_tao * w_j_coef * contact_normal.x * f;
	//				corrections[4] = -k * clamp_tao * w_j_coef * contact_normal.y * f;
	//				corrections[5] = -k * clamp_tao * w_j_coef * contact_normal.z * f;

	//				const float x_i_tao = x_i.x + tao * v_ix;
	//				const float y_i_tao = x_i.y + tao * v_iy;
	//				const float z_i_tao = x_i.z + tao * v_iz;
	//				const float x_j_tao = x_j.x + tao * v_jx;
	//				const float y_j_tao = x_j.y + tao * v_jy;
	//				const float z_j_tao = x_j.z + tao * v_jz;
	//				x_pred_w_delta1.x = corrections[0] + x_i_min;
	//				x_pred_w_delta1.y = corrections[1] + y_i_min;
	//				x_pred_w_delta1.z = corrections[2] + z_i_min;
	//				x_pred_w_delta2.x = corrections[3] + x_j_min;
	//				x_pred_w_delta2.y = corrections[4] + y_j_min;
	//				x_pred_w_delta2.z = corrections[5] + z_j_min;

	//				float n_norm = dist_vec3(x_pred_w_delta1, x_pred_w_delta2);
	//				contact_normal.y = (x_pred_w_delta1.x - x_pred_w_delta2.x) / n_norm;
	//				contact_normal.x =
	//					-(x_pred_w_delta1.y - x_pred_w_delta2.y) / n_norm;
	//				contact_normal.z =
	//					-(x_pred_w_delta1.z - x_pred_w_delta2.z) / n_norm;
	//				tangential_displacement.x =
	//					x_pred_w_delta1.x - x_i_tao - (x_pred_w_delta2.x - x_j_tao);
	//				tangential_displacement.y =
	//					x_pred_w_delta1.y - y_i_tao - (x_pred_w_delta2.y - y_j_tao);
	//				tangential_displacement.z =
	//					x_pred_w_delta1.z - z_i_tao - (x_pred_w_delta2.z - z_j_tao);
	//				out=project_on_vector(tangential_displacement, contact_normal);
	//				float out_norm = length_vec3(out);
	//				if (out_norm >= mui_static * d) {
	//					float coef = std::min(1.f, mui_kinematic * d / out_norm);
	//					out.x *= coef;
	//					out.y *= coef;
	//					out.z *= coef;
	//				}
	//				corrections[0] += -out.x * w_i_coef;
	//				corrections[1] += -out.y * w_i_coef;
	//				corrections[2] += -out.z * w_i_coef;
	//				corrections[3] += -out.x * w_j_coef;
	//				corrections[4] += -out.y * w_j_coef;
	//				corrections[5] += -out.z * w_j_coef;
	//			}
	//		}
	//	}
	//	
	//}
	return corrections;
}
//collision constraint between two active particles
void resolve_collision_constraints() {
	for (int i = 0; i < particles.size(); i++) {
		//if (particles[i].isStatic)
			//continue;
		for (int j = i + 1; j < particles.size(); j++) {
			//if (particles[i].isStatic && particles[j].isStatic) {
				//continue;
			//}
			float* deltas = collision_constraint(particles[i], particles[j]);
			float delta_x1 = deltas[0];
			float delta_y1 = deltas[1];
			float delta_z1 = deltas[2];
			float delta_x2 = deltas[3];
			float delta_y2 = deltas[4];
			float delta_z2 = deltas[5];

			particles[i].px += delta_x1;
			particles[i].py += delta_y1;
			particles[i].pz += delta_z1;
			particles[j].px += delta_x2;
			particles[j].py += delta_y2;
			particles[j].pz += delta_z2;
		}
	}
}

//friction constraint between two active particles
void resolve_friction_constraints() {
	for (int i = 0; i < particles.size(); i++) {
		//if (particles[i].isStatic)
			//continue;
		for (int j = i + 1; j < particles.size(); j++) {
			if (particles[i].isStatic && particles[j].isStatic) {

				float* deltas = friction_constraint(particles[i], particles[j]);
				float delta_x1 = deltas[0];
				float delta_y1 = deltas[1];
				float delta_z1 = deltas[2];
				float delta_x2 = deltas[3];
				float delta_y2 = deltas[4];
				float delta_z2 = deltas[5];

				particles[i].px += delta_x1;
				particles[i].py += delta_y1;
				particles[i].pz += delta_z1;
				particles[j].px += delta_x2;
				particles[j].py += delta_y2;
				particles[j].pz += delta_z2;
			}
		}
	}
}
void write_to_file() {
	FILE* fp_out;
	std::string path = std::string(OUT_PATH) + std::to_string(frame) + std::string(".txt");
	fp_out = fopen(path.c_str(), "w");
	if (fp_out != NULL) {
		for (int i = 0; i < num_of_particles; i++) {
			if(i!=num_of_particles-1)
				fprintf(fp_out, "%.5f,%.5f,%.5f,", particles[i].x,particles[i].y, particles[i].z);
			else
				fprintf(fp_out, "%.5f,%.5f,%.5f", particles[i].x, particles[i].y, particles[i].z);
		}
	}
	fclose(fp_out);
}

void createSetUpFile(int sim) {
	FILE* fp_out;
	std::string path =std::string("setup")+std::to_string(sim)+std::string(".txt");
	fp_out = fopen(path.c_str(), "w");
	if (fp_out != NULL) {
		fprintf(fp_out, "Number of Particles:%i\n",num_of_particles);
		fprintf(fp_out, "Number of Platforms:%i\n", platforms.size());
		for (int i = 0; i < platforms.size(); i++) {
			if(platforms[i].tp==sphere)
				fprintf(fp_out, "sphere,%.5f,%.5f,%.5f,%.5f\n",platforms[i].r,platforms[i].x, platforms[i].y, platforms[i].z);
			else
				fprintf(fp_out, "box,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", platforms[i].l, platforms[i].w, platforms[i].h, platforms[i].x, platforms[i].y, platforms[i].z);
		}
	}
	fclose(fp_out);
}

void create_setup() {

}
void pbd_main_loop(int a)
{
	
	//checks for static obsticles 
	check_particle_platform_collision();
	check_particle_particle_collision();
	//update particles
	update_particles();
	for (int i = 0; i < particles.size(); i++) {
		// get initial projected positions + apply external forces - line 7
		particles[i].px = particles[i].x + particles[i].vx;
		particles[i].py = particles[i].y + particles[i].vy + gravity * time_delta;
		particles[i].pz = particles[i].z + particles[i].vz;
	}
	
	
	
	//for (int i = 0; i < 3; i++) {
	//resolve_friction_constraint(); friction constraint resolved before/after collision constraint
	//resolve_friction_constraints();
	resolve_collision_constraints();
	//resolve_friction_constraints();
	resolveClosestSurface();
	//}
	
	resolve_collisions_with_static_objects();
	//resolve_ant_platform_collision();
	for (int i = 0; i < particles.size(); i++) {
		// line 13
		//particles[i].vx = (particles[i].px - particles[i].x) / time_delta;
		//particles[i].vy = (particles[i].py - particles[i].y) / time_delta;
		// line 14
		if (!particles[i].isStatic) {
			particles[i].x = particles[i].px;
			particles[i].y = particles[i].py;
			particles[i].z = particles[i].pz;
		}

	}
	//write_to_file();
	frame++;
	//printf("frame %i \n", frame);
	glutPostRedisplay();
	glutTimerFunc(2, pbd_main_loop, 25);//Call update after each 25 millisecond
}

vec3 spawnRandomCircle(vec3 center, float radius,int dir) {
	//0 for xz,1 for yx,2 for zy
	float ang = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 360;
	//printf("angle %f", ang);
	vec3 pos;
	if (dir == 0) {
		pos.x = center.x + radius * sinf(ang * (pi * 2) / 360);
		pos.z = center.z + radius * cosf(ang * (pi * 2) / 360);
		pos.y = center.y;
	}
	if (dir == 1) {
		pos.x = center.x + radius * sinf(ang * (pi * 2) / 360);
		pos.y = center.y + radius * cosf(ang * (pi * 2) / 360);
		pos.z = center.z;
	}
	if (dir == 2) {
		pos.z = center.z + radius * sinf(ang * (pi * 2) / 360);
		pos.y = center.y + radius * cosf(ang * (pi * 2) / 360);
		pos.x = center.x;
	}

	return pos;
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {

	//while (reachedGoal==false||frame < 70000) {
	//	pbd_main_loop(0);
	//}
	int i;
	std::cout << "enter simulation. 0-horizantal bridge,1-verticle bridge" ;
	std::cin >> i;
	printf("int %i", i);
	switch (i) {
	case 0: {
		platforms.push_back(Platform(0.0, 0.0, 0.0, box, .2, .9, .2, 0));
		platforms.push_back(Platform(0.0, -0.3, 0.0, box, 2, .2, 2, 1));
		platforms.push_back(Platform(0.0, 2.0, 0.0, box, .2, .9, .2, 2));
		platforms.push_back(Platform(0.0, 2.0, 0.0, box, 2, .2, 2, 3));

		setCameraPosition(0, 1, 5);
		//spawn 50 below
		int j = 0;
		for (j; j < 50; j++) {
			vec3 pos = spawnRandomCircle({ 0,0,0 }, 1,0);
			particles.push_back(Particle(pos.x, pos.y, pos.z, j));
			particles[j].setDestination({ 0, 2,0 });
		}
		//spawn 50 above
		for (j; j < 100; j++) {
			vec3 pos = spawnRandomCircle({ 0,1.5,0 }, 1,0);
			particles.push_back(Particle(pos.x, pos.y, pos.z, j));
			particles[j].setDestination({ 0, 0,0 });
		}
		createSetUpFile(i);
	}
		break;
	case 1:{
		platforms.push_back(Platform(-1.0, 1.0, 0.0, box, .9, .2, .2, 0));
		platforms.push_back(Platform(-2.0, 1.0, 0.0, box, .2, 2, 2, 1));
		platforms.push_back(Platform(1.0, 1.0, 0.0, box, .9, .2, .2, 2));
		platforms.push_back(Platform(2.0, 1.0, 0.0, box, .2, 2, 2, 3));

		setCameraPosition(0, 1, 5);
		//spawn 50 below
		int j = 0;
		for (j; j < 50; j++) {
			vec3 pos = spawnRandomCircle({ -1,1,0 }, 1,2);
			particles.push_back(Particle(pos.x, pos.y, pos.z, j));
			particles[j].setDestination({ 3, 2,0 });
		}
		//spawn 50 above
		for (j; j < 100; j++) {
			vec3 pos = spawnRandomCircle({ 1,1,0 }, 1,2);
			particles.push_back(Particle(pos.x, pos.y, pos.z, j));
			particles[j].setDestination({ -3, 2,0 });
		}
		createSetUpFile(i); 
	}
		break;
	case 2:{}
		break;
	}
	glutInit(&argc, argv);            // Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE); // Enable doub4le buffered mode
	glutInitWindowSize(SCR_WIDTH, SCR_HEIGHT);   // Set the window's initial width & height
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	glutCreateWindow(title);          // Create window with the given title
	//ourShader = &Shader("shader.vs", "shader.fs");
	glutDisplayFunc(display);       // Register callback handler for window re-paint event
	glutReshapeFunc(reshape);       // Register callback handler for window re-size event
	//glutMouseFunc(mouse);
	glutPassiveMotionFunc(mousePassive);
	glutSpecialFunc(processSpecialKeys);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(2, pbd_main_loop, 0);
	initGL();                       // Our own OpenGL initialization
	//glutFullScreen();
	glutMainLoop();                 // Enter the infinite event-processing loop
	glDeleteTextures(1, &TexID1);
	glDeleteTextures(1, &TexID2);
	return 0;
}

