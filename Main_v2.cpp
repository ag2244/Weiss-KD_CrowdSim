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

//in simulation
float particle_radii = .02;
float time_delta = 1 / 64.;
int max_particles = 200;
float particle_distance = particle_radii * 2.5;
int num_of_particles = 10;

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
	//float gravity = -.2;
	float r = particle_radii;
	float inv_mass = -1;
	float speed = .02;
	int timer = 0;
	vec3 destination = { 0,0,0 };
	vec3 up = { 0,1,0 };
	vec3 right = { 0,0,1 };
	vec3 forward = { 1,0,0 };
	glm::quat rotation = glm::quat(1, 0, 0, 0);
	vec3 projforward = { 0,0,0 };
	PathFindingMode pfm = PathFindingMode::simple;
	bool foundDest = false;
	bool collision = false;
	int obsticleCollisionId = -1;
	char collWith;//p for plat, a for ant
	std::unordered_map<char, std::vector<int>> obsticleCollisionIds;
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
		if (timer == 0) {
			timerPosition = { x,y,z };
		}
		else if (timer > 200) {
			if (isStatic == false) {
				if (distance(x, y, z, timerPosition.x, timerPosition.y, timerPosition.z) < r * 2) {
					isStatic = true;
				}
			}
			timer = -1;
		}

		if (isStatic == false) {
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
			vx = 0;
			vy = 0;
			vz = 0;
		}
		update_rotations();
		timer++;
	};


	void calculateMovement() {
		vx = 0;
		vy = 0;//gravity *speed* .99 * time_delta; maybe turn gravity on when on bridge
		vz = 0;
		if (x > destination.x)
			vx = -1 * speed * .99 * time_delta;
		else if (x < destination.x)
			vx = 1 * speed * .99 * time_delta;
		if (y > destination.y)
			vy = -.02 * .99* time_delta;
		else if (y < destination.y)
			vy = .02 * .99* time_delta;
		if (z > destination.z)
			vz = -1 * speed * .99 * time_delta;
		else if (z < destination.z)
			vz = 1 * speed * .99 * time_delta;
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
				}
				else if (collWith == 'a') {
					//printf("checking a collision");
					contactPoint = getClosestPointToSphere(destination, r, { x,y,z });
				}
			}
			dir = Direction::up;
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
			vec3 rotated = qRotate(dir, .01, glm::vec3(up.x, up.y, up.z));
			//printf("right rotation %f,%f,%f", rotated.x, rotated.y, rotated.z);
			float tempvx = rotated.x;
			float tempvy = rotated.y;
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
				vy = tempvy;
				vz = tempvz;
				//printf("pathfound ");
			}

			if (freeSpaceLeft == false) {
				//check forward
				predx = x + vx;
				predy = y + vy;
				predy = z + vz;
				bool canMoveForward = false;
				//printf("no free space");
				if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == true) {
					//printf("no pass");
					bool pathfound = false;
					if (pathfound == false) {
						//rotate it right
						vec3 rotated = qRotate(glm::vec3(vx, vy, vz), .01, glm::vec3(up.x, up.y, up.z));
						vx = rotated.x;
						vy = rotated.y;
						vz = rotated.z;
						predx = x + vx;
						predy = y + vy;
						predz = y + vz;
						if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
							//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
							pathfound = true;
							//printf("forward pathfound");
							canMoveForward = true;
						}
						else {
							//printf("rotating id:%i", id);
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
			float tempvy = rotated.y;
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
				predy = z + vz;
				bool canMoveForward = false;
				//printf("no free space");
				if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == true) {
					//printf("no pass");
					bool pathfound = false;
					if (pathfound == false) {
						//rotate it left
						vec3 rotated = qRotate(glm::vec3(vx, vy, vz), .01, glm::vec3(up.x, up.y, up.z));
						vx = rotated.x;
						vy = rotated.y;
						vz = rotated.z;
						predx = x + vx;
						predy = y + vy;
						predz = y + vz;
						if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
							//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
							pathfound = true;
							//printf("forward pathfound");
							canMoveForward = true;
						}
						else {
							//printf("rotating id:%i", id);
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
			vec3 rotated = qRotate(dir, -.01, glm::vec3(right.x, right.y, right.z));
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
				//predx = x + vx;
				//predy = y + vy;
				//predy = z + vz;
				bool canMoveForward = false;
				//printf("no free space");
				if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == true) {
					//printf("no pass");
					bool pathfound = false;
					if (pathfound == false) {
						//rotate it left
						float angle = .01; //angle in radians
						vec3 rotated = qRotate(glm::vec3(vx, vy, vz), angle, glm::vec3(right.x, right.y, right.z));
						vx = rotated.x;
						vy = rotated.y;
						vz = rotated.z;
						predx = x + vx;
						predy = y + vy;
						predz = y + vz;
						if (particleObsticle({ predx,predy,predz }, obsticleCollisionId, collWith) == false) {
							//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
							pathfound = true;
							//printf("forward pathfound");
							canMoveForward = true;
						}
						else {
							//printf("rotating id:%i", id);
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
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position2);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	//Image* image1 = loadTexture("wood.bmp");
	//Image* image2 = loadTexture("brick.bmp");
	//if (image1 == NULL) {

	//	printf("Image was not returned from loadTexture\n");

	//	exit(0);

	//}

	//makeCheckImage();

	//glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	//// Create Texture

	//glGenTextures(2, texture);

	//glBindTexture(GL_TEXTURE_2D, texture[0]);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); //scale linearly when image bigger than texture

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); //scale linearly when image smalled than texture

	//glTexImage2D(GL_TEXTURE_2D, 0, 3, image1->sizeX, image1->sizeY, 0,

	//	GL_RGB, GL_UNSIGNED_BYTE, image1->data);

	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

	//glBindTexture(GL_TEXTURE_2D, texture[1]);

	//glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);

	//glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	//glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);

	//glTexImage2D(GL_TEXTURE_2D, 0, 3, image2->sizeX, image2->sizeY, 0,

	//	GL_RGB, GL_UNSIGNED_BYTE, image2->data);

	//glEnable(GL_TEXTURE_2D);

	//glShadeModel(GL_FLAT);
	//glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}

//draw cube
void drawCube(float x, float y, float z, float w, float h, float l) {
	glDisable(GL_LIGHTING);
	glPushMatrix();
	//ourShader->setVec3("transPosition", glm::vec3(x, y, z));
	//ourShader->setInt("texture1", texture[0]);
	glTranslatef(x,y,z);  // Move right and into the screen
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glEnable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D, texture[1]);
	//glActiveTexture(GL_TEXTURE0);
	

	//int texcoord_index = glGetAttribLocation(ourShader->ID, "textureCoords");

	glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads

	   // Top face (y = 1.0f)
	   // Define vertices in counter-clockwise (CCW) order with normal pointing out
	//glColor3f(0.0f, 0.0f, 1.0f);      // Green
	//glTexCoord2f(1, 1);
	//ourShader->setVec2("aTexCoords", glm::vec2(1, 1));
	//glVertexAttrib2f(texcoord_index, 1, 1);
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
	glColor3f(0.0f, 0.0f, 1.0f);      // Orange
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
	glBindTexture(GL_TEXTURE_2D, 0);
	glEnable(GL_LIGHTING);
}


void drawSphere(float x, float y, float z, float r) {
	glColor3f(1.0f, 0.0f, 0.0f);
	GLUquadric* quad = gluNewQuadric();
	//gluQuadricDrawStyle(quad, GLU_FILL);
	//gluQuadricOrientation(quad, GLU_OUTSIDE);
	//gluQuadricNormals(quad, GLU_SMOOTH);
	//gluQuadricTexture(quad, GL_TRUE);
	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glPushMatrix();
	//ourShader->setVec3("transPosition", glm::vec3(x, y, z));
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	//glActiveTexture(GL_TEXTURE0);
	glEnable(GL_TEXTURE_2D);

	//int texcoord_index = glGetAttribLocation(ourShader->ID, "textureCoords");

	glTranslatef(x, y,z);
	
	//glVertexAttrib2f(texcoord_index, x, y);
	gluSphere(quad, r, 10, 10);
	glPopMatrix();
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
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
	for (int i = 0; i < particles.size(); i++) {
		drawSphere(particles[i].x, particles[i].y, particles[i].z, particles[i].r);
		//if (particles[i].startObjectTrace == true)
			//drawSphere(particles[0].contactPoint.x, particles[0].contactPoint.y, particles[0].contactPoint.z, particles[0].r);
		if (debug == true) {
			//drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x+ particles[i].vx*1000, particles[i].y+ particles[i].vy*1000, particles[i].z+ particles[i].vz*1000);
			drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x + particles[i].forward.x * 2, particles[i].y + particles[i].forward.y * 2, particles[i].z + particles[i].forward.z * 2);
			drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x + particles[i].right.x * 2, particles[i].y + particles[i].right.y * 2, particles[i].z + particles[i].right.z * 2);
			drawLine(particles[i].x, particles[i].y, particles[i].z, particles[i].x + particles[i].up.x * 100, particles[i].y + particles[i].up.y * 100, particles[i].z + particles[i].up.z * 100);
			glDisable(GL_LIGHTING);
			drawSphere(particles[i].destination.x, particles[i].destination.y, particles[i].destination.z, particles[i].r);
			drawSphere(particles[i].contactPoint.x, particles[i].contactPoint.y, particles[i].contactPoint.z, particles[i].r);
			glEnable(GL_LIGHTING);
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
		return(false);
	}

	return(true);
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
			if (j == i || particles[j].isStatic == false)
				continue;
			coll = sphereSphere(particles[i], particles[j]);
			if (coll) {
				particles[i].addToObsticleList(j, 'a');
				//if (collctr == 0) {
				particles[i].handleObsticle(j, 'a');
				//}
				
				//printf("collided with %i",j);
				collctr++;
				//break;
			}
		}
	}
}

//float sdBox(vec3 p, vec3 b)
//{
	//vec3 q = sub_vec3(abs_vec3(p), b);
	//printf("sub %f,%f,%f", q.x, q.y, q.z);


	//return length_vec3({ std::max(q.x,0.0f),std::max(q.y,0.0f),std::max(q.z,0.0f) }) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
//}

float udBox(vec3 p, vec3 b)
{
	//printf("bounds %f,%f,%f ", b.x, b.y, b.z);
	vec3 absp = abs_vec3(p);
	vec3 value = sub_vec3(absp, b);
	//printf("point abs %f,%f,%f ", value.x, value.y, value.z);


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
	float closest = 9999;
	int id = -1;
	for (int i = 0; i < platforms.size(); i++) {
		vec3 p = { platforms[i].w,platforms[i].h,platforms[i].l };
		float dis = 0;
		if (platforms[i].tp == box)
			dis = udBox({ pos.x - platforms[i].x,pos.y - platforms[i].y,pos.z - platforms[i].z }, p);
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
	float closest = 9999;
	int id = -1;
	for (int i = 0; i < particles.size(); i++) {
		if (i == pId)
			continue;
		if (particles[i].isStatic) {
			float dis = 0;
			dis = sdSphere({ pos.x - particles[i].x,pos.y - particles[i].y,pos.z - particles[i].z }, particles[i].r);
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

void resolveClosestSurface() {
	for (int i = 0; i < particles.size(); i++) {
		std::pair<int, char> closestObst = getClosestObsticle({ particles[i].px,particles[i].py,particles[i].pz }, i);
		int closestId = closestObst.first;
		//get closest point to object
		vec3 oldPosition = { particles[i].px,particles[i].py,particles[i].pz };
		vec3 closestToObsticle;
		if (closestObst.second == 'p')
			closestToObsticle = getClosestPointToObsticle(closestId, { particles[i].px,particles[i].py,particles[i].pz });
		else if (closestObst.second == 'a')
			closestToObsticle = getClosestPointToSphere({ particles[closestId].x,particles[closestId].y,particles[closestId].z }, particles[i].r, { particles[i].px, particles[i].py, particles[i].pz });

		vec3 closestToSphere = getClosestPointToSphere({ particles[i].px,particles[i].py,particles[i].pz }, particles[i].r-.0001, closestToObsticle);
		vec3 newPosition = { closestToObsticle.x,closestToObsticle.y,closestToObsticle.z };
		vec3 diff = sub_vec3(newPosition, oldPosition);
		closestToSphere = add_vec3(closestToSphere, diff);
		vec3 disFromClosestToNew = sub_vec3(newPosition, closestToSphere);
		newPosition = add_vec3(newPosition, disFromClosestToNew);

		//printf("closest point is %f,%f,%f", c.x, c.y, c.z);
		particles[i].px = newPosition.x;
		particles[i].py = newPosition.y;
		particles[i].pz = newPosition.z;
		//clamp to closest point
	}
}

void pbd_main_loop(int a)
{
	float gravity = 0.0;
	check_particle_platform_collision();
	check_particle_particle_collision();
	update_particles();
	for (int i = 0; i < particles.size(); i++) {
		// apply external forces - line 5
		particles[i].vx += 0.0;
		particles[i].vy += gravity * time_delta;
		particles[i].vz += 0.0;
		// get initial projected positions - line 7
		particles[i].px = particles[i].x + particles[i].vx;
		particles[i].py = particles[i].y + particles[i].vy;
		particles[i].pz = particles[i].z + particles[i].vz;
	}
	//line 9
	//int i = 1;
	//while (i < 4) {
	//	for (int j = 0; j < point_constraints.size(); j++) {
	//		float* pc = point_constraint(particles[point_constraints[j].id1], point_constraints[j].x, point_constraints[j].y);
	//		particles[point_constraints[j].id1].px += 1.0*pc[0];
	//		particles[point_constraints[j].id1].py +=1.0* pc[1];
	//	}
	//	for (int j = 0; j < distance_constraints.size(); j++) {
	//		float stiffness = 1 - pow((1 - distance_constraints[j].stiffness), (1 / i));
	//		float* dist = distance_constraint(particles[distance_constraints[j].id1], particles[distance_constraints[j].id2], distance_constraints[j].distance);
	//		float delta_x1 = dist[0];
	//		float delta_y1 = dist[1];
	//		float delta_x2 = dist[2];
	//		float delta_y2 = dist[3];
	//		particles[distance_constraints[j].id1].px += stiffness * delta_x1;
	//		particles[distance_constraints[j].id1].py += stiffness * delta_y1;
	//		particles[distance_constraints[j].id2].px += stiffness * delta_x2;
	//		particles[distance_constraints[j].id2].py += stiffness * delta_y2;
	//	}
	//	i++;
	//	
	//}
	//resolve_collision_constraints();
	resolveClosestSurface();
	for (int i = 0; i < particles.size(); i++) {
		// line 13
		//particles[i].vx = (particles[i].px - particles[i].x) / time_delta;
		//particles[i].vy = (particles[i].py - particles[i].y) / time_delta;
		// line 14
		particles[i].x = particles[i].px;
		particles[i].y = particles[i].py;
		particles[i].z = particles[i].pz;

	}
	glutPostRedisplay();
	glutTimerFunc(2, pbd_main_loop, 25);//Call update after each 25 millisecond
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv) {
	float x = 0, z = 0;
	for (int i = 0; i < 300; i++) {
		particles.push_back(Particle(x, 0, z, i));
		particles[i].setDestination({ -3.0, 0.0, -3.0 });
		x =((float)rand() / (RAND_MAX)) + 1;
		z =((float)rand() / (RAND_MAX)) + 1;
	}
	//particles[0].isStatic = true;
	platforms.push_back(Platform(-2.0, 0.0, -2.0, box, .9, .5, .5, 0));
	platforms.push_back(Platform(0.0, -0.3, 0.0, box, 2, .2, 2, 1));
	platforms.push_back(Platform(-0.5, 0.0, -2.0, box, .9, .5, .5, 2));
	platforms.push_back(Platform(-1.5, 0.0, -2.0, sphere, .9, 3));
	platforms.push_back(Platform(-2.9, .50, -2.0, sphere, .3, 4));
	//platforms.push_back(Platform(-1.0, -.05, -1.0, box, .09, .06, .06, 5));
	//printf("coll %i", pointRect({ -.5,0,-.5 }, platforms[0]));
	//printf("closest id:%i", getClosestObsticle({ -2.500000, -0.100000, -2.900000, }));

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

