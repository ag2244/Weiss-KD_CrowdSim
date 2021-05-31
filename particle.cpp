#include "main.h";
#include <unordered_map>
//particle

float particle_radii = .3;
Particle::Particle() {
		x = 0;
		y = 0;
		z = 0;
		id = -1;
		vx = 0.0;
		vy = 0.0;
		vz = 0;
		px = x;
		py = y;
		pz = 0;
		r = particle_radii;
		inv_mass = 1.0;
	};
Particle::Particle(float s_x, float s_y, float s_z, int s_id) {
		x = s_x;
		y = s_y;
		z = s_z;
		id = s_id;
		vx = 0.0;
		vy = 0.0;
		px = x;
		py = y;
		pz = 0;
		r = particle_radii;
		inv_mass = 1.0;
		destination.x = 0.0;
		destination.y = 0.0;
	}

void Particle::update() {
		//check if im in object tracing mode
		//if i can make a straight uninterupted line to the goal switch back to simple movements
		if (pfm == objectTracing) {
			bool ObsticleInPath = checkLineOfSight();

			if (ObsticleInPath == true)
				calculatePredictedMovement();
			else {
				pfm = simple;
				//reset stuff here
				startObjectTrace = false;
			}
		}
		else {
			calculateMovement();
		}
	};


void Particle::calculateMovement() {
		if (x > destination.x)
			vx = -1 * .99 * time_delta;
		else if (x < destination.x)
			vx = 1 * .99 * time_delta;
		if (y > destination.y)
			vy = -1 * .99 * time_delta;
		else if (y < destination.y)
			vy = 1 * .99 * time_delta;

	}
void Particle::calculatePredictedMovement() {

		if (startObjectTrace == false) {
			float predx = 0;
			float predy = 0;
			calculateMovement();
			//printf("should only see this once");
			startObjectTrace = true;
			//find the contact point
			//set contact point as dest
			predx = x + vx;
			predy = y + vy;
			if (obsticleCollisionId != -1) {
				Platform plat = getPlatform(obsticleCollisionId);
				float* point = getContactPoint(x, y, destination.x, destination.y, plat.x, plat.y, plat.width, plat.height);
				contactPoint.x = point[0];
				contactPoint.y = point[1];
				//printf(" contact point at %f,%f at obsticle %i", contactPoint.x, contactPoint.y,obsticleCollisionId);
			}

			//straighten vector for object tracing in a direction left/right
			bool straightenDirection = false;
			while (straightenDirection == false) {
				//rotate it right
				vec3 rotated = rotate_2d_vector(vx, vy, -.01);
				vx = rotated.x;
				vy = rotated.y;
				predx = x + vx;
				predy = y + vy;
				if (pointObsticle(predx, predy, obsticleCollisionId) == false) {
					//printf("straight");
					straightenDirection = true;
					//printf("pathfound");

				}
			}
		}
		if (distance(x, y, contactPoint.x, contactPoint.y) <= r) {
			pfm = simple;
			printf("founddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
			startObjectTrace = false;
			return;
		}

		//vx = 0;
		//vy = 0;

		//rotate either left/right chose left
		float predx;
		float predy;

		//check left
		bool freeSpaceLeft = false;
		//rotate it right
		vec3 rotated = rotate_2d_vector(vx, vy, .01);
		float tempvx = rotated.x;
		float tempvy = rotated.y;
		predx = x + tempvx;
		predy = y + tempvy;
		if (pointObsticle(predx, predy, obsticleCollisionId) == false) {
			//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, tempvx, tempvy, predx, predy);
			freeSpaceLeft = true;
		}
		if (freeSpaceLeft == true) {
			//change velocity
			vx = tempvx;
			vy = tempvy;
			//printf("pathfound");
		}

		if (freeSpaceLeft == false) {
			//check forward
			predx = x + vx;
			predy = y + vy;
			bool canMoveForward = false;
			if (pointObsticle(predx, predy, obsticleCollisionId) == true) {
				//printf("no pass");
				bool pathfound = false;
				while (pathfound == false) {
					//rotate it right
					vec3 rotated = rotate_2d_vector(vx, vy, -.01);
					vx = rotated.x;
					vy = rotated.y;
					predx = x + vx;
					predy = y + vy;
					if (pointObsticle(predx, predy, obsticleCollisionId) == false) {
						//printf("current pos %f,%f, velocity %f,%f ,predicted pos %f,%f", x, y, vx, vy, predx, predy);
						pathfound = true;
						printf("pathfound");
						canMoveForward = true;
					}
					else {
						printf("rotating id:%i", id);
					}
				}
			}
		}

	}

void Particle::handleObsticle(int id) {
		//collision = true;
		if (pfm != objectTracing)
			pfm = objectTracing;
		obsticleCollisionId = id;
		obsticleCollisionIds[id] = 1;
	}

bool Particle::checkLineOfSight() {
		bool contact = false;
		for (std::pair<int, int> element : obsticleCollisionIds)
		{
			Platform plat = getPlatform(element.first);
			bool contact = lineRect(x, y, destination.x, destination.y, plat.x, plat.y, plat.width, plat.height);
			if (contact == true)
				return contact;
		}
	}

void Particle::clearObsticleList() {
		for (std::pair<int, int> element : obsticleCollisionIds)
		{
			obsticleCollisionIds[element.first] = 0;
		}
	}

void Particle::setMovementType(PathFindingMode m) {
		pfm = m;
	}

void Particle::setDestination(vec3 dest) {
		destination = dest;
		foundDest = false;
	};
