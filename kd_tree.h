#pragma once

#ifndef _KD_TREE_H_
#define _KD_TREE_H_

#include "main.h"


/******************
* KD TREE ADDITIONS
******************/

bool debugKD = false;

float* perform_collision(Particle* particle1, Particle* particle2)
{
	static float corrections[6];
	float correction_x1 = 0.0;
	float correction_y1 = 0.0;
	float correction_z1 = 0.0;
	float correction_x2 = 0.0;
	float correction_y2 = 0.0;
	float correction_z2 = 0.0;

	float px1 = particle1->px;
	float py1 = particle1->py;
	float pz1 = particle1->pz;
	float px2 = particle2->px;
	float py2 = particle2->py;
	float pz2 = particle2->pz;

	float currdist = distance(px1, py1, pz1, px2, py2, pz2);
	if (currdist == 0)
		return corrections;
	float dist_diff = currdist - (particle1->r + particle2->r);

	if (dist_diff < 0) {

		float coef1 = -particle1->inv_mass / (particle1->inv_mass + particle2->inv_mass);
		float coef2 = particle2->inv_mass / (particle1->inv_mass + particle2->inv_mass);
		float coef = .5f * dist_diff / currdist;
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

class KD_Tree
{

private:

	//Node for a particle
	class KD_Node
	{
	public:

		KD_Node* left;
		KD_Node* right;

		Particle* particle;

		KD_Node(Particle* newparticle) : left(NULL), right(NULL), particle(newparticle) {}

		void del_node(KD_Node* node)
		{
			if (node != NULL) {
				delete node->left;
				delete node->right;
				delete node;
			}
		}

		void insert(Particle* newparticle, int k)
		{

			if (k > 2) { k = 0; }

			switch (k)
			{
				//Dealing with x dimension
			case 0:
				if (newparticle->x < particle->x)
				{
					if (left != NULL)
					{
						left->insert(newparticle, k + 1);
					}

					else { left = new KD_Node(newparticle); }
				}

				else
				{
					if (right != NULL)
					{
						right->insert(newparticle, k + 1);
					}

					else { right = new KD_Node(newparticle); }
				}

				break;

				//Dealing with y dimension
			case 1:

				if (newparticle->y < particle->y)
				{
					if (left != NULL)
					{
						left->insert(newparticle, k + 1);
					}

					else { left = new KD_Node(newparticle); }
				}

				else
				{
					if (right != NULL)
					{
						right->insert(newparticle, k + 1);
					}

					else { right = new KD_Node(newparticle); }
				}

				break;

				//Dealing with z dimension
			case 2:

				if (newparticle->z < particle->z)
				{
					if (left != NULL)
					{
						left->insert(newparticle, k + 1);
					}

					else { left = new KD_Node(newparticle); }
				}

				else
				{
					if (right != NULL)
					{
						right->insert(newparticle, k + 1);
					}

					else { right = new KD_Node(newparticle); }
				}

				break;

			}

		}

		void traverse(Particle* inparticle, int k)
		{
			//First: Collide with the particle represented in this node.

			if (debugKD) printf("\n In function Node::Traverse \n");

			if (particle->id != inparticle->id)
			{

				if (debugKD) printf("\n Performing collision... \n");

				float* deltas = perform_collision(inparticle, particle);
				float delta_x1 = deltas[0];
				float delta_y1 = deltas[1];
				float delta_z1 = deltas[2];
				float delta_x2 = deltas[3];
				float delta_y2 = deltas[4];
				float delta_z2 = deltas[5];

				inparticle->px += delta_x1;
				inparticle->py += delta_y1;
				inparticle->pz += delta_z1;
				particle->px += delta_x2;
				particle->py += delta_y2;
				particle->pz += delta_z2;

				if (debugKD) printf("\n Performed collision! \n");
			}

			//Traverse down the tree

			if (k > 2) { k = 0; }
			//std::cout << k << std::endl;

			//If only left or right exists, traverse there
			if ((left) && (!right))
			{

				if (debugKD) printf("\n Only left exists \n");

				left->traverse(inparticle, k + 1);

				return;
			}

			else if ((right) && (!left))
			{

				if (debugKD) printf("\n Only right exists \n");

				right->traverse(inparticle, k + 1);

				return;
			}

			//If neither exist, return
			else if ((!right) && (!left))
			{

				if (debugKD) printf("\n No left or right \n");

				return;
			}

			if (debugKD) printf("\n Made sure if both left and right exist \n");

			//Get the k values of the particle being compared, the left node's particle, the right node's particle, and the current node's particle to compare
			float particleK; float leftK; float rightK;
			float thisK;

			switch (k)
			{

				//1st dimension (x)
			case 0:

				if (debugKD) printf("\n K = X \n");

				particleK = inparticle->x;
				leftK = left->particle->x;
				rightK = right->particle->x;

				thisK = particle->x;

				break;

				//2nd dimension (y)
			case 1:

				if (debugKD) printf("\n K = Y \n");

				particleK = inparticle->y;
				leftK = left->particle->y;
				rightK = right->particle->y;

				thisK = particle->y;

				break;

				//3rd dimension (z)
			case 2:

				if (debugKD) printf("\n K = Z \n");

				particleK = inparticle->z;
				leftK = left->particle->z;
				rightK = right->particle->z;

				thisK = particle->z;

				break;

			default: return;

			}

			//If particleK is closer to leftK than rightK
			if (abs(particleK - leftK) < abs(particleK - rightK))
			{

				if (debugKD) printf("\n K closer to leftK than rightK \n");

				//Traverse left
				left->traverse(inparticle, k + 1);

				//If particle is closer to the rightward division than to the left particle, traverse right too
				if (abs(particleK - thisK) <= distance(inparticle->x, inparticle->y, inparticle->z, left->particle->x, left->particle->y, left->particle->z))
				{

					if (debugKD) printf("\n Closer to other division than left particle\n");

					//Traverse right
					right->traverse(inparticle, k + 1);
				}
			}

			//If particleK is closer to rightK than leftK
			else
			{

				if (debugKD) printf("\n K closer to rightK than leftK\n");

				//Traverse right
				right->traverse(inparticle, k + 1);

				//If particle is closer to the leftward division than to the right particle, traverse left too
				if (abs(particleK - thisK) <= distance(inparticle->x, inparticle->y, inparticle->z, right->particle->x, right->particle->y, right->particle->z))
				{

					if (debugKD) printf("\n Closer to other division than right particle\n");

					left->traverse(inparticle, k + 1);
				}
			}

		}
	};

	KD_Node* root;

public:

	KD_Tree() : root(NULL) {}

	explicit KD_Tree(KD_Node* newroot) : root(newroot) {}

	~KD_Tree() { delete root; }

	void insert(Particle* newparticle)
	{

		if (!root)
		{
			root = new KD_Node(newparticle);

			return;
		}

		root->insert(newparticle, 0);
	}

	void buildParticleTree(std::vector<Particle> newparticles)
	{
		for (int i = 0; i < newparticles.size(); i++)
		{
			insert(&newparticles[i]);
		}
	}

	void traverse(Particle* particle)
	{

		if (debugKD) printf("\n BEGINNING TRAVERSAL\n");

		if (!root) { return; }

		root->traverse(particle, 0);
	}

};

/*void keyboard(unsigned char key, int x, int y) {
	if (key == 27)
		exit(0);
	else if (key == 'd') {
		debug = !debug;
	}
	else if (key == 'f') {
		debugKD = !debugKD;
	}
}*/

/**********************
* END KD TREE ADDITIONS
**********************/

#endif