#pragma once

#ifndef _KD_TREE_H_
#define _KD_TREE_H_

class KD_tree 
{

private:

	class Agent_Node
	{
	public:

		size_t begin; //beginning node number

		size_t end; //ending node number

		size_t left; //left node number

		size_t right; //right node number

		float maxX; float maxY;

		float minX; float minY;
	};

	class Obstacle_Node
	{

		Obstacle_Node* left;
		Obstacle_Node* right;

		//const Obstacle* obstacle;

	};

};

#endif