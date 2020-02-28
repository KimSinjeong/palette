#ifndef _AI
#define _AI

#include "stdio.h"
#include "stdlib.h""
#include "string.h"
#include "myLine.h"
#include "myPoint.h"

const int BSIZE = 12;

using namespace std;

typedef struct tagNode {
	int matrix[BSIZE][BSIZE];
	int degree;
	int score;
	int node; //for minimax algorithm
	struct tagNode* child;
}myNode;


typedef struct tagTree {
	struct tagNode* root;
}myTree;

//i,j starts from 0
//index starts from 0
myNode* newNode(int(*state)[BSIZE]);
myTree* newTree();

void NodeChangeData(myNode* p, int row, int column, int playerjm);
void ParentInsertChild(myNode* p, int(*state)[BSIZE], int Score);
//free instead of delete in website
void DeleteAllChild(myNode* p);
//return 1 connected 5 detected, return 0 else.
//return 2 if conneced 6 or moredetected 
//else, return 0
int SearchEnd(int(*state)[12], int playerjm);
/* compare function for qsort */
int compare(const void* num1, const void* num2);
/* Search possible moves */
/* 2x2 square */
/* output: array of index of possible moves */
void PossibleMoves(int* array, int(*state)[12]);
/* print all child Nodes */
int ArraySize(int* array);
/* compute score of state perspective of player */
//output: (score1)(score2)(score2)(score3)(score4)in octal number
int ComputeScoreUD(int(*state)[12], int playerjm);
//diagonal: shift up and down and, after it transpose
// transpose since we wrote a code only ComputeScoreUD
int ComputeScore(int(*state)[12], int playerjm);
//score: 1~9
int CompareScore(int score1, int score2);
void minimax(myTree* t, int(*state)[12]);
//warning: I didn't consider there is no Node
void ClearArray(int* array);
//playerjm: player just moved
void NextStone(int(*state)[12], int playerjm);
int ai_turn(myPoint(*points)[BSIZE]);

#endif