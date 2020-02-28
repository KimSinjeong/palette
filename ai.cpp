#include <iostream>
#include "ai.h"

//i,j starts from 0
//index starts from 0

myNode* newNode(int(*state)[BSIZE]) {
	myNode* returnNode = (myNode*)malloc(sizeof(myNode));
	memcpy(returnNode->matrix, state, sizeof(int) * BSIZE * BSIZE);
	returnNode->node = -1;
	returnNode->score = -2;
	returnNode->degree = 0;
	returnNode->child = NULL;
	return returnNode;
}


myTree* newTree() {
	myTree* returnTree = (myTree*)malloc(sizeof(myTree));
	returnTree->root = NULL;
	return returnTree;
}


void NodeChangeData(myNode* p, int row, int column, int playerjm) {
	p->matrix[row][column] = playerjm;
}

void ParentInsertChild(myNode* p, int(*state)[BSIZE], int Score) {
	p->child = (myNode*)realloc(p->child, sizeof(myNode) * (p->degree + 1));
	memcpy(&(p->child[p->degree].matrix), state, sizeof(int) * BSIZE * BSIZE);
	p->child[p->degree].degree = 0;
	p->child[p->degree].score = Score;
	p->child[p->degree].child = NULL;
	p->degree++;
}


//free instead of delete in website
void DeleteAllChild(myNode* p) {
	if (p == NULL) return;
	for (int i = 0; i < p->degree; i++) DeleteAllChild(&(p->child[i]));
	free(p->child);
	p->degree = 0;
	p->child = NULL;
}


//return 1 connected 5 detected, return 0 else.
//return 2 if conneced 6 or moredetected 
//else, return 0
int SearchEnd(int(*state)[12], int playerjm) {
	int count = 0;
	int count1 = 0;
	int count2 = 0;
	/* Search up and down */
	for (int j = 0; j < 12; j++) {
		for (int i = 0; i < 12; i++) {
			if (state[i][j] == playerjm) count++;
			else count = 0;

			if (count == 5 && i == 11) return 1;
			if (count == 5 && i < 11 && state[i + 1][j] != playerjm) return 1;
			if (count == 5 && i < 11 && state[i + 1][j] == playerjm) return 2;
		}
	}


	/* Search left and right */
	count = 0;
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			if (state[i][j] == playerjm) count++;
			else count = 0;

			if (count == 5 && j == 11) return 1;
			if (count == 5 && j < 11 && state[i][j + 1] != playerjm) return 1;
			if (count == 5 && j < 11 && state[i][j + 1] == playerjm) return 2;
		}
	}



	/* Search diagonal */
	count1 = 0;
	count2 = 0;
	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 12 - i; j++) {
			if (state[i + j][j] == playerjm) count1++;
			else count1 = 0;
			if (state[j][i + j] == playerjm) count2++;
			else count2 = 0;

			if (count1 == 5 && j == 11 - i) return 1;
			if (count1 == 5 && j < 11 - i && state[i + j + 1][j + 1] != playerjm) return 1;
			if (count1 == 5 && j < 11 - i && state[i + j + 1][j + 1] == playerjm) return 2;
			if (count2 == 5 && j == 11 - i) return 1;
			if (count2 == 5 && j < 11 - i && state[j + 1][i + j + 1] != playerjm) return 1;
			if (count2 == 5 && j < 11 - i && state[j + 1][i + j + 1] == playerjm) return 2;

		}
	}


	/* Search skew diagonal */
	count1 = 0;
	count2 = 0;
	for (int i = 11; i > 3; i--) {
		for (int j = 0; j < i + 1; j++) {
			if (state[i - j][j] == playerjm) count1++;
			else count1 = 0;
			if (state[11 - j][11 - i + j] == playerjm) count2++;
			else count2 = 0;

			if (count1 == 5 && j == i) return 1;
			if (count1 == 5 && j < i && state[i - j - 1][j + 1] != playerjm) return 1;
			if (count1 == 5 && j < i && state[i - j - 1][j + 1] == playerjm) return 2;
			if (count2 == 5 && j == i) return 1;
			if (count2 == 5 && j < i && state[11 - j - 1][11 - i + j + 1] != playerjm) return 1;
			if (count2 == 5 && j < i && state[11 - j - 1][11 - i + j + 1] == playerjm) return 2;

		}
	}

	return 0;
}


/* compare function for qsort */
int compare(const void* num1, const void* num2) {
	if (*(int*)num1 > * (int*)num2) return 1;
	else if (*(int*)num1 < *(int*)num2) return -1;
	else return 0;

}


/* Search possible moves */
/* 2x2 square */
/* output: array of index of possible moves */
void PossibleMoves(int* array, int(*state)[12]) {
	int temp[3500];
	int ind = 0;
	for (int i = 0; i < BSIZE; i++) {
		for (int j = 0; j < BSIZE; j++) {

			if (state[i][j] != 0)
				for (int temp1 = -1; temp1 < 2; temp1++) {
					for (int temp2 = -1; temp2 < 2; temp2++) {
						if (temp1 != 0 || temp2 != 0) {
							if (0 <= i + temp1 && i + temp1 < 12 && 0 <= j + temp2 && j + temp2 < 12 && state[i + temp1][j + temp2] == 0) {
								temp[ind] = (i + temp1) + 12 * (j + temp2);
								ind++;
							}
						}
					}
				}
		}
	}
	qsort(temp, ind, sizeof(int), compare);
	// remove duplication
	int base, present;
	base = 0;
	for (present = 1; present < ind; present++) {
		if (temp[present] != temp[base]) {
			base++;
			temp[base] = temp[present];
		}
	}
	memcpy(array, temp, sizeof(int) * (base + 1));
	array[base + 1] = -1;
}


int ArraySize(int* array) {
	int ind = 0;
	while (array[ind] != -1) {
		ind++;
	}
	return ind;
}

/* compute score of state perspective of player */
//output: (score1)(score2)(score2)(score3)(score4)in octal number
int ComputeScoreUD(int(*state)[12], int playerjm) {
	int score1 = 0;// oooxo, ooxoo,oxooo, xoooox, xoooob, boooox . warning: connected 6 is important. ooxoo consider later.//////////////
	//oooxo check not oooxoo
	int score2 = 0;// xxoooxx //.
	int score3 = 0;// bxoooxx, xxoooxb (do not need to consider o in side)
	int score4 = 0;// boooxxb, boooxxx, bxxooob, xxxooob, bxoooxb
	int count = 0;
	/* Search up and down */
	for (int j = 0; j < 12; j++) {
		for (int i = 0; i < 12; i++) {
			if (state[i][j] == playerjm) count++;
			else count = 0;
			if (count == 3) {
				if (i < 11 && state[i + 1][j] == playerjm) {
				} //do nothing
				else if (i == 2 || i == 3 || i == 10 || i == 11) {
					switch (i) {
					case 2:
						if (state[3][j] == 0 && state[4][j] == 0 && state[5][j] != playerjm) score4++;
						if (state[3][j] == 0 && state[4][j] == playerjm && state[5][j] != playerjm) score1++;
						break;
					case 3:
						if (state[4][j] == 0 && state[5][j] == playerjm && state[6][j] != playerjm) score1++;
						if (state[0][j] == 0 && state[4][j] == 0 && state[5][j] == 0) score3++; // bxoooxx
						if (state[0][j] == 3 - playerjm && state[4][j] == 0 && state[5][j] == 0) score3++; //xxoooxb       
						break;
					case 10:
						if (state[5][j] != playerjm && state[6][j] == playerjm && state[7][j] == 0) score1++;
						if (state[6][j] == 0 && state[7][j] == 0 && state[11][j] == 0) score3++; // xxoooxb
						if (state[5][j] != playerjm && state[6][j] == 0 && state[7][j] == 0 && state[11][j] == 3 - playerjm) score4++; // bxxooob, xxxooob
						if (state[6][j] == 3 - playerjm && state[7][j] == 0 && state[11][j] == 0) score4++; // bxoooxb
						break;
					case 11:
						if (state[6][j] != playerjm && state[7][j] == playerjm && state[8][j] == 0) score1++;
						if (state[6][j] != playerjm && state[7][j] == 0 && state[8][j] == 0) score4++;
						break;
					}
				}
				else {
					if (state[i + 1][j] == 0 && state[i + 2][j] == playerjm) {
						if (state[i - 3][j] == 3 - playerjm || (state[i - 3][j] == 0 && state[i - 4][j] != playerjm))
							score1++;
					} //oooxo. check not oxoooxo and oooxoo 
					if (state[i - 3][j] == 0 && state[i - 4][j] == playerjm) {
						if (state[i + 1][j] == 3 - playerjm || (state[i + 1][j] == 0 && state[i + 2][j] != playerjm))
							score1++;
					} //oxooo. check not oxoooxo and ooxooo
					if (state[i - 3][j] == 0 && state[i - 4][j] == 0 && state[i + 1][j] == 0 && state[i + 2][j] == 0) {
						score2++;
					} //xxoooxx
					if (state[i - 4][j] == 3 - playerjm && state[i - 3][j] == 0 && state[i + 1][j] == 0 && state[i + 2][j] == 0) {
						score3++;
					} //bxoooxx
					if (state[i - 4][j] == 0 && state[i - 3][j] == 0 && state[i + 1][j] == 0 && state[i + 2][j] == 3 - playerjm) {
						score3++;
					} //xxoooxb
					if (state[i - 3][j] == 3 - playerjm && state[i + 1][j] == 0 && state[i + 2][j] == 0) {
						if (i != 9) {
							if (state[i + 3][j] != playerjm) score4++;
						}
						else {
							score4++;
						}
					} //boooxxb, boooxxx
					if (state[i - 4][j] == 0 && state[i - 3][j] == 0 && state[i + 1][j] == 3 - playerjm) {
						if (i != 4) {
							if (state[i - 5][j] != playerjm) score4++;
						}
						else {
							score4++;
						}
					} //bxxooob, xxxooob
					if (state[i - 4][j] == 3 - playerjm && state[i - 3][j] == 0 && state[i + 1][j] == 0 && state[i + 2][j] == 3 - playerjm) {
						score4++;
					} //bxoooxb
				}
			}

			// oooxo, ooxoo,oxooo, xoooox, xoooob, boooox . warning: connected 6 is important. ooxoo consider later.//////////////
			// xxoooxx .
			// bxoooxx, xxoooxb (do not need to consider o in side)
			// boooxxb, boooxxx, bxxooob, xxxooob, bxoooxb

			if (count == 4) {
				if (i != 3 && i != 11) {
					if (state[i - 4][j] == 0 && state[i + 1][j] == 0) {
						score1++;
					}
				} // xoooox
				if (i != 3 && state[i - 4][j] == 0) {
					if (i == 11) {
						score1++;
					}
					else if (state[i + 1][j] == 3 - playerjm) {
						score1++;
					}
				}// xoooob

				if (i != 11 && state[i - 4][j] == 0) {
					if (i == 3) {
						score1++;
					}
					else if (state[i - 4][j] == 3 - playerjm) {
						score1++;
					}
				}// boooox      
			}
		}
	}

	int score = (score1 * 64 * 64 * 64) + (score2 * 64 * 64) + (score3 * 64) + score4;
	return score;
}

//diagonal: shift up and down and, after it transpose
// transpose since we wrote a code only ComputeScoreUD
int ComputeScore(int(*state)[12], int playerjm) {
	int score = 0;
	int transpose[12][12];
	int diagonal1[12][12];
	int diagonal2[12][12];
	int skewdiagonal1[12][12];
	int skewdiagonal2[12][12];

	/* compute score up and down */
	score = ComputeScoreUD(state, playerjm);

	/* compute score left and right */
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			transpose[i][j] = state[j][i];
		}
	}
	score = score + ComputeScoreUD(transpose, playerjm);

	/* compute score main diagonal */
	for (int j = 0; j < 12; j++) {
		for (int i = 0; i < 12; i++) {
			if (i + j > -1 && i + j < 12)
				diagonal1[j][i] = state[i + j][j];
			else
				diagonal1[j][i] = 3 - playerjm;
		}
	}
	score = score + ComputeScoreUD(diagonal1, playerjm);


	for (int j = 0; j < 12; j++) {
		for (int i = 0; i < 12; i++) {
			if (i + 11 - j > -1 && i + 11 - j < 12)
				diagonal2[j][i] = state[i + 11 - j][j];
			else
				diagonal2[j][i] = 3 - playerjm;
		}
	}
	score = score + ComputeScoreUD(diagonal2, playerjm);

	/* compute score skew diagonal */
	/* compute score main diagonal */
	for (int j = 0; j < 12; j++) {
		for (int i = 0; i < 12; i++) {
			if (i - j > -1 && i - j < 12)
				skewdiagonal1[j][i] = state[i - j][j];
			else
				skewdiagonal1[j][i] = 3 - playerjm;
		}
	}
	score = score + ComputeScoreUD(skewdiagonal1, playerjm);

	for (int j = 0; j < 12; j++) {
		for (int i = 0; i < 12; i++) {
			if (i - 11 + j > -1 && i - 11 + j < 12)
				skewdiagonal2[j][i] = state[i - 11 + j][j];
			else
				skewdiagonal2[j][i] = 3 - playerjm;
		}
	}
	score = score + ComputeScoreUD(skewdiagonal2, playerjm);
	return score;
}

//warning: I did not consider ooxo, xoxoob, ooxoo
//warning: I did not consdier oxxoooxxo
//warning: I din not condiser for oooxo, but not oxoooxo and oooxoo
//warning: If player end, score needs to be lowest. I did not consider



// oooxo, ooxoo,oxooo, xoooox, xoooob, boooox . warning: connected 6 is important. ooxoo consider later.//////////////
// xxoooxx .
// bxoooxx, xxoooxb (do not need to consider o in side)
// boooxxb, boooxxx, bxxooob, xxxooob, bxoooxb

//score: 1~9
int CompareScore(int score1, int score2) { //score1: computer, score2: player

	int score11 = score1 / (64 * 64 * 64);
	int score12 = score1 / (64 * 64);
	int score13 = score1 / 64;
	int score14 = score1 % 64;
	int score21 = score2 / (64 * 64 * 64);
	int score22 = score2 / (64 * 64);
	int score23 = score2 / 64;
	int score24 = score2 % 64;


	if (score11 > 0) return 9;
	else if (score21 > 0) return 1;
	else if (score12 > 0) return 8;
	else if (score22 > 0) return 2;
	else if (score13 > 0) return 7;
	else if (score23 > 0) return 3;
	else if (score14 > 0) return 6;
	else if (score24 > 0) return 4;
	else return 5;
}

void minimax(myTree* t, int(*state)[12]) {
	myNode* rootNode = t->root;
	int degree1 = rootNode->degree;
	for (int i = 0; i < degree1; i++) {
		myNode* currentNode1 = &(rootNode->child[i]);
		int degree2 = currentNode1->degree;
		int min = 10;
		int node2 = 0;
		for (int j = 0; j < degree2; j++) {
			myNode currentNode2 = currentNode1->child[j];
			if (currentNode2.score < min) {
				node2 = j;
				min = currentNode2.score;
			}
		}
		currentNode1->node = node2;
		currentNode1->score = min;
	}
	int max = -2;
	int node1 = 0;
	for (int i = 0; i < degree1; i++) {
		myNode currentNode1 = rootNode->child[i];
		if (currentNode1.score > max) {
			node1 = i;
			max = currentNode1.score;
		}
	}
	memcpy(state, rootNode->child[node1].matrix, sizeof(int) * BSIZE * BSIZE);
}

//warning: I didn't consider there is no Node

void ClearArray(int* array) {
	int ind = 0;
	while (ind == -1) {
		array[ind] = 0;
		ind++;
	}
	array[ind] = 0;
}


//playerjm: player just moved
void NextStone(int(*state)[12], int playerjm) {
	int possibleArray[3500];
	playerjm = 3 - playerjm;
	myTree* tree_gomoku;
	tree_gomoku = newTree();
	tree_gomoku->root = newNode(state);

	/* depth 1 */
	//////////if there is end, go there.
	PossibleMoves(possibleArray, state);
	int size = ArraySize(possibleArray);
	int index;
	int End;
	int state_temp[12][12];
	for (int i = 0; i < size; i++) {
		memcpy(state_temp, state, sizeof(int) * BSIZE * BSIZE);
		index = possibleArray[i];
		state_temp[index % 12][index / 12] = playerjm;
		End = SearchEnd(state_temp, playerjm);
		if (End == 1) {
			printf("player lose\n");
			memcpy(state, state_temp, sizeof(int) * BSIZE * BSIZE);
			return;
		}
		else if (End == 0) {
			ParentInsertChild(tree_gomoku->root, state_temp, -2);
		}
	}

	ClearArray(possibleArray);
	/* depth2 */
	for (int i1 = 0; i1 < tree_gomoku->root->degree; i1++) {
		PossibleMoves(possibleArray, tree_gomoku->root->child[i1].matrix);
		size = ArraySize(possibleArray);
		for (int i2 = 0; i2 < size; i2++) {
			index = possibleArray[i2];
			int state_temp[12][12];
			memcpy(state_temp, tree_gomoku->root->child[i1].matrix, sizeof(int) * BSIZE * BSIZE);
			state_temp[index % 12][index / 12] = 3 - playerjm;
			int End = SearchEnd(state_temp, 3 - playerjm);
			if (End == 1) {
				ParentInsertChild(&(tree_gomoku->root->child[i1]), state_temp, -1);
			}
			else if (End == 0) {
				int score1 = ComputeScore(state_temp, playerjm);
				int score2 = ComputeScore(state_temp, 3 - playerjm);
				int score = CompareScore(score1, score2);
				ParentInsertChild(&(tree_gomoku->root->child[i1]), state_temp, score);
			}
		}
	}
	minimax(tree_gomoku, state);
}

int ai_turn(myPoint(*points)[BSIZE]) {
	cout << "ai turn start" << endl;
	int state[BSIZE][BSIZE];
	int temp_state[BSIZE][BSIZE];
	to_state_arr(points, state);
	memcpy(temp_state, state, sizeof(int) * BSIZE * BSIZE);
	if (SearchEnd(state, 2) == 1) return -2;
	NextStone(state, 2);
	to_point_arr(state, points);
	cout << "ai turn end, index is " << endl;
	if (SearchEnd(state, 1) == 1) return -1;
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 12; j++) {
			if (temp_state[i][j] != state[i][j]) return i * 12 + j;

		}
	}

	return -3;
}



//check clear all of malloc
