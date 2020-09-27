#pragma once
#include "sensornode.h"

#ifndef FUNCS_H_
#define FUNCS_H_

template <class T>
class Funcs
{
public:
	Funcs() {};
	~Funcs() {};
	
	int getRandIndex(int limit);
	float getRandFloat();
	double tanhFunc(const double& x);
	double invTanhFunc(const double& x);
	bool checkDuplicates(std::vector<int> source);
	void delDuplicates(Cluster<T>& cluster);
	void delDuplicates(std::vector<Point<T>>& p_list);
	void delDuplicates(std::vector<int>& idx_list);
};


#endif // FUNCS_H_
