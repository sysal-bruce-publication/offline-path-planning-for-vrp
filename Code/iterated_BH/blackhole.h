#pragma once
#include "sensornode.h"

#ifndef BLACKHOLE_H_
#define BLACKHOLE_H_

using namespace std;

template <class T>
class BlackHole : protected SensorNode<T>
{
public:
	Point<T>* origin = nullptr;
	vector<vector<vector<int>>> tars_idx;
	vector<vector<double>> tars_metric;
	vector<Point<T>> req_ps;
	double alg_time = 0.;

	BlackHole();
	~BlackHole();

	void updateEnergy(vector<SensorNode<T>>& sn_list);
	void initParams(vector<SensorNode<T>*> req_sn_ptr);
	void initOneSol(const int& pdv_num, vector<vector<int>>& idx_list, vector<Point<T>> req_ps, vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr);
	void saveGuessToTxt(int pop_num, int pdv_num_txt, int sn_num, vector<int> path_to_save);
	int calcOptPdvNum(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> temp_req_p);
	bool calcInitGuess(const int& r_num, const int& pdv_num, const int& pop_num, vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> req_ps);
	double calcFarNeighDist(vector<Point<T>> init_path);
	double calcNearNeighDist(vector<Point<T>> init_path);
	double fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> idx_list);
	vector<int> readGuessData(int pop_num, int pdv_num);
	void saveSubPathToCsv(vector<SensorNode<T>> sn_list, vector<vector<int>> path_to_save);
	void attraction(int cur_gen, int cur_pdv, const int& bh_num, vector<SensorNode<T>> sn_list, vector<Point<T>> req_ps, vector<vector<int>>& tar_vec);
	void calcFinalPath(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> candidates);
	bool checkTask(vector<SensorNode<T>>& sn_list);
};

#endif // !BLACKHOLE_H_
