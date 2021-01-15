#pragma once
#include "sensornode.h"

#ifndef CASES_H_
#define CASES_H_

template <class T>
class Cases {
public:
	std::vector<SensorNode<T>> in_sns;
	std::vector<float> alg_pect;
	std::vector<double> alg_cost;
	std::vector<double> alg_rec;

	Cases() {};
	~Cases() {};

	void initAlgResults();
	void readInputs(int n_file, std::vector<SensorNode<T>>& sn_list);
	void executeGA(int case_num, std::vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
		int gen_num, int pop_num, int cr_num, int rec_num, int max_neigh);
	void executeBH(int case_num, std::vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
		int gen_num, int pop_num, int ar_num, int rec_num, int max_neigh);
	void executeSA(int case_num, std::vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
		double init_temp, double min_temp, double temp_factor, int pop_num,
		int rec_num, int max_neigh);
	
	void single_test(int n);
	void ensemble_test();

	void evaluateAlgResults(int input_num, int iter_num, std::vector<float> pect_, std::vector<double> cost_, std::vector<double> rec_);
};

#endif // !CASES_H_
