#pragma once
#include "sensornode.h"

#ifndef CASES_H_
#define CASES_H_

template <class T>
class Cases {
public:
	std::vector<SensorNode<T>> in_sns;

	Cases() {};
	~Cases() {};

	void readInputs(int n_file, std::vector<SensorNode<T>>& sn_list);
	void executeGA(std::vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
		int gen_num, int pop_num, int cr_num, int rec_num, int max_neigh);
	void executeBH(std::vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
		int gen_num, int pop_num, int ar_num, int rec_num, int max_neigh);
	void executeSA(std::vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
		double init_temp, double min_temp, double temp_factor, int pop_num,
		int rec_num, int max_neigh);
	
	void single_test(int n);
	
	void ensemble_test();
};

#endif // !CASES_H_
