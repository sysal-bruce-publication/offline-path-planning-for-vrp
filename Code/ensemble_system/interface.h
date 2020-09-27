#pragma once
#include <string>
#include <vector>
#include <iostream>
#include "sensornode.h"

template <class T>
class Interface
{
public:
	std::vector<SensorNode<T>> sn_list;
	std::vector<std::vector<int>> opt_sol;
	double alg_t = 0.;
	int n_alg = -1;

	Interface();
	~Interface();

private:
	void resetParams();

	void interfaceInvalid(std::string message);

	void interfaceIntro();

	void readInputFromCsv(std::vector<SensorNode<T>>& input_sns);

	void inputCheck();

	void interfaceSelectAlg();

	void interfaceGA(std::vector<SensorNode<T>>& input_sns);

	void interfaceBH(std::vector<SensorNode<T>>& input_sns);

	void interfaceSA(std::vector<SensorNode<T>>& input_sns);

	void saveSubPathToCsv(int alg_num, std::vector<std::vector<int>> out_paths);

	void PdvFlightSimulation(std::vector<std::vector<int>> out_paths);

	void saveOutputToCsv(int pdv_num, float* pct_list, double* pdv_eng_cost, double* charged_eng,
		double* flight_ds, double spent_t);
};