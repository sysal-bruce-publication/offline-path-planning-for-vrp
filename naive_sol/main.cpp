#include <fstream>
#include <string>
#include <iostream>
#include <algorithm>
#include "pdv.h"
#include "point.h"
#include "point.cpp"
#include "pdv.cpp"
#include "sensornode.h"
#include "sensornode.cpp"

using namespace std;

vector<int> pdv_nums = { 5, 8, 10, 8, 15, 20, 15, 23, 32 };

void readInputs(int n_file, std::vector<SensorNode<double>>& sn_list) {
	fstream file;
	file.open("../../input/input" + to_string(n_file) + ".csv", fstream::in);
	if (!file) {
		cerr << "input" + to_string(n_file) + ".csv cannot open";
		return;
	}

	string line;
	if (file.good()) getline(file, line);

	string xs, ys, flags, volts, weights;
	while (getline(file, xs, ',')) {
		getline(file, ys, ',');
		getline(file, flags, ',');
		getline(file, volts, ',');
		getline(file, weights);

		sn_list.push_back(SensorNode<double>(static_cast<float>(stof(xs) / 100.),
			static_cast<float>(stof(ys) / 100.), stof(volts), stoi(weights),
			static_cast<bool>(stoi(flags))));
	}
	file.close();
}


void readSubPaths(int n_case, int n_pdv, vector<Point<double>>& sn_list) {
	fstream file;
	file.open("../../input/naive/sub" + to_string(n_case) + "/sub_path" + to_string(n_pdv) + ".csv", fstream::in);
	if (!file) {
		cerr << "Input of case " + to_string(n_case) + " sub path " + to_string(n_pdv) + "cannot open";
		return;
	}

	string line;
	if (file.good()) getline(file, line);

	string xs, ys;
	while (getline(file, xs, ',')) {
		getline(file, ys);

		sn_list.push_back(Point<double>(static_cast<float>(stof(xs) / 100.),
			static_cast<float>(stof(ys) / 100.)));
	}

	file.close();
}

int main() {

	for (int i = 0; i < 9; i++) {
		cerr << "case" << i + 1 << endl;
		vector<SensorNode<double>> sn_list;
		readInputs(i, sn_list);

		vector<vector<Point<double>>> flight_path;
		int n_pdv = pdv_nums[i];
		flight_path.resize(n_pdv);
		int n_sns = 0;
		for (int j = 0; j < n_pdv; j++) {
			readSubPaths(i, j, flight_path[j]);
		}

		float* pect_list = new float[n_pdv];
		double* pdv_eng_cost = new double[n_pdv];
		double* delta_wsn_eng = new double[n_pdv];
		double* flight_ds = new double[n_pdv];
		double* flight_time = new double[n_pdv];

		for (int j = 0; j < n_pdv; j++) {
			pect_list[j] = (float)(0.);
			pdv_eng_cost[j] = 0.;
			delta_wsn_eng[j] = 0.;
			flight_ds[j] = 0.;
			flight_time[j] = 0.;

			auto* pdv = new PDV<double>();
			float pect = 0.;

			pect = pdv->flightSimulation(delta_wsn_eng[j], flight_time[j], sn_list, flight_path[j]);

			pect_list[j] = pect;
			pdv_eng_cost[j] = 187. - pdv->f_eng;
			flight_ds[j] = pdv->f_dist;

			delete pdv;
		}

		fstream file;
		file.open("../output/sum.csv", fstream::app);
		if (!file) {
			cerr << "sum.csv cannot open";
			return -1;
		}

		for (int j = 0; j < n_pdv; j++) {
			file << j << "," << flight_ds[j] << "," << pect_list[j]
				<< "," << pdv_eng_cost[j] << "," << delta_wsn_eng[j]
				<< endl;
		}
		file << endl;
		file.close();

		delete[] pect_list;
		delete[] pdv_eng_cost;
		delete[] delta_wsn_eng;
		delete[] flight_ds;
		delete[] flight_time;
	}

	return 0;
}