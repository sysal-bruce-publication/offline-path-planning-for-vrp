#include <fstream>
#include <string>
#include <iostream>
#include "cases.h"
#include "pdv.h"
#include "genetic.h"
#include "blackhole.h"
#include "annealing.h"

using namespace std;

template <class T>
void Cases<T>::readInputs(int n_file, std::vector<SensorNode<T>>& sn_list) {
	fstream file;
	file.open("../input/input" + to_string(n_file) + ".csv", fstream::in);
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

		sn_list.push_back(SensorNode<T>(static_cast<float>(stof(xs) / 100.),
			static_cast<float>(stof(ys) / 100.), stof(volts), stoi(weights),
			static_cast<bool>(stoi(flags))));
	}
	file.close();
}

template <class T>
void Cases<T>::executeGA(vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
	int gen_num, int pop_num, int cr_num, int rec_num, int max_neigh) {

	Genetic<T>* ga = new Genetic<T>(w_rec, w_pdv, w_dist, gen_num, pop_num, cr_num, rec_num, max_neigh);
	if (!ga->checkTask(sn_list)) {
		delete ga;
		cerr << "No enough sensor nodes to be recharged !";
		return;
	}

	int n_pdv = ga->best_sol.size();
	vector<vector<Point<T>>> final_sols;
	final_sols.resize(n_pdv);
	int n_sns = 0;
	for (int i = 0; i < n_pdv; i++) {
		n_sns = ga->best_sol[i].size();
		final_sols[i].resize(n_sns);
		for (unsigned j = 0; j < ga->best_sol[i].size(); j++) {
			final_sols[i][j] = sn_list[ga->best_sol[i][j]].pos;
		}
	}

	float* pect_list = new float[n_pdv];
	double* pdv_eng_cost = new double[n_pdv];
	double* delta_wsn_eng = new double[n_pdv];
	double* flight_ds = new double[n_pdv];
	double* flight_time = new double[n_pdv];

	for (int i = 0; i < n_pdv; i++) {
		pect_list[i] = (float)(0.);
		pdv_eng_cost[i] = 0.;
		delta_wsn_eng[i] = 0.;
		flight_ds[i] = 0.;
		flight_time[i] = 0.;

		auto* pdv = new PDV<T>();
		float pect = 0.;

		pect = pdv->flightSimulation(delta_wsn_eng[i], flight_time[i], sn_list, final_sols[i]);

		pect_list[i] = pect;
		pdv_eng_cost[i] = 187. - pdv->f_eng;
		flight_ds[i] = pdv->f_dist;

		delete pdv;
	}

	double max_t = flight_time[0];
	for (int i = 1; i < n_pdv; i++) {
		if (flight_time[i] > max_t) max_t = flight_time[i];
	}

	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V > sn_list[i].getCriticalVolt()) {
			sn_list[i].updateEnergy(max_t * 3600, sn_list[i].SC_E);
			sn_list[i].updateVolt(sn_list[i].SC_V);
			sn_list[i].updateWeight(sn_list[i].SC_V, sn_list[i].weight);
		}
	}

	fstream file;
	file.open("../output/ga_sum.csv", fstream::app);
	if (!file) {
		cerr << "ga_sum.csv cannot open";
		return;
	}

	for (int j = 0; j < n_pdv; j++) {
		file << j << "," << flight_ds[j] << "," << pect_list[j]
			<< "," << pdv_eng_cost[j] << "," << delta_wsn_eng[j]
			<< "," << ga->alg_time << endl;
	}
	file << endl;
	file.close();

	delete ga;
	delete[] pect_list;
	delete[] pdv_eng_cost;
	delete[] delta_wsn_eng;
	delete[] flight_ds;
	delete[] flight_time;
}

template <class T>
void Cases<T>::executeBH(vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
	int gen_num, int pop_num, int ar_num, int rec_num, int max_neigh) {

	BlackHole<T>* bh = new BlackHole<T>(w_rec, w_pdv, w_dist, gen_num, pop_num, ar_num, rec_num, max_neigh);
	if (!bh->checkTask(sn_list)) {
		delete bh;
		cerr << "No enough sensor nodes to be recharged !";
		return;
	}

	int n_pdv = bh->best_sol.size();
	vector<vector<Point<T>>> final_sols;
	final_sols.resize(n_pdv);
	int n_sns = 0;
	for (int i = 0; i < n_pdv; i++) {
		n_sns = bh->best_sol[i].size();
		final_sols[i].resize(n_sns);
		for (unsigned j = 0; j < bh->best_sol[i].size(); j++) {
			final_sols[i][j] = sn_list[bh->best_sol[i][j]].pos;
		}
	}

	float* pect_list = new float[n_pdv];
	double* pdv_eng_cost = new double[n_pdv];
	double* delta_wsn_eng = new double[n_pdv];
	double* flight_ds = new double[n_pdv];
	double* flight_time = new double[n_pdv];

	for (int i = 0; i < n_pdv; i++) {
		pect_list[i] = (float)(0.);
		pdv_eng_cost[i] = 0.;
		delta_wsn_eng[i] = 0.;
		flight_ds[i] = 0.;
		flight_time[i] = 0.;

		auto* pdv = new PDV<T>();
		float pect = 0.;

		pect = pdv->flightSimulation(delta_wsn_eng[i], flight_time[i], sn_list, final_sols[i]);

		pect_list[i] = pect;
		pdv_eng_cost[i] = 187. - pdv->f_eng;
		flight_ds[i] = pdv->f_dist;

		delete pdv;
	}

	double max_t = flight_time[0];
	for (int i = 1; i < n_pdv; i++) {
		if (flight_time[i] > max_t) max_t = flight_time[i];
	}

	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V > sn_list[i].getCriticalVolt()) {
			sn_list[i].updateEnergy(max_t * 3600, sn_list[i].SC_E);
			sn_list[i].updateVolt(sn_list[i].SC_V);
			sn_list[i].updateWeight(sn_list[i].SC_V, sn_list[i].weight);
		}
	}

	fstream file;
	file.open("../output/bh_sum.csv", fstream::app);
	if (!file) {
		cerr << "bh_sum.csv cannot open";
		return;
	}

	for (int j = 0; j < n_pdv; j++) {
		file << j << "," << flight_ds[j] << "," << pect_list[j]
			<< "," << pdv_eng_cost[j] << "," << delta_wsn_eng[j]
			<< "," << bh->alg_time << endl;
	}
	file << endl;
	file.close();

	delete bh;
	delete[] pect_list;
	delete[] pdv_eng_cost;
	delete[] delta_wsn_eng;
	delete[] flight_ds;
	delete[] flight_time;
}

template <class T>
void Cases<T>::executeSA(vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
	double init_temp, double min_temp, double temp_factor, int pop_num,
	int rec_num, int max_neigh) {

	Annealing<T>* sa = new Annealing<T>(w_rec, w_pdv, w_dist, init_temp, min_temp, 
		temp_factor, pop_num, rec_num, max_neigh);

	if (!sa->checkTask(sn_list)) {
		delete sa;
		cerr << "No enough sensor nodes to be recharged !";
		return;
	}

	int n_pdv = sa->best_sol.size();
	vector<vector<Point<T>>> final_sols;
	final_sols.resize(n_pdv);
	int n_sns = 0;
	for (int i = 0; i < n_pdv; i++) {
		n_sns = sa->best_sol[i].size();
		final_sols[i].resize(n_sns);
		for (unsigned j = 0; j < sa->best_sol[i].size(); j++) {
			final_sols[i][j] = sn_list[sa->best_sol[i][j]].pos;
		}
	}

	float* pect_list = new float[n_pdv];
	double* pdv_eng_cost = new double[n_pdv];
	double* delta_wsn_eng = new double[n_pdv];
	double* flight_ds = new double[n_pdv];
	double* flight_time = new double[n_pdv];

	for (int i = 0; i < n_pdv; i++) {
		pect_list[i] = (float)(0.);
		pdv_eng_cost[i] = 0.;
		delta_wsn_eng[i] = 0.;
		flight_ds[i] = 0.;
		flight_time[i] = 0.;

		auto* pdv = new PDV<T>();
		float pect = 0.;

		pect = pdv->flightSimulation(delta_wsn_eng[i], flight_time[i], sn_list, final_sols[i]);

		pect_list[i] = pect;
		pdv_eng_cost[i] = 187. - pdv->f_eng;
		flight_ds[i] = pdv->f_dist;

		delete pdv;
	}

	double max_t = flight_time[0];
	for (int i = 1; i < n_pdv; i++) {
		if (flight_time[i] > max_t) max_t = flight_time[i];
	}

	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V > sn_list[i].getCriticalVolt()) {
			sn_list[i].updateEnergy(max_t * 3600, sn_list[i].SC_E);
			sn_list[i].updateVolt(sn_list[i].SC_V);
			sn_list[i].updateWeight(sn_list[i].SC_V, sn_list[i].weight);
		}
	}

	fstream file;
	file.open("../output/sa_sum.csv", fstream::app);
	if (!file) {
		cerr << "sa_sum.csv cannot open";
		return;
	}

	for (int j = 0; j < n_pdv; j++) {
		file << j << "," << flight_ds[j] << "," << pect_list[j]
			<< "," << pdv_eng_cost[j] << "," << delta_wsn_eng[j]
			<< "," << sa->alg_time << endl;
	}
	file << endl;
	file.close();

	delete sa;
	delete[] pect_list;
	delete[] pdv_eng_cost;
	delete[] delta_wsn_eng;
	delete[] flight_ds;
	delete[] flight_time;
}

template<class T>
void Cases<T>::single_test(int n) {
	for (int j = 0; j < 20; j++) {
		cerr << "input " << to_string(n) << "\t iter " << to_string(j) << endl;

		readInputs(n, this->in_sns);
		executeGA(this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
		this->in_sns.clear();

		readInputs(n, this->in_sns);
		executeBH(this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
		this->in_sns.clear();

		readInputs(n, this->in_sns);
		executeSA(this->in_sns, 80, 20, 0, 1e3, 1e-4, 0.985, 25, 25, 5);
		this->in_sns.clear();
	}
}

template <class T>
void Cases<T>::ensemble_test() {
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 20; j++) {
			cerr << "input " << to_string(i) << "\t iter " << to_string(j) << endl;
			
			readInputs(i, this->in_sns);
			executeGA(this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
			this->in_sns.clear();

			readInputs(i, this->in_sns);
			executeBH(this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
			this->in_sns.clear();

			readInputs(i, this->in_sns);
			executeSA(this->in_sns, 80, 20, 0, 1e3, 5e-3, 0.94, 25, 25, 5);
			this->in_sns.clear();
		}
	}
}