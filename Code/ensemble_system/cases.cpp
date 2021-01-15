#include <fstream>
#include <string>
#include <iostream>
#include <algorithm>
#include "cases.h"
#include "pdv.h"
#include "genetic.h"
#include "blackhole.h"
#include "annealing.h"

using namespace std;

template <class T>
void Cases<T>::initAlgResults() {
	this->alg_pect.resize(3);
	this->alg_cost.resize(3);
	this->alg_rec.resize(3);
	for (int i = 0; i < 3; i++) {
		this->alg_pect[i] = 0.;
		this->alg_cost[i] = 0.;
		this->alg_rec[i] = 0.;
	}
}

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
void Cases<T>::executeGA(int case_num, vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
	int gen_num, int pop_num, int cr_num, int rec_num, int max_neigh) {

	Genetic<T>* ga = new Genetic<T>(w_rec, w_pdv, w_dist, gen_num, pop_num, cr_num, rec_num, max_neigh);
	if (!ga->checkTask(case_num, sn_list)) {
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

		this->alg_pect[0] += pect;
		this->alg_cost[0] += pdv_eng_cost[i];
		this->alg_rec[0] += delta_wsn_eng[i];

		delete pdv;
	}

	this->alg_pect[0] /= n_pdv;

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
void Cases<T>::executeBH(int case_num, vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
	int gen_num, int pop_num, int ar_num, int rec_num, int max_neigh) {

	BlackHole<T>* bh = new BlackHole<T>(w_rec, w_pdv, w_dist, gen_num, pop_num, ar_num, rec_num, max_neigh);
	if (!bh->checkTask(case_num, sn_list)) {
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

		this->alg_pect[1] += pect;
		this->alg_cost[1] += pdv_eng_cost[i];
		this->alg_rec[1] += delta_wsn_eng[i];

		delete pdv;
	}

	this->alg_pect[1] /= n_pdv;

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
void Cases<T>::executeSA(int case_num, vector<SensorNode<T>>& sn_list, int w_rec, int w_pdv, int w_dist,
	double init_temp, double min_temp, double temp_factor, int pop_num,
	int rec_num, int max_neigh) {

	Annealing<T>* sa = new Annealing<T>(w_rec, w_pdv, w_dist, init_temp, min_temp, 
		temp_factor, pop_num, rec_num, max_neigh);

	if (!sa->checkTask(case_num, sn_list)) {
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

		this->alg_pect[2] += pect;
		this->alg_cost[2] += pdv_eng_cost[i];
		this->alg_rec[2] += delta_wsn_eng[i];

		delete pdv;
	}

	this->alg_pect[2] /= n_pdv;

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
void Cases<T>::evaluateAlgResults(int input_num, int iter_num, vector<float> pect_, vector<double> cost_, vector<double> rec_) {
	vector<int> mark = { 0, 0, 0 };
	int rec_idx = distance(rec_.begin(), max_element(rec_.begin(), rec_.end()));
	int cost_idx = distance(cost_.begin(), min_element(cost_.begin(), cost_.end()));
	int pect_idx = distance(pect_.begin(), max_element(pect_.begin(), pect_.end()));
	for (int i = 0; i < 3; i++) {
		if (i == rec_idx) mark[i] += 2;
		if (i == cost_idx) mark[i] += 1;
		if (i == pect_idx) mark[i] += 1;
	}

	int best_idx = distance(mark.begin(), max_element(mark.begin(), mark.end()));

	fstream file;
	file.open("../output/eva_result.csv", fstream::app);
	if (!file) {
		cerr << "eva_result.csv cannot open";
		return;
	}

	file << "For input" << input_num << ".csv and iteration " << iter_num << ", "
		<< "the best alg. is ";

	if (best_idx == 0) file << "Genetic Algorithm." << endl;
	else if (best_idx == 1) file << "Black Hole Algorithm." << endl;
	else file << "Simulated Annealing Algorithm." << endl << endl;

	for (int i = 0; i < 3; i++) {
		file << pect_[i] << "," << cost_[i] << "," << rec_[i] << endl;
	}

	file << "Evaluation end." << endl << endl;
	file.close();
}

template<class T>
void Cases<T>::single_test(int n) {
	for (int j = 0; j < 20; j++) {
		this->initAlgResults();
		cerr << "input " << to_string(n) << "\t iter " << to_string(j) << endl;

		readInputs(n, this->in_sns);
		executeGA(n, this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
		this->in_sns.clear();

		readInputs(n, this->in_sns);
		executeBH(this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
		this->in_sns.clear();

		readInputs(n, this->in_sns);
		executeSA(this->in_sns, 80, 20, 0, 1e3, 1e-4, 0.985, 25, 25, 5);
		this->in_sns.clear();

		//this->evaluateAlgResults(n, j, this->alg_pect, this->alg_cost, this->alg_rec);
	}
}

template <class T>
void Cases<T>::ensemble_test() {

	for (int i = 1; i < 2; i++) {
		
		for (int j = 0; j < 1; j++) {
			this->initAlgResults();
			cerr << "input " << to_string(i) << "\t iter " << to_string(j) << endl;
			
			//readInputs(i, this->in_sns);
			//executeGA(i, this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
			//this->in_sns.clear();

			readInputs(i, this->in_sns);
			executeBH(i, this->in_sns, 80, 20, 0, 50, 50, 50, 25, 5);
			this->in_sns.clear();

			//readInputs(i, this->in_sns);
			//executeSA(i, this->in_sns, 80, 20, 0, 1e3, 5e-3, 0.94, 25, 25, 5);
			//this->in_sns.clear();

			//this->evaluateAlgResults(i, j, this->alg_pect, this->alg_cost, this->alg_rec);
		}
	}
}