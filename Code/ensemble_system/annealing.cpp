/*! @file annealing.cpp
 *
 *  @warning This is the internal cpp file of the ODP project.
 *  Do not use it directly in other code.
 *  Copyright (C) Qiuchen Qian, 2020
 *  Imperial College, London
 */


#include <cmath>
#include <chrono>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include "pdv.h"
#include "annealing.h"
#include "funcs.h"

using namespace std;

template <class T>
Annealing<T>::Annealing() {
	this->origin = new Point<T>();
}

template <class T>
Annealing<T>::Annealing(int w_wsn_eng, int w_pdv_eng, int w_pdv_dist, double init_t, double min_t,
		double t_fac, int n_pop, int n_req, int max_r) {
	this->origin = new Point<T>();
	this->alg_time = 0.;
	this->coeff_wsn_eng = w_wsn_eng;
	this->coeff_pdv_eng = w_pdv_eng;
	this->coeff_dist = w_pdv_dist;
	this->init_temp = init_t;
	this->min_temp = min_t;
	this->temp_factor = t_fac;
	this->pop = n_pop;
	this->min_req_num = n_req;
	this->max_num_r = max_r;
}

template <class T>
Annealing<T>::~Annealing() {
	delete this->origin;
}

template<class T>
void Annealing<T>::initParams(vector<SensorNode<T>*>& req_sn_ptr) {
	this->req_ps.resize(req_sn_ptr.size());
	for (unsigned i = 0; i < req_sn_ptr.size(); i++) {
		this->req_ps[i] = req_sn_ptr[i]->pos;
	}

	if (this->tars_idx.size() != 0) {
		for (int i = 0; i < this->pop; i++) {
			if (this->trails_idx[i].size() != 0) this->trails_idx[i].clear();
			if (this->tars_idx[i].size() != 0) this->tars_idx[i].clear();
		}
	}
	this->tars_idx.resize(this->pop);
	this->trails_idx.resize(this->pop);

	if (this->tars_met.size() != 0) {
		for (unsigned i = 0; i < tars_met.size(); i++) {
			this->tars_met.clear();
		}
		this->trails_met.clear();
	}
	this->tars_met.resize(this->pop);
}

template <class T>
void Annealing<T>::saveGuessToTxt(int pop_num, int pdv_num_txt, int sn_num, vector<int> path_to_save) {
	string fname = "../input/initial_guess/pop" + to_string(pop_num)
		+ "_pdv" + to_string(pdv_num_txt) + ".txt";

	fstream file;
	try {
		file.open(fname, fstream::out);
		if (!file.is_open())
			throw runtime_error(strerror(errno));
	}
	catch (const exception& e) {
		cerr << e.what() << "\n";
	}

	for (int i = 0; i < sn_num; i++) {
		file << path_to_save[i];
		if (i == sn_num - 1) {
			file << endl;
			break;
		}
		file << ",";
	}
	file.close();
}

template <class T>
int Annealing<T>::calcOptPdvNum(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> temp_req_p) {
	PDV<T>* temp_pdv = new PDV<T>();

	int pdv_num = 1;
	do {
		vector<double> d_list = temp_pdv->pos.calcDist(temp_req_p);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

		int this_cn = -1;
		for (unsigned i = 0; i < sn_list.size(); i++) {
			if (sn_list[i].pos.isCoincide(req_sn_ptr[next]->pos)) {
				this_cn = i;
				break;
			}
		}

		if (temp_pdv->calcEnergyCost(temp_req_p[next].calcDist(Point<T>()) / temp_pdv->getPdvSpeed())
			+ temp_pdv->calcEnergyCost(this->pos.calcDist(temp_req_p[next]) / temp_pdv->getPdvSpeed()) > temp_pdv->f_eng) {

			pdv_num++;
			temp_pdv->resetPdvStatus();
			continue;
		}

		temp_pdv->updatePdvStatus(temp_req_p[next]);
		
		double ipt_eng = 0.;
		temp_pdv->iptEnergyCost(sn_list[this_cn], ipt_eng);
		temp_pdv->f_eng -= ipt_eng;

		temp_req_p.erase(temp_req_p.begin() + next);
		req_sn_ptr.erase(req_sn_ptr.begin() + next);
	} while (temp_req_p.size());

	delete temp_pdv;
	return pdv_num;
}


template <class T>
bool Annealing<T>::calcInitGuess(const int& r_num, const int& pdv_num, const int& pop_num,
	vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> req_ps) {
	int len = req_sn_ptr.size();
	int len_sub_path = len / pdv_num;
	bool is_match = true;
	if (len % pdv_num != 0) is_match = false;

	vector<Point<T>> temp_req_p = req_ps;
	for (int i = 0; i < pop_num; i++) {
		//! The sub vector of @a clsts_list
		vector<int> req_sn_list;
		req_sn_list.reserve(len_sub_path);
		Point<T>* temp_p = new Point<T>();
		for (int j = 0; j < pdv_num; j++) {

			//! If not divisible, the last PDV assignment should be different
			if (j == pdv_num - 1 && !is_match) {
				break;
			}

			for (int k = 0; k < len_sub_path; k++) {
				//! Find the index randomly (the `num`th shortest)
				vector<double> d_list = temp_p->calcDist(temp_req_p);
				int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

				int sn_idx = -1;
				for (unsigned z = 0; z < sn_list.size(); z++) {
					if (sn_list[z].pos.isCoincide(temp_req_p[next])) {
						sn_idx = z;
						break;
					}
				}

				req_sn_list.push_back(sn_idx);
				temp_req_p.erase(temp_req_p.begin() + next);
			}

			this->saveGuessToTxt(i, j, req_sn_list.size(), req_sn_list);
			req_sn_list.clear();
			temp_p->setX(0.);
			temp_p->setY(0.);
		}

		if (!is_match) {
			do {
				//! Find the index randomly (the `num`th shortest)
				vector<double> d_list = temp_p->calcDist(temp_req_p);
				int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

				int sn_idx = -1;
				for (unsigned z = 0; z < sn_list.size(); z++) {
					if (sn_list[z].pos.isCoincide(temp_req_p[next])) {
						sn_idx = z;
						break;
					}
				}

				req_sn_list.push_back(sn_idx);
				temp_req_p.erase(temp_req_p.begin() + next);
			} while (temp_req_p.size());

			this->saveGuessToTxt(i, pdv_num - 1, req_sn_list.size(), req_sn_list);
			req_sn_list.clear();
			temp_p->setX(0.);
			temp_p->setY(0.);
		}

		//! Re-initialize @a temp_req_p for each population
		temp_req_p = req_ps;
		delete temp_p;
	}

	return is_match;
}

template <class T>
double Annealing<T>::fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> idx_list) {
	vector<Point<T>> this_flight_path;
	this_flight_path.reserve(idx_list.size());
	for (unsigned i = 0; i < idx_list.size(); i++) {
		this_flight_path.push_back(sn_list[idx_list[i]].pos);
	}

	unique_ptr<PDV<T>> pdv = make_unique<PDV<T>>();
	double charged_eng = 0., total_eng = 0.;

	do {
		vector<double> d_list = pdv->pos.calcDist(this_flight_path);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

		//! Implement inductive power transfer to the center node.
		int this_cn = -1;
		for (unsigned i = 0; i < sn_list.size(); i++) {
			if (sn_list[i].pos.isCoincide(this_flight_path[next])) {
				this_cn = i;
				break;
			}
		}

		if (pdv->calcEnergyCost(this_flight_path[next].calcDist(Point<T>()) / pdv->getPdvSpeed())
			+ pdv->calcEnergyCost(pdv->pos.calcDist(this_flight_path[next]) / pdv->getPdvSpeed()) > pdv->f_eng) {
			return -1;
		}

		pdv->updatePdvStatus(this_flight_path[next]);

		//! Update IPT energy cost
		double ipt_eng = 0.;
		pdv->iptEnergyCost(sn_list[this_cn], ipt_eng);
		pdv->f_eng -= ipt_eng;
		charged_eng += sn_list[this_cn].calcPackage() - 12.;
		total_eng += sn_list[this_cn].calcMaxEnergy();

		unique_ptr<Cluster<T>> clst = make_unique<Cluster<T>>();
		clst->center = &sn_list[this_cn].pos;
		clst->contains.clear();
		clst->contains.push_back(&sn_list[this_cn]);
		clst->assignEndNodes(sn_list);

		//! Implement center node acoustic power trannsfer.
		for (unsigned i = 1; i < clst->contains.size(); i++) {
			auto this_sn = clst->contains[i] - &sn_list[0];
			double temp_d = 0., temp_g = 0.;
			temp_d = sn_list[this_sn].pos.calcDist(*clst->center);
			temp_g = exp(-1. * pow(2 * PI * 47500, EFF_ACOUS) * temp_d * ALPHA_MAT);
			charged_eng += EFF_PIEZO * EFF_PIEZO * EFF_ACOUS2DC * temp_g * ACOUS_ENERGY_SEND;
		}

		this_flight_path.erase(this_flight_path.begin() + next);
	} while (this_flight_path.size());

	//! Return to home process
	pdv->updatePdvStatus(Point<T>());

	auto* funcs = new Funcs<double>();

	double wsn_e_fac = this->coeff_wsn_eng * funcs->tanhFunc(charged_eng / (total_eng * 0.9));		//! Wh
	double pdv_e_fac = this->coeff_pdv_eng * funcs->invTanhFunc((187. - pdv->f_eng) / 187.);			//! Wh

	double fitness = wsn_e_fac + pdv_e_fac;
	delete funcs;
	return fitness;
}

template <class T>
int Annealing<T>::getBestSol(const int& pdv_num) {
	int best = 0;
	if (this->pop == 1) return best;

	for (int i = 1; i < this->pop; i++) {
		if (this->tars_met[best][pdv_num] < this->tars_met[i][pdv_num]) best = i;
	}

	return best;
}

template <class T>
void Annealing<T>::initOneSol(const int& cur_pdv, vector<vector<int>>& idx_list, vector<Point<T>> req_ps, vector<SensorNode<T>> sn_list) {
	auto* funcs = new Funcs<double>();
	int r2 = funcs->getRandIndex(idx_list[cur_pdv].size() - 1);

	vector<double> d_list = sn_list[idx_list[cur_pdv][r2]].pos.calcDist(req_ps);
	int num = 0;
	int temp_size = req_ps.size();
	if (temp_size > this->max_num_r) num = funcs->getRandIndex(this->max_num_r - 1);
	else num = funcs->getRandIndex(temp_size - 1);

	if (!num) num++;

	unique_ptr<vector<double>> sort_ds = make_unique<vector<double>>(num + 1);
	partial_sort_copy(d_list.begin(), d_list.end(), sort_ds->begin(), sort_ds->end());
	auto it = find(d_list.begin(), d_list.end(), (*sort_ds)[num]);
	int next = distance(d_list.begin(), it);

	int sn_idx = -1;
	for (unsigned z = 0; z < sn_list.size(); z++) {
		if (sn_list[z].pos.isCoincide(req_ps[next])) {
			sn_idx = z;
			break;
		}
	}

	for (unsigned m = 0; m < idx_list.size(); m++) {
		for (unsigned n = 0; n < idx_list[m].size(); n++) {
			if (idx_list[m][n] == sn_idx) {
				swap(idx_list[cur_pdv][r2], idx_list[m][n]);
			}
		}
	}
}

template <class T>
vector<int> Annealing<T>::readGuessData(int pop_num, int pdv_num) {
	string fname = "../input/initial_guess/pop" + to_string(pop_num) + "_pdv" + to_string(pdv_num) + ".txt";
	vector<int> idx_list;
	fstream file;
	try {
		file.open(fname, fstream::in);
		if (!file.is_open()) throw runtime_error(strerror(errno));
	}
	catch (const std::exception& e) {
		cerr << e.what() << "\n";
	}

	string line;
	if (file.good()) getline(file, line);
	file.close();

	stringstream ss(line);
	idx_list.clear();
	for (int i; ss >> i;) {
		idx_list.push_back(i);
		if (ss.peek() == ',') ss.ignore();
	}

	return idx_list;
}

template <class T>
void Annealing<T>::saveSubPathToCsv(vector<SensorNode<T>> sn_list, vector<vector<int>> path_to_save) {
	for (unsigned i = 0; i < path_to_save.size(); i++) {
		string out_path = "../output/sub_path/sa_path" + to_string(i) + ".csv";
		fstream file;
		try {
			file.open(out_path, fstream::out);
			if (!file.is_open())
				throw runtime_error(strerror(errno));
		}
		catch (const exception& e) {
			cerr << e.what() << "\n";
		}
		file << "x_pos,y_pos" << endl;
		for (unsigned j = 0; j < path_to_save[i].size(); j++) {
			file << sn_list[path_to_save[i][j]].pos.getX() << ","
				<< sn_list[path_to_save[i][j]].pos.getY()
				<< endl;
		}

		file.close();
	}
}

template <class T>
void Annealing<T>::calcFinalPath(vector<SensorNode<T>>& sn_list, vector<SensorNode<T>*> candidates) {
	bool is_match = false;
	int pdv_num = this->calcOptPdvNum(sn_list, candidates, this->req_ps);
	//int pdv_num = 5;
	is_match = this->calcInitGuess(this->max_num_r, pdv_num, this->pop, sn_list, candidates, this->req_ps);

	auto* funcs = new Funcs<double>();
	auto start = chrono::high_resolution_clock::now();

	this->trails_idx.resize(pdv_num);
	this->trails_met.resize(pdv_num + 1);

	for (int i = 0; i < this->pop; i++) {
		this->tars_idx[i].resize(pdv_num);
		this->tars_met[i].resize(pdv_num);
		
		double tars_met_sum = -1.;
		for (int j = 0; j < pdv_num; j++) {
			this->tars_idx[i][j] = this->readGuessData(i, j);
			this->tars_met[i][j] = this->fitnessFunc(sn_list, this->tars_idx[i][j]);
			tars_met_sum += this->tars_met[i][j];
		}
		this->tars_met[i].push_back(tars_met_sum);
	}

	double temp = this->init_temp;
	for (int i = 0; i < this->pop; i++) {
		while (temp > this->min_temp) {
			bool can_finish = true;
			double trails_met_sum = -1.;

			this->trails_idx = this->tars_idx[i];
			for (int j = 0; j < pdv_num; j++) {
				this->initOneSol(j, this->trails_idx, this->req_ps, sn_list);
				this->trails_met[j] = this->fitnessFunc(sn_list, trails_idx[j]);
				if (this->trails_met[j] == -1) {
					can_finish = false;
					break;
				}
				trails_met_sum += this->trails_met[j];
			}
			if (!can_finish) {
				this->trails_met.clear();
				this->trails_met.resize(pdv_num + 1);
			}
			this->trails_met[pdv_num] = trails_met_sum;

			double delta_met = (this->trails_met[pdv_num] - this->tars_met[i][pdv_num])*500.;
			if (delta_met >= 0) {
				this->tars_idx[i] = this->trails_idx;
				this->tars_met[i] = this->trails_met;
				trails_met.clear();
				trails_met.resize(pdv_num + 1);
			}
			else {
				if (exp(delta_met / temp) > funcs->getRandFloat()) {
					this->tars_idx[i] = this->trails_idx;
					this->tars_met[i] = this->trails_met;
					this->trails_met.clear();
					this->trails_met.resize(pdv_num + 1);
				}
			}

			temp = this->temp_factor * temp;
		}

		temp = this->init_temp;
	}
	int best = this->getBestSol(pdv_num);

	auto stop = chrono::high_resolution_clock::now();
	delete funcs;
	this->alg_time = chrono::duration_cast<chrono::milliseconds>(stop - start).count();

	//this->saveSubPathToCsv(sn_list, this->tars_idx[best]);
	this->best_sol = this->tars_idx[best];
}

template <class T>
bool Annealing<T>::checkTask(vector<SensorNode<T>>& sn_list) {
	//! Before genetic algorithm process, update weights of sensor nodes according to inputs. 
	vector<SensorNode<T>*> requested_list;
	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V < sn_list[i].getMinVolt()) requested_list.push_back(&sn_list[i]);
	}

	if (requested_list.size() > this->min_req_num) {
		this->initParams(requested_list);
		this->calcFinalPath(sn_list, requested_list);
		return true;
	}

	return false;
}
