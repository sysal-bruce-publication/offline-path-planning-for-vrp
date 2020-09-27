/*! @file blackhole.cpp
 *
 *  @warning This is the internal cpp file of the ODP project.
 *  Do not use it directly in other code. 
 *  Copyright (C) Qiuchen Qian, 2020
 *  Imperial College, London
 */

#include <cstdlib>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <valarray>
#include "blackhole.h"
#include "funcs.h"
#include "pdv.h"

using namespace std;

template <class T>
BlackHole<T>::BlackHole() {
	this->origin = new Point<T>();
}

template <class T>
BlackHole<T>::BlackHole(int w_wsn_eng, int w_pdv_eng, int w_pdv_dist, int n_gen,
		int n_pop, int n_ar, int n_req, int max_r) {
	this->origin = new Point<T>();
	this->alg_time = 0.;
	this->coeff_wsn_eng = w_wsn_eng;
	this->coeff_pdv_eng = w_pdv_eng;
	this->coeff_dist = w_pdv_dist;
	this->gen = n_gen;
	this->pop = n_pop;
	this->ar = n_ar;
	this->min_req_num = n_req;
	this->max_num_r = max_r;
}

template <class T>
BlackHole<T>::~BlackHole() {
	delete this->origin;
}

template <class T>
void BlackHole<T>::initParams(vector<SensorNode<T>*> req_sn_ptr) {
	this->req_ps.resize(req_sn_ptr.size());
	for (unsigned i = 0; i < req_sn_ptr.size(); i++) {
		this->req_ps[i] = req_sn_ptr[i]->pos;
	}

	if (this->tars_idx.size() != 0) {
		for (int i = 0; i < this->pop; i++) {
			if (this->tars_idx[i].size() != 0) this->tars_idx[i].clear();
		}
	}
	this->tars_idx.resize(this->pop);

	if (this->tars_metric.size() != 0) this->tars_metric.clear();
	this->tars_metric.resize(this->pop);
}

template <class T>
int BlackHole<T>::calcOptPdvNum(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> temp_req_p) {
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
void BlackHole<T>::saveGuessToTxt(int pop_num, int pdv_num_txt, int sn_num, vector<int> path_to_save) {
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
bool BlackHole<T>::calcInitGuess(const int& r_num, const int& pdv_num, const int& pop_num, 
		vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> req_ps) {
	int len = req_sn_ptr.size();
	int len_sub_path = len / pdv_num;
	bool is_match = true;
	if (len % pdv_num != 0) is_match = false;

	vector<Point<T>> temp_req_p = req_ps;
	auto* funcs = new Funcs<double>();

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
				int num = 0;
				int temp_size = temp_req_p.size();
				if (temp_size > r_num) num = funcs->getRandIndex(r_num - 1);
				else num = funcs->getRandIndex(temp_size - 1);

				unique_ptr<vector<double>> sort_ds = make_unique<vector<double>>(num + 1);
				partial_sort_copy(d_list.begin(), d_list.end(), sort_ds->begin(), sort_ds->end());
				auto it = find(d_list.begin(), d_list.end(), (*sort_ds)[num]);
				int next = distance(d_list.begin(), it);

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
				int num = 0;
				int temp_size = temp_req_p.size();
				if (temp_size > r_num) num = funcs->getRandIndex(r_num - 1);
				else num = funcs->getRandIndex(temp_size - 1);

				unique_ptr<vector<double>> sort_ds = make_unique<vector<double>>(num + 1);
				partial_sort_copy(d_list.begin(), d_list.end(), sort_ds->begin(), sort_ds->end());
				auto it = find(d_list.begin(), d_list.end(), (*sort_ds)[num]);
				int next = distance(d_list.begin(), it);

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
	delete funcs;
	return is_match;
}

template <class T>
void BlackHole<T>::initOneSol(const int& pdv_num, vector<vector<int>>& idx_list, vector<Point<T>> req_ps,
		vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr) {
	if (idx_list.size()) {
		for (int i = 0; i < pdv_num; i++) {
			idx_list[i].clear();
		}
	}
	idx_list.resize(pdv_num);

	int len = req_sn_ptr.size();
	int len_sub_path = len / pdv_num;
	bool is_match = true;
	if (len % pdv_num != 0) is_match = false;

	vector<Point<T>> temp_req_p = req_ps;
	vector<int> req_sn_list;
	req_sn_list.reserve(len_sub_path);
	Point<T>* temp_p = new Point<T>();

	auto* funcs = new Funcs<double>();

	for (int j = 0; j < pdv_num; j++) {
		//! If not divisible, the last PDV assignment should be different
		if (j == pdv_num - 1 && !is_match) {
			break;
		}

		for (int k = 0; k < len_sub_path; k++) {
			//! Find the index randomly (the `num`th shortest)
			vector<double> d_list = temp_p->calcDist(temp_req_p);
			int num = 0;
			int temp_size = temp_req_p.size();
			if (temp_size > this->max_num_r) num = funcs->getRandIndex(this->max_num_r - 1);
			else num = funcs->getRandIndex(temp_size - 1);

			unique_ptr<vector<double>> sort_ds = make_unique<vector<double>>(num + 1);
			partial_sort_copy(d_list.begin(), d_list.end(), sort_ds->begin(), sort_ds->end());
			auto it = find(d_list.begin(), d_list.end(), (*sort_ds)[num]);
			int next = distance(d_list.begin(), it);

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

		idx_list[j] = req_sn_list;

		req_sn_list.clear();
		temp_p->setX(0.);
		temp_p->setY(0.);
	}

	if (!is_match) {
		do {
			//! Find the index randomly (the `num`th shortest)
			vector<double> d_list = temp_p->calcDist(temp_req_p);
			int num = 0;
			int temp_size = temp_req_p.size();
			if (temp_size > this->max_num_r) num = funcs->getRandIndex(this->max_num_r - 1);
			else num = funcs->getRandIndex(temp_size - 1);

			unique_ptr<vector<double>> sort_ds = make_unique<vector<double>>(num + 1);
			partial_sort_copy(d_list.begin(), d_list.end(), sort_ds->begin(), sort_ds->end());
			auto it = find(d_list.begin(), d_list.end(), (*sort_ds)[num]);
			int next = distance(d_list.begin(), it);

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

		idx_list[pdv_num - 1] = req_sn_list;
		req_sn_list.clear();
		temp_p->setX(0.);
		temp_p->setY(0.);
	}

	delete temp_p;
	delete funcs;
}

template <class T>
double BlackHole<T>::fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> idx_list) {
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
vector<int> BlackHole<T>::readGuessData(int pop_num, int pdv_num) {
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
void BlackHole<T>::saveSubPathToCsv(vector<SensorNode<T>> sn_list, vector<vector<int>> path_to_save) {
	for (unsigned i = 0; i < path_to_save.size(); i++) {
		string out_path = "../output/sub_path/bh_path" + to_string(i) + ".csv";
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
void BlackHole<T>::attraction(int cur_gen, int cur_pdv, const int& bh_num,
		vector<SensorNode<T>> sn_list, vector<Point<T>> req_ps, vector<vector<int>>& tar_vec) {
	double delta_x = 0.;
	double delta_y = 0.;
	double rand_fac = 0.;
	auto* funcs = new Funcs<double>();
	for (unsigned z = 0; z < tar_vec[cur_pdv].size(); z++) {
		if (funcs->getRandIndex(100) <= this->ar) continue;
		rand_fac = funcs->getRandFloat();
		delta_x = rand_fac * (sn_list[tar_vec[cur_pdv][z]].pos.getX() - sn_list[this->tars_idx[bh_num][cur_pdv][z]].pos.getX());
		delta_y = rand_fac * (sn_list[tar_vec[cur_pdv][z]].pos.getY() - sn_list[this->tars_idx[bh_num][cur_pdv][z]].pos.getY());
		unique_ptr<Point<T>> temp_p = make_unique<Point<T>>(sn_list[tar_vec[cur_pdv][z]].pos.getX() + delta_x, 
			sn_list[tar_vec[cur_pdv][z]].pos.getY() + delta_y);
		vector<double> d_list = temp_p->calcDist(req_ps);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));
		for (unsigned m = 0; m < sn_list.size(); m++) {
			if (req_ps[next].isCoincide(sn_list[m].pos)) {
				next = m;
				break;
			}
		}

		for (unsigned m = 0; m < tar_vec.size(); m++) {
			for (unsigned n = 0; n < tar_vec[m].size(); n++) {
				if (m == cur_pdv && n == z) continue;
				if (tar_vec[m][n] == next) {
					swap(tar_vec[cur_pdv][z], tar_vec[m][n]);
				}
			}
		}
	}
	delete funcs;
}

template <class T>
void BlackHole<T>::calcFinalPath(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> candidates) {
	bool is_match = false;
	int pdv_num = this->calcOptPdvNum(sn_list, candidates, this->req_ps);
	is_match = this->calcInitGuess(this->max_num_r, pdv_num, this->pop, sn_list, candidates, this->req_ps);
	
	auto start = chrono::high_resolution_clock::now();
	auto* funcs = new Funcs<double>();
	//! Initialization
	for (int i = 0; i < this->pop; i++) {
		this->tars_idx[i].resize(pdv_num);
		this->tars_metric[i].resize(pdv_num + 1);

		double tar_met_sum = 0.;
		for (int j = 0; j < pdv_num; j++) {
			this->tars_idx[i][j] = this->readGuessData(i, j);
			this->tars_metric[i][j] = this->fitnessFunc(sn_list, this->tars_idx[i][j]);
			tar_met_sum += this->tars_metric[i][j];
		}
		this->tars_metric[i][pdv_num] = tar_met_sum;
	}

	int bh_idx = 0;
	for (int i = 1; i < this->pop; i++) {
		if (this->tars_metric[i][pdv_num] > this->tars_metric[bh_idx][pdv_num]) bh_idx = i;
	}

	for (int i = 0; i < this->gen; i++) {
		double sum_fitness = 0., bh_r = 0.;
		for (int j = 0; j < this->pop; j++) {
			// @a sum_fitness includes the fitness metric of BH
			if (j == bh_idx) {
				sum_fitness += this->tars_metric[j][pdv_num];
				continue;			//! Black hole
			}

			double new_tar_sum = 0.;
			for (int k = 0; k < pdv_num; k++) {
				this->attraction(j, k, bh_idx, sn_list, this->req_ps, this->tars_idx[j]); 
				this->tars_metric[j][k] = this->fitnessFunc(sn_list, this->tars_idx[j][k]);
				new_tar_sum += this->tars_metric[j][k];
			}
			this->tars_metric[j][pdv_num] = new_tar_sum;
			sum_fitness += new_tar_sum;
		}

		bh_idx = 0;
		for (int i = 1; i < this->pop; i++) {
			if (this->tars_metric[i][pdv_num] > this->tars_metric[bh_idx][pdv_num]) bh_idx = i;
		}

		if (i == this->gen - 1) break;

		bh_r = this->tars_metric[bh_idx][pdv_num] / sum_fitness;
		for (int j = 0; j < this->pop; j++) {
			if (j == bh_idx) continue;

			if (funcs->getRandFloat() < bh_r) {
				this->initOneSol(pdv_num, this->tars_idx[j], this->req_ps, sn_list, candidates);

				double new_sol_met_sum = 0.;
				for (int k = 0; k < pdv_num; k++) {
					this->tars_metric[j][k] = this->fitnessFunc(sn_list, this->tars_idx[j][k]);
					new_sol_met_sum += this->tars_metric[j][k];
				}
				this->tars_metric[j][pdv_num] = new_sol_met_sum;
			}
		}
	}

	auto stop = chrono::high_resolution_clock::now();
	this->alg_time = chrono::duration_cast<chrono::milliseconds>(stop - start).count();
	delete funcs;

	//this->saveSubPathToCsv(sn_list, this->tars_idx[bh_idx]);
	this->best_sol = this->tars_idx[bh_idx];
}

template <class T>
bool BlackHole<T>::checkTask(vector<SensorNode<T>>& sn_list) {
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