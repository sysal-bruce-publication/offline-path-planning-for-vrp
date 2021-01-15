/*! @file	genetic.cpp
 *
 *  @warning This is the internal header of the ODP project.
 *  Do not use it directly in other code. Please note that this file
 *  is based on the open source code from
 *  <a href="https://github.com/achu6393/dynamicWeightedClustering">
 *	dynamicWeightedClustering
 *  </a>
 *  Copyright (C) Qiuchen Qian, 2020
 *  Imperial College, London
 */

#include <chrono>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include "pdv.h"
#include "funcs.h"
#include "genetic.h"

using namespace std;

template <class T>
Genetic<T>::Genetic() {
	this->origin = new Point<T>();
	this->alg_time = 0.;
}

template <class T>
Genetic<T>::Genetic(int w_wsn_eng, int w_pdv_eng, int w_pdv_dist, int n_gen, 
		int n_pop, int n_cr, int n_req, int max_r) {
	this->origin = new Point<T>();
	this->alg_time = 0.;
	this->coeff_wsn_eng = w_wsn_eng;
	this->coeff_pdv_eng = w_pdv_eng;
	this->coeff_dist = w_pdv_dist;
	this->gen = n_gen;
	this->pop = n_pop;
	this->cr = n_cr;
	this->min_req_num = n_req;
	this->max_num_r = max_r;
}

template <class T>
Genetic<T>::~Genetic() {
	delete this->origin;
}

template<class T>
void Genetic<T>::initParams(vector<SensorNode<T>*>& req_sn_ptr) {
	this->req_ps.resize(req_sn_ptr.size());
	for (unsigned i = 0; i < req_sn_ptr.size(); i++) {
		this->req_ps[i] = req_sn_ptr[i]->pos;
	}

	if (this->trail_int.size() != 0) {
		for (int i = 0; i < this->pop; i++) {
			if (this->trail_int[i].size() != 0) this->trail_int[i].clear();
			if (this->tars_int[i].size() != 0) this->tars_int[i].clear();
		}
	}
	this->tars_int.resize(this->pop);
	this->trail_int.resize(this->pop);

	if (this->targets_metric.size() != 0) {
		this->targets_metric.clear();
		this->trails_metric.clear();
	}
	this->targets_metric.resize(this->pop);
	this->trails_metric.resize(this->pop);
	
	this->alg_time = 0.;
}

template <class T>
void Genetic<T>::saveGuessToTxt(int pop_num, int pdv_num_txt, int sn_num, vector<int> path_to_save) {
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
int Genetic<T>::calcOptPdvNum(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> temp_req_p) {
	PDV<T>* temp_pdv = new PDV<T>();

	int pdv_num = 1;
	do {
		if (temp_pdv->f_eng <= 20) {
			pdv_num++;
			temp_pdv->resetPdvStatus();
			continue;
		}

		vector<double> d_list = temp_pdv->pos.calcDist(temp_req_p);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

		int this_cn = -1;
		for (unsigned i = 0; i < sn_list.size(); i++) {
			if (sn_list[i].pos.isCoincide(req_sn_ptr[next]->pos)) {
				this_cn = i;
				break;
			}
		}

		double ipt_eng = 0.;
		temp_pdv->iptEnergyCost(sn_list[this_cn], ipt_eng);

		if (temp_pdv->calcEnergyCost(temp_req_p[next].calcDist(Point<T>()) / temp_pdv->getPdvSpeed())
			+ temp_pdv->calcEnergyCost(temp_pdv->pos.calcDist(temp_req_p[next]) / temp_pdv->getPdvSpeed())
			+ ipt_eng + 20 > temp_pdv->f_eng) {

			pdv_num++;
			temp_pdv->resetPdvStatus();
			continue;
		}

		temp_pdv->updatePdvStatus(temp_req_p[next]);
		temp_pdv->f_eng -= ipt_eng;

		temp_req_p.erase(temp_req_p.begin() + next);
		req_sn_ptr.erase(req_sn_ptr.begin() + next);
	} while (temp_req_p.size());

	delete temp_pdv;
	return pdv_num;
}


template <class T>
bool Genetic<T>::calcInitGuess(const int& r_num, const int& pdv_num, const int& pop_num, 
		vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> req_ps) {
	int len = req_sn_ptr.size();
	int len_sub_path = len / pdv_num;
	bool is_match = true;
	if (len % pdv_num != 0) is_match = false;
	
	auto* funcs = new Funcs<double>();

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
void Genetic<T>::crossover(int cross_ratio, int pop_num, vector<vector<vector<int>>> tar_vec, vector<vector<vector<int>>>& trail_vec, vector<SensorNode<T>> sn_list) {
	int r1 = -1;
	auto* funcs = new Funcs<double>();
	for (int i = 0; i < pop_num; i++) {
		r1 = funcs->getRandIndex(pop_num - 1);
		trail_vec[i] = tar_vec[r1];

		// for each node in trail vector
		for (unsigned j = 0; j < trail_vec[i].size(); j++) {
			for (unsigned k = 0; k < trail_vec[i][j].size(); k++) {
				if (funcs->getRandIndex(100) > cross_ratio) {

					//! Find the index randomly (the `num`th shortest)
					// Calculate the distance between all other nodes and current node [i][j][k]
					vector<double> d_list;
					for (unsigned m = 0; m < trail_vec[i].size(); m++) {
						for (unsigned n = 0; n < trail_vec[i][m].size(); n++) {
							d_list.push_back(sn_list[trail_vec[i][j][k]].pos.calcDist(sn_list[trail_vec[i][m][n]].pos));
						}
					}
					int num = funcs->getRandIndex(9);
					if (trail_vec[i][j].size() < 10) num = funcs->getRandIndex(trail_vec[i][j].size() - 1);
					
					unique_ptr<vector<double>> sort_ds = make_unique<vector<double>>(num + 1);
					partial_sort_copy(d_list.begin(), d_list.end(), sort_ds->begin(), sort_ds->end());
					auto it = find(d_list.begin(), d_list.end(), (*sort_ds)[num]);
					int next = distance(d_list.begin(), it);

					int temp_pdv_id = 0, temp_num = 0, temp_pop_size = 0;
					for (unsigned m = 0; m < tar_vec[r1].size(); m++) {
						temp_pop_size += tar_vec[r1][m].size();
						if (next == temp_pop_size) {
							temp_pdv_id = m;
							temp_num = tar_vec[r1][m].size() - 1;
							break;
						}

						if (next < temp_pop_size) {
							temp_pdv_id = m;
							int over_pop_size = 0;
							if (!m) temp_num = next; break;
							for (int n = 0; n < m; n++) {
								over_pop_size += n * tar_vec[r1][n].size();
								if (over_pop_size >= temp_pop_size) {
									temp_num = temp_pop_size - (over_pop_size - n * tar_vec[r1][n].size()) + 1;
									break;
								}
							}
							if (!temp_num) cerr << "Cannot find index number !" << endl; return;
							break;
						}
					}
					// OLD VERSION - IGNORE IT
					//if (temp_pdv_id >= tar_vec[i].size() - 1) temp_pdv_id = tar_vec[i].size() - 1;
					//int temp_num = next % trail_vec[i][0].size();
					//if (!is_match) {
					//	if (temp_pdv_id == trail_vec[i].size()) {
					//		temp_pdv_id -= 1;
					//		temp_num += trail_vec[i][0].size();
					//	}
					//}
					
					swap(trail_vec[i][j][k], trail_vec[i][temp_pdv_id][temp_num]);
				}
			}
		}
	}
	delete funcs;
}

template <class T>
double Genetic<T>::fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> idx_list) {
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

		if (pdv->calcEnergyCost(this_flight_path[next].calcDist(Point<T>()) / pdv->getPdvSpeed()) + 
			pdv->calcEnergyCost(pdv->pos.calcDist(this_flight_path[next]) / pdv->getPdvSpeed()) > pdv->f_eng) {
			return -1000.;
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
int Genetic<T>::getBestSol(const int& pdv_num) {
	int best = 0;

	for (int i = 1; i < this->pop; i++) {
		if (this->targets_metric[best][pdv_num] < this->targets_metric[i][pdv_num]) best = i;
	}

	return best;
}

template <class T>
vector<int> Genetic<T>::readGuessData(int case_num, int pop_num, int pdv_num) {
	string fname = "../input/initial_guess/case" + to_string(case_num) + "/pop" + to_string(pop_num) + "pdv" + to_string(pdv_num) + ".txt";
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
void Genetic<T>::calcFinalPath(int case_num, vector<SensorNode<T>>& sn_list, vector<SensorNode<T>*> candidates) {
	//bool is_match = false;
	int pdv_num = this->calcOptPdvNum(sn_list, candidates, this->req_ps);
	//is_match = this->calcInitGuess(this->max_num_r, pdv_num, this->pop, sn_list, candidates, this->req_ps);

	auto start = chrono::high_resolution_clock::now();
	for (int i = 0; i < this->pop; i++) {
		this->tars_int[i].resize(pdv_num);
		this->trail_int[i].resize(pdv_num);
		this->targets_metric[i].resize(pdv_num + 1);
		this->trails_metric[i].resize(pdv_num + 1);
		for (int j = 0; j < pdv_num; j++) {
			this->tars_int[i][j] = this->readGuessData(case_num, i, j);
			this->targets_metric[i][j] = -1.;
			this->trails_metric[i][j] = -1.;
		}
		this->targets_metric[i][pdv_num] = -1.;
		this->trails_metric[i][pdv_num] = -1.;
	}

	for (int i = 0; i < this->gen; i++) {
		this->crossover(this->cr, this->pop, this->tars_int, this->trail_int, sn_list);

		for (int j = 0; j < this->pop; j++) {
			double targe_met_sum = -1.;
			double trail_met_sum = -1.;

			for (int k = 0; k < pdv_num; k++) {
				this->targets_metric[j][k] = this->fitnessFunc(sn_list, this->tars_int[j][k]);
				targe_met_sum += this->targets_metric[j][k];
				this->trails_metric[j][k] = this->fitnessFunc(sn_list, this->trail_int[j][k]);
				trail_met_sum += this->trails_metric[j][k];
			}
			this->targets_metric[j][pdv_num] = targe_met_sum;
			this->trails_metric[j][pdv_num] = trail_met_sum;

			//! Selection
			if (trail_met_sum > targe_met_sum) {
				this->tars_int[j] = this->trail_int[j];
				this->targets_metric[j] = this->trails_metric[j];
			}
		}
	}
	int best = this->getBestSol(pdv_num);

	auto stop = chrono::high_resolution_clock::now();
	this->alg_time = chrono::duration_cast<chrono::milliseconds>(stop - start).count();

	//this->saveSubPathToCsv(sn_list, this->tars_int[best]);
	this->best_sol = this->tars_int[best];
}

template <class T>
bool Genetic<T>::checkTask(int case_num, vector<SensorNode<T>>& sn_list) {
	//! Before genetic algorithm process, update weights of sensor nodes according to inputs. 
	vector<SensorNode<T>*> requested_list;
	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V < sn_list[i].getMinVolt()) requested_list.push_back(&sn_list[i]);
	}

	if (requested_list.size() > this->min_req_num) {
		this->initParams(requested_list);
		this->calcFinalPath(case_num, sn_list, requested_list);
		return true;
	}

	return false;
}