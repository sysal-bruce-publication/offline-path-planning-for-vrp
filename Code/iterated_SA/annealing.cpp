/*! @file annealing.cpp
 *
 *  @warning This is the internal cpp file of the ODP project.
 *  Do not use it directly in other code.
 *  Copyright (C) Qiuchen Qian, 2020
 *  Imperial College, London
 */


#include <cmath>
#include <valarray>
#include <chrono>
#include <string>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <random>
#include "pdv.h"
#include "annealing.h"

using namespace std::chrono;

 //! Coefficient for charged energy of WSN in fitness function
const int coeff_wsn_eng = 50;
//! Coefficient for flight energy cost of PDV in fitness function
const int coeff_pdv_eng = 25;
//! Coefficient for flight distance of PDV in fitness function
const int coeff_dist = 25;
//! The number of population
const int pop = 25;
//! The minimum number of a task
const int min_req_num = 20;
//! The number of sensor nodes in the range
const int max_num_r = 5;
//! Initial temperature
const double init_temp = 1.2e3;
//! Minimum temperature
const double min_temp = 1e-5;


/*! @fn			inline int getRandIndex(int limit)
 *  @brief			Get a random integer ranging from 0 to @a limit
 *  @param limit		upper limit of the generated integer
 *  @return			An integer
 */
inline int getRandIndex(int limit) {
	unsigned seed = static_cast<int> (std::chrono::system_clock::now().time_since_epoch().count());
	static std::default_random_engine generator_int(seed);

	std::uniform_int_distribution<int> distribution(0, limit);
	return distribution(generator_int);
}

inline float getRandDou() {
	mt19937_64 rng;
	//! initialize the random number generator with time-dependent seed
	uint64_t timeSeed = chrono::high_resolution_clock::now().time_since_epoch().count();
	seed_seq ss{ uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
	rng.seed(ss);
	//! initialize a uniform distribution between 0 and 1
	uniform_real_distribution<float> unif(0, 1);

	return unif(rng);
}

inline double tanhFunc(const double& x) {
	return 2. / (double)(1. + exp(-2. * x)) - 1.;
}

inline double invTanhFunc(const double& x) {
	return 1. - (2. / (double)(1. + exp(-2. * x)) - 1);
}

inline bool checkDuplicates(vector<int> source) {
	set<int> s(source.begin(), source.end());
	return s.size() != source.size();
}

/*! @fn			void delDuplicates(Cluster<T>& cluster)
 *  @brief			For single @a Cluster object, delete duplicates in @a contains
 *  @param cluster	@a Cluster object to be processed
 *  @tparam T		The type of data to present point coordinates.
 */
template <class T>
void delDuplicates(Cluster<T>& cluster) {
	for (unsigned i = 0; i < cluster.contains.size() - 1; i++) {
		for (unsigned j = i + 1; j < cluster.contains.size(); j++) {
			//! If found two same points, delete one
			if (cluster.contains[i] == cluster.contains[j]) {
				cluster.contains.erase(cluster.contains.begin() + j);
				j--;
			}
		}
	}
}

/*! @fn			void delDuplicates(vector<Point<T>>& p_list)
 *  @brief			For a vector of @a Point objects, delete duplicate points
 *  @param p_list		A vector of @a Points to be processed
 *  @tparam T		The type of data to present point coordinates.
 */
template <class T>
void delDuplicates(vector<Point<T>>& p_list) {
	for (unsigned i = 0; i < p_list.size() - 1; i++) {
		for (unsigned j = i + 1; j < p_list.size(); j++) {
			if (p_list[i].isCoincide(p_list[j])) {
				p_list.erase(p_list.begin() + j);
				j--;
			}
		}
	}
}

/*! @fn			void delDuplicates(vector<Point<T>>& p_list)
 *  @brief			For a vector of @a Point objects, delete duplicate points
 *  @param p_list		A vector of @a Points to be processed
 */
inline void delDuplicates(vector<int>& idx_list) {
	for (unsigned i = 0; i < idx_list.size() - 1; i++) {
		for (unsigned j = i + 1; j < idx_list.size(); j++) {
			if (idx_list[i] == idx_list[j]) {
				idx_list.erase(idx_list.begin() + j);
				j--;
			}
		}
	}
}

template <class T>
Annealing<T>::Annealing() {
	this->origin = new Point<T>();
}

template <class T>
Annealing<T>::~Annealing() {
	delete this->origin;

	//for (int i = 0; i < pop; i++) {
	//	this->tars_int[i].clear();
	//	this->trail_int[i].clear();
	//}

	//this->tars_met.clear();
	//this->trails_metric.clear();
}

template<class T>
void Annealing<T>::initParams(vector<SensorNode<T>*>& req_sn_ptr) {
	this->req_ps.resize(req_sn_ptr.size());
	for (unsigned i = 0; i < req_sn_ptr.size(); i++) {
		this->req_ps[i] = req_sn_ptr[i]->pos;
	}

	if (this->tars_idx.size() != 0) {
		for (int i = 0; i < pop; i++) {
			if (this->trails_idx[i].size() != 0) this->trails_idx[i].clear();
			if (this->tars_idx[i].size() != 0) this->tars_idx[i].clear();
		}
	}
	this->tars_idx.resize(pop);
	this->trails_idx.resize(pop);

	if (this->tars_met.size() != 0) {
		for (unsigned i = 0; i < tars_met.size(); i++) {
			this->tars_met.clear();
		}
		this->trails_met.clear();
	}
	this->tars_met.resize(pop);
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
	double ascent_t = 0., descent_t = 0.;
	double uwb_t = 0., gps_t = 0.;
	double rth_gps_t = 0., rth_uwb_t = 0.;
	double temp_ipt = 0.;
	double temp_asc = 0., temp_des = 0.;
	double temp_uwb = 0., temp_gps = 0.;
	double temp_rth_gps = 0., temp_rth_uwb = 0.;
	float rand_offset = 0.;

	PDV<T>* temp_pdv = new PDV<T>();

	// Take off from the BS
	temp_pdv->ascentEnergyCost(ascent_t, temp_asc);
	temp_pdv->updateEnergy(temp_asc);

	// Calculate descent energy in advance
	temp_pdv->descentEnergyCost(descent_t, temp_des);
	int cnt = 0;
	int pdv_num = 1;
	do {
		vector<double> d_list = temp_pdv->pos.calcDist(temp_req_p);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

		rand_offset = temp_pdv->getRandOffset();
		temp_pdv->gpsEnergyCost(rand_offset, temp_req_p[next], gps_t, temp_gps);
		temp_pdv->uwbEnergyCost(rand_offset, uwb_t, temp_uwb);

		rand_offset = temp_pdv->getRandOffset();
		temp_pdv->rthEnergyCost(rand_offset, temp_req_p[next], rth_gps_t, temp_rth_gps);
		temp_pdv->uwbEnergyCost(rand_offset, rth_uwb_t, temp_rth_uwb);

		int this_cn = -1;
		for (unsigned i = 0; i < sn_list.size(); i++) {
			if (sn_list[i].pos.isCoincide(req_sn_ptr[next]->pos)) {
				temp_pdv->iptEnergyCost(sn_list[i], temp_ipt);
				this_cn = i;
				break;
			}
		}

		if (temp_rth_gps + temp_rth_uwb + temp_gps + temp_uwb + temp_des + temp_ipt + 18.7 > temp_pdv->getPdvEnergy()) {
			pdv_num++;

			temp_pdv->resetPdvStatus();
			temp_pdv->updateEnergy(temp_asc);
			continue;
		}
		cnt++;
		temp_pdv->updatePos(temp_req_p[next]);
		temp_pdv->updateEnergy(temp_gps, temp_uwb);

		temp_pdv->updateEnergy(temp_ipt);

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
double Annealing<T>::calcFarNeighDist(vector<Point<T>> init_path) {
	Point<T>* temp_p = new Point<T>;
	double temp_d = 0.;
	do {
		vector<double> d_list = temp_p->calcDist(init_path);
		int next = distance(d_list.begin(), max_element(d_list.begin(), d_list.end()));

		temp_d += d_list[next];
		temp_p->setX(init_path[next].getX());
		temp_p->setY(init_path[next].getY());

		init_path.erase(init_path.begin() + next);
	} while (init_path.size());

	temp_d += temp_p->calcDist(0., 0.) + 40;
	delete temp_p;

	return temp_d;
}

template <class T>
double Annealing<T>::calcNearNeighDist(vector<Point<T>> init_path) {
	Point<T>* temp_p = new Point<T>;
	double temp_d = 0.;
	do {
		vector<double> d_list = temp_p->calcDist(init_path);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

		temp_d += d_list[next];
		temp_p->setX(init_path[next].getX());
		temp_p->setY(init_path[next].getY());

		init_path.erase(init_path.begin() + next);
	} while (init_path.size());

	temp_d += temp_p->calcDist(0., 0.) + 40;
	delete temp_p;

	return temp_d;
}

template <class T>
double Annealing<T>::fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> idx_list) {
	vector<Point<T>> this_flight_path;
	this_flight_path.reserve(idx_list.size());
	for (unsigned i = 0; i < idx_list.size(); i++) {
		this_flight_path.push_back(sn_list[idx_list[i]].pos);
	}

	//! Calculate the total distance of a trusted bad and good solution
	double far_d = calcFarNeighDist(this_flight_path);
	double near_d = calcNearNeighDist(this_flight_path);

	double ascent_t = 0., descent_t = 0.;
	double uwb_t = 0., gps_t = 0.;
	double rth_gps_t = 0., rth_uwb_t = 0.;
	double temp_ipt = 0.;
	double temp_asc = 0., temp_des = 0.;
	double temp_uwb = 0., temp_gps = 0.;
	double temp_rth_gps = 0., temp_rth_uwb = 0.;
	float rand_offset = 0.;

	unique_ptr<PDV<T>> pdv = make_unique<PDV<T>>();
	double charged_eng = 0., total_eng = 0.;

	// Take off from the BS
	pdv->ascentEnergyCost(ascent_t, temp_asc);
	pdv->updateEnergy(temp_asc);
	pdv->updateFlightDist(20.);

	// Calculate descent energy in advance
	pdv->descentEnergyCost(descent_t, temp_des);

	do {
		vector<double> d_list = pdv->pos.calcDist(this_flight_path);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

		rand_offset = pdv->getRandOffset();
		pdv->gpsEnergyCost(rand_offset, this_flight_path[next], gps_t, temp_gps);
		pdv->uwbEnergyCost(rand_offset, uwb_t, temp_uwb);

		rand_offset = pdv->getRandOffset();
		pdv->rthEnergyCost(rand_offset, this_flight_path[next], rth_gps_t, temp_rth_gps);
		pdv->uwbEnergyCost(rand_offset, rth_uwb_t, temp_rth_uwb);

		//! Implement inductive power transfer to the center node.
		int this_cn = -1;
		for (unsigned i = 0; i < sn_list.size(); i++) {
			if (sn_list[i].pos.isCoincide(this_flight_path[next])) {
				pdv->iptEnergyCost(sn_list[i], temp_ipt);
				this_cn = i;
				break;
			}
		}

		if (temp_rth_gps + temp_rth_uwb + temp_gps + temp_uwb + temp_des + temp_ipt + 18.7 > pdv->getPdvEnergy()) {
			return -1;
		}

		pdv->updateFlightDist(d_list[next]);
		pdv->updatePos(this_flight_path[next]);
		pdv->updateEnergy(temp_gps, temp_uwb);

		//! Update IPT energy cost
		pdv->updateEnergy(temp_ipt);
		charged_eng += sn_list[this_cn].calcPackage();
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
			temp_g = std::exp(-1. * pow(2 * PI * 47500, EFF_ACOUS) * temp_d * ALPHA_MAT);
			charged_eng += EFF_PIEZO * EFF_PIEZO * EFF_ACOUS2DC * temp_g * ACOUS_ENERGY_SEND;
		}

		this_flight_path.erase(this_flight_path.begin() + next);
	} while (this_flight_path.size());

	//! Return to home process
	rand_offset = pdv->getRandOffset();
	pdv->rthEnergyCost(rand_offset, pdv->pos, rth_gps_t, temp_rth_gps);
	pdv->uwbEnergyCost(rand_offset, rth_uwb_t, temp_rth_uwb);

	pdv->updateFlightDist(pdv->pos.calcDist(0., 0.));
	pdv->updateEnergy(temp_rth_gps, temp_rth_uwb);

	double wsn_e_fac = coeff_wsn_eng * tanhFunc(charged_eng / (total_eng * 0.9));					//! Wh
	double pdv_d_fac = coeff_dist * invTanhFunc((pdv->getPdvDistance() - near_d) / (far_d - near_d));	//! km
	double pdv_e_fac = coeff_pdv_eng * invTanhFunc((187. - pdv->getPdvEnergy()) / 187.);			//! Wh

	double fitness = wsn_e_fac + pdv_d_fac + pdv_e_fac;
	return fitness;
}

template <class T>
int Annealing<T>::getBestSol(const int& pdv_num) {
	int best = 0;
	if (pop == 1) return best;

	for (int i = 1; i < pop; i++) {
		if (this->tars_met[best][pdv_num] < this->tars_met[i][pdv_num]) best = i;
	}

	return best;
}

template <class T>
void Annealing<T>::initOneSol(const int& cur_pdv, vector<vector<int>>& idx_list, vector<Point<T>> req_ps, vector<SensorNode<T>> sn_list) {
	int r2 = getRandIndex(idx_list[cur_pdv].size() - 1);

	vector<double> d_list = sn_list[idx_list[cur_pdv][r2]].pos.calcDist(req_ps);
	int num = 0;
	int temp_size = req_ps.size();
	if (temp_size > max_num_r) num = getRandIndex(max_num_r - 1);
	else num = getRandIndex(temp_size - 1);

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
	is_match = this->calcInitGuess(max_num_r, pdv_num, pop, sn_list, candidates, this->req_ps);

	auto start = high_resolution_clock::now();

	this->trails_idx.resize(pdv_num);
	this->trails_met.resize(pdv_num + 1);

	for (int i = 0; i < pop; i++) {
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

	double temp = init_temp;
	for (int i = 0; i < pop; i++) {		
		while (temp > min_temp) {
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
				if (exp(delta_met / temp) > getRandDou()) {
					this->tars_idx[i] = this->trails_idx;
					this->tars_met[i] = this->trails_met;
					this->trails_met.clear();
					this->trails_met.resize(pdv_num + 1);
				}
			}

			temp = 0.98 * temp;
		}

		temp = init_temp;
	}
	int best = this->getBestSol(pdv_num);

	auto stop = high_resolution_clock::now();
	this->alg_time = duration_cast<milliseconds>(stop - start).count();

	this->saveSubPathToCsv(sn_list, this->tars_idx[best]);
}

template <class T>
bool Annealing<T>::checkTask(vector<SensorNode<T>>& sn_list) {
	//! Before genetic algorithm process, update weights of sensor nodes according to inputs. 
	vector<SensorNode<T>*> requested_list;
	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V < sn_list[i].getMinVolt()) requested_list.push_back(&sn_list[i]);
	}

	if (requested_list.size() > min_req_num) {
		this->initParams(requested_list);
		this->calcFinalPath(sn_list, requested_list);
		return true;
	}

	return false;
}
