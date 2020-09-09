/*! @file	pdv.cpp
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


#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include "pdv.h"

#define N_RF2DC 0.5

using namespace std;

template <class T>
PDV<T>::PDV() {
	this->pos = Point<T>();
	this->flight_time_ = 0.;
	this->pdv_energy_ = 187.;
	this->flight_distance_ = 0.;
}

template <class T>
PDV<T>::~PDV() {
	this->pos = Point<T>();
	this->flight_time_ = 0.;
	this->pdv_energy_ = 187.;
	this->flight_distance_ = 0.;
}

template <class T>
void PDV<T>::resetPdvStatus() {
	this->pos.setX(0.);
	this->pos.setY(0.);
	this->flight_time_ = 0.;
	this->pdv_energy_ = 187.;
	this->flight_distance_ = 0.;
}

template <class T>
float PDV<T>::getRandOffset() const {
	srand(static_cast<unsigned>(time(0)));
	float low = -3.;
	float high = 3.;
	return low + static_cast<float>(rand()) / static_cast<float>(RAND_MAX / (high - low));
}

template <class T>
bool PDV<T>::taskCheck(vector<SensorNode<T>>& sn_list) {
	vector<SensorNode<T>*> requested_list;
	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V < sn_list[i].getMinVolt()) {
			requested_list.push_back(&sn_list[i]);
		}
	}
	if (requested_list.size() > this->min_charge_num) {
		return true;
	}

	return false;
}

template <class T>
void PDV<T>::ascentEnergyCost(double& t, double& e) {
	t = this->flight_altitude / (this->max_ascent_speed * 0.9);
	e = this->calcEnergyCost(t);
}

template <class T>
void PDV<T>::descentEnergyCost(double& t, double& e) {
	t = this->flight_altitude / (this->max_descent_speed * 0.9);
	e = this->calcEnergyCost(t);
}

template <class T>
void PDV<T>::uwbEnergyCost(const float& rand_offset, double& t, double& e) {
	t = (static_cast<float>(10) - rand_offset) / (this->max_gps_speed * 0.8);
	e = this->calcEnergyCost(t);
}

template <class T>
void PDV<T>::gpsEnergyCost(const float& rand_offset, const Point<T>& next_p, double& t, double& e) {
	double total_d = this->pos.calcDist(next_p) - (static_cast<float>(10) - rand_offset);
	t = total_d / (this->max_gps_speed);
	e = calcEnergyCost(t);
}

template <class T>
void PDV<T>::rthEnergyCost(const float& rand_offset, Point<T> next_p, double& t, double& e) {
	Point<T>* origin = new Point<T>();
	double total_d = next_p.calcDist(*origin) - (static_cast<float>(10) - rand_offset);
	delete origin;
	t = total_d / (this->max_gps_speed);
	e = calcEnergyCost(t);
}

template <class T>
void PDV<T>::iptEnergyCost(const SensorNode<T>& next_sn, double& e) {
	double factor, pckt;
	factor = 1. / (N_RF2DC * 3600);			/*!< Convert J to Wh */
	pckt = 0.5 * next_sn.getCapacitance() * (pow(next_sn.getMaxVolt(), 2) - pow(next_sn.SC_V, 2));
	e = factor * pckt;
}

template <class T>
float PDV<T>::flightSimulation(double& charged_e, double& pdv_t, vector<SensorNode<T>>& sn_list, vector<Point<T>>& path) {
	double ascent_t = 0., descent_t = 0.;
	double uwb_t = 0., gps_t = 0.;
	double rth_gps_t = 0., rth_uwb_t = 0.;
	double ipt_t = 0., temp_ipt = 0.;
	double temp_asc = 0., temp_des = 0.;
	double temp_uwb = 0., temp_gps = 0.;
	double temp_rth_gps = 0., temp_rth_uwb = 0.;
	float rand_offset = 0.;

	int charged = 0;
	int path_len = path.size();
	charged_e = 0.;
	pdv_t = 0.;

	// Take off from the BS
	this->ascentEnergyCost(ascent_t, temp_asc);
	this->updateFlightTime(ascent_t);
	this->updateEnergy(temp_asc);
	this->updateFlightDist(20.);

	// Calculate descent energy in advance
	this->descentEnergyCost(descent_t, temp_des);

	do {			//£¡ SJN method
		vector<double> d_list = this->pos.calcDist(path);
		int next = distance(d_list.begin(), min_element(d_list.begin(), d_list.end()));

		rand_offset = this->getRandOffset();
		this->gpsEnergyCost(rand_offset, path[next], gps_t, temp_gps);
		this->uwbEnergyCost(rand_offset, uwb_t, temp_uwb);

		rand_offset = this->getRandOffset();
		this->rthEnergyCost(rand_offset, path[next], rth_gps_t, temp_rth_gps);
		this->uwbEnergyCost(rand_offset, rth_uwb_t, temp_rth_uwb);

		int this_cn = -1;
		for (unsigned i = 0; i < sn_list.size(); i++) {
			if (sn_list[i].pos.isCoincide(path[next])) {
				this->iptEnergyCost(sn_list[i], temp_ipt);
				this_cn = i;
				break;
			}
		}

		if (temp_rth_gps + temp_rth_uwb + temp_gps + temp_uwb + temp_des + temp_ipt + 18.7 > this->getPdvEnergy()) {
			break;
		}

		this->updateFlightTime(gps_t, uwb_t);
		pdv_t += (gps_t + uwb_t);
		this->updateFlightDist(d_list[next]);
		this->updatePos(path[next]);
		this->updateEnergy(temp_gps, temp_uwb);

		this->updateEnergy(temp_ipt);
		this->updateFlightTime(0.05);
		pdv_t += 0.05;
		charged_e += sn_list[this_cn].calcPackage();
		sn_list[this_cn].SC_V = sn_list[this_cn].getMaxVolt();
		sn_list[this_cn].updateEnergy(sn_list[this_cn].SC_E);
		sn_list[this_cn].updateWeight(sn_list[this_cn].SC_V, sn_list[this_cn].weight);
		charged++;

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
			charged_e += EFF_PIEZO * EFF_PIEZO * EFF_ACOUS2DC * temp_g * ACOUS_ENERGY_SEND;
		}

		path.erase(path.begin() + next);

	} while (path.size());

	//! Return to home process
	rand_offset = this->getRandOffset();
	this->rthEnergyCost(rand_offset, this->pos, rth_gps_t, temp_rth_gps);
	this->uwbEnergyCost(rand_offset, rth_uwb_t, temp_rth_uwb);

	this->updateFlightDist(this->pos.calcDist(0., 0.));
	this->updateFlightTime(rth_gps_t, rth_uwb_t);
	pdv_t += (rth_gps_t + rth_uwb_t);
	this->updatePos(Point<T>());
	this->updateEnergy(temp_rth_gps, temp_rth_uwb);

	//! Descent process
	this->updateFlightDist(20);
	this->updateFlightTime(descent_t);
	pdv_t += descent_t;
	this->updateEnergy(temp_des);

	return static_cast <float> ((charged /(float) path_len) * 100);
}
