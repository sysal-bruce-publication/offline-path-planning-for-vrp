/*! @file sensornode.cpp
 *
 *  @warning This is the internal source of the ODP project.
 *  Do not use it directly in other code. Please note that this file
 *  is based on the open source code from
 *  <a href="https://github.com/achu6393/dynamicWeightedClustering">
 *	dynamicWeightedClustering
 *  </a>
 *  Copyright (C) Qiuchen Qian, 2020
 *  Imperial College, London
 */

#include <iostream>
#include <algorithm>
#include "sensornode.h"

template <class T>
SensorNode<T>::SensorNode() {
	this->pos = Point<T>();
	this->SC_V = 3.4;
	this->SC_E = 17.34;
	this->weight = 10;
	this->fails = 0;
	this->p_sensor_type = true;
}

template <class T>
SensorNode<T>::SensorNode(const float& x, const float& y, const double& v, int w, bool p_type) {
	//£¡ Common public member variables
	this->pos = Point<T>(x, y);
	this->SC_V = v;
	this->weight = w;
	this->p_sensor_type = p_type;
	this->sense_cycle = 2e-3;
	this->comm_cycle = 0.5;
	this->fails = 0;

	if (p_type) {
		//! Specific public member variables
		this->SC_C = 3;
		this->time_to_change = 1;
		this->time_to_reset = 1;

		//! Protected member variables
		this->SC_Vmax = 5.;
		this->SC_Vmin = 3.5;
		this->SC_Vcritical = 3.3;
		this->I_sense = 1.2e-3;
		this->I_idle = 4e-6;
		this->idle_cycle = 9.498;
	}
	else {
		//! Public member variables
		this->SC_C = 6;
		this->time_to_change = 10;
		this->time_to_reset = 10;

		//! Protected member variables
		this->SC_Vmax = 2.5;
		this->SC_Vmin = 1.75;
		this->SC_Vcritical = 1.5;
		this->I_sense = 5.4e-6;
		this->I_idle = 4e-9;
		this->idle_cycle = 99.498;
	}

	this->SC_E = 0.5 * this->SC_C * this->SC_V * this->SC_V;
}

template <class T>
SensorNode<T>::~SensorNode() {}

template <class T>
void SensorNode<T>::updateVolt(double& v) {
	double temp_v = 2 * this->SC_E / this->SC_C;
	v = sqrt(temp_v);
}

template <class T>
void SensorNode<T>::updateEnergy(double& e) {
	e = 0.5 * this->SC_C * this->SC_V * this->SC_V;
}

template <class T>
void SensorNode<T>::updateEnergy(const double& t, double& e) {
	//! t - unit: s
	int cycles = static_cast<int>(floor(t / (3 * this->time_to_reset)));

	e -= (cycles * this->sense_cycle * this->V_sense * this->I_sense
		+ cycles * this->idle_cycle * this->I_idle * this->V_sense
		+ cycles * this->comm_cycle * 2.45e-3);
}

template <class T>
void SensorNode<T>::updateWeight(const double& v, int& w) {
	if (this->SC_Vmax - v <= 0.25) w = 10;	/*!< Be considered as full voltage */
	else if (this->SC_Vmax - v > 0.25 && this->SC_Vmax - v <= 0.5) w = 9;
	else if (this->SC_Vmax - v > 0.5 && this->SC_Vmax - v <= 0.75) w = 8;
	else if (this->SC_Vmax - v > 0.75 && this->SC_Vmax - v <= 1.)  w = 7;
	else if (this->SC_Vmax - v > 1. && this->SC_Vmax - v <= 1.25)  w = 6;
	else if (this->SC_Vmax - v > 1.25 && this->SC_Vmax - v <= 1.5) w = 5;
	else if (this->SC_Vmax - v > 1.5 && this->SC_Vmax - v <= 1.75) w = 4;
	else w = 3;						/*!< Be considered as the lowest voltage */
}

template <class T>
double SensorNode<T>::acousTransfer(const double& d) {
	//! Acoustic frequency is 4.75 kHz, the unit of d is meter.
	double g_coeff = exp(-1. * pow(2 * PI * 47500, EFF_ACOUS) * d * ALPHA_MAT);
	double temp_e = EFF_PIEZO * EFF_PIEZO * EFF_ACOUS2DC * g_coeff * ACOUS_ENERGY_SEND;
	this->SC_E += temp_e;

	this->updateVolt(this->SC_V);
	//! Total voltage cannot exceed the maximum voltage level.
	this->SC_V = std::min(this->SC_V, this->SC_Vmax);

	return temp_e;
}

template <class T>
void SensorNode<T>::printSensorNodeInfo() {
	this->pos.printPointLoc();

	std::cout << "Sensor Node Information:" << std::endl << std::endl
		<< "Capacitance: \t" << this->SC_C << " F" << std::endl
		<< "Capacitor Energy: \t" << this->SC_E << " J" << std::endl
		<< "Capacitor Voltage: \t" << this->SC_E << " V" << std::endl
		<< "Weight: \t" << this->weight << std::endl
		<< "T: \t" << this->time_to_change << " s" << std::endl
		<< "Original T: \t" << this->time_to_reset << " s" << std::endl
		<< "Fail times: \t" << this->fails << std::endl
		<< "P sensor type? \t" << this->time_to_reset << std::endl
		<< "Maximum Voltage: \t" << this->SC_Vmax << " V" << std::endl
		<< "Minimum Voltage: \t" << this->SC_Vmin << " V" << std::endl
		<< "Critical Voltage: \t" << this->SC_Vcritical << " V" << std::endl
		<< "Current Voltage: \t" << this->SC_V << " V" << std::endl
		<< "Sense Voltage: \t" << this->V_sense << " V" << std::endl
		<< "Sense Current: \t" << this->I_sense << " A" << std::endl
		<< "Idle Current: \t" << this->I_idle << " A" << std::endl
		<< "Sense cycle duration: \t" << this->sense_cycle << " s" << std::endl
		<< "Idle cycle duration: \t" << this->idle_cycle << " s" << std::endl
		<< "Communication cycle duration: \t" << this->comm_cycle << " s" << std::endl
		<< std::endl;
}
