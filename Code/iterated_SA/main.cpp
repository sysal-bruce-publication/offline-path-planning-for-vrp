/*! @file	main.cpp
 *
 *  @warning This is the internal header of the ODP project.
 *  Do not use it directly in other code. Please note that this file
 *  is based on the open source code from
 *  <a href="https://github.com/achu6393/dynamicWeightedClustering">
 *	dynamicWeightedClustering
 *  </a>
 *  Copyright (C) Qiuchen Qian, 2020
 *  Imperial College, London
 *  Users should ensure that there are no overlapped sensor nodes.
 *
 *  @pre Total number of sensor nodes @a NODE_NUM needs to be set at the beginning. \\
 * 	  C++ language version ISO C++17. \\
 * 	  Set SDL check to NO.
 */


#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include "point.h"
#include "sensornode.h"
#include "pdv.h"
#include "annealing.h"
#include "point.cpp"
#include "sensornode.cpp"
#include "pdv.cpp"
#include "annealing.cpp"

using namespace std;
namespace fs = std::filesystem;

/*!
*  @defgroup	SN_CONSTANTS Sensor node parameters
*  @{
*/

//��The number of all sensor nodes
#define NODE_NUM 1500

/*! @} */


/*! @fn			vector<SensorNode<float>> readFullData(int pdv_num, string fname = "")
 *  @brief			Read CSV file of all sensor nodes information.
 *  @param pdv_num	The number of PDVs.
 *  @param fname		The absolute path to read the CSV file.
 *  @exception runtime_error		No such file or directory.
 *  @exception invalid_argument	Exist duplicates in sensor node deployment.
 *  @return			A vector of @c float type (in this case) sensor nodes.
 */
vector<SensorNode<float>> readFullData(int pdv_num, string fname = "") {
	vector<SensorNode<float>> sn_list;
	sn_list.reserve(pdv_num);
	string all_inputs_fname = "../input/inputs.csv";
	if (fname.length()) all_inputs_fname = fname;

	fstream file;
	try {
		file.open(all_inputs_fname, fstream::in);
		if (!file.is_open()) throw runtime_error(strerror(errno));
	}
	catch (const std::exception& e) {
		cerr << e.what() << "\n";
	}

	string line;
	if (file.good()) getline(file, line);

	string xs, ys, flags, volts, weights;
	while (getline(file, xs, ',')) {
		getline(file, ys, ',');
		getline(file, flags, ',');
		getline(file, volts, ',');
		getline(file, weights);

		// Convert to meters
		sn_list.push_back(SensorNode<float>(static_cast<float>(stof(xs) / 100),
			static_cast<float>(stof(ys) / 100), stof(volts), stoi(weights),
			static_cast<bool>(stoi(flags))));
	}
	file.close();

	for (unsigned i = 0; i < sn_list.size() - 1; i++) {
		for (unsigned j = i + 1; j < sn_list.size(); j++) {
			if (sn_list[i].pos.isCoincide(sn_list[j].pos)) {
				throw invalid_argument("Exist duplicates in sensor node deployment !");
			}
		}
	}

	return sn_list;
}

/*! @fn			vector<Point<float>> readSubData(int num, string fname = "")
 *  @brief			Read CSV file of sub path for each single PDV
 *  @param num		The PDV ID number.
 *  @param fname		The absolute path to read the CSV file.
 *  @exception runtime_error		No such file or directory.
 *  @exception invalid_argument	Exist duplicates in sensor node deployment.
 *  @return			A vector of @c float type (in this case) points.
 */
vector<Point<float>> readSubData(int num, string fname = "") {
	vector<Point<float>> path;
	string sub_input_fname = "../input/sub_path" + to_string(num) + ".csv";
	if (fname.length()) sub_input_fname = fname;

	fstream file;
	try {
		file.open(sub_input_fname, fstream::in);
		if (!file.is_open())
			throw runtime_error(strerror(errno));
	}
	catch (const std::exception& e) {
		cerr << e.what() << "\n";
	}

	string line;
	if (file.good()) getline(file, line);

	string xs, ys;
	while (getline(file, xs, ',')) {
		getline(file, ys);

		path.push_back(Point<float>(stof(xs), stof(ys)));
	}
	file.close();

	for (unsigned i = 0; i < path.size() - 1; i++) {
		for (unsigned j = i + 1; j < path.size(); j++) {
			if (path[i].isCoincide(path[j])) {
				throw invalid_argument("Exist duplicate points in the flight path !");
			}
		}
	}

	return path;
}

/*! @fn			vector<SensorNode<float>> readFullData(int pdv_num, string fname = "")
 *  @brief			Read CSV file of all sensor nodes information.
 *  @param pdv_num	The number of PDVs.
 *  @param fname		The absolute path to read the CSV file.
 *  @exception runtime_error		No such file or directory.
 *  @exception invalid_argument	Exist duplicates in sensor node deployment.
 *  @return			A vector of @c float type (in this case) sensor nodes.
 */
vector<Point<float>> readSubPath(int pdv_num, string fname = "") {
	vector<Point<float>> p_list;
	p_list.reserve(pdv_num);
	string sub_fname = "../output/sub_path/sa_path" + to_string(pdv_num) + ".csv";
	if (fname.length()) sub_fname = fname;

	fstream file;
	try {
		file.open(sub_fname, fstream::in);
		if (!file.is_open()) throw runtime_error(strerror(errno));
	}
	catch (const std::exception& e) {
		cerr << e.what() << "\n";
	}

	string line;
	if (file.good()) getline(file, line);

	string xs, ys;
	while (getline(file, xs, ',')) {
		getline(file, ys);

		p_list.push_back(Point<float>(static_cast<float>(stof(xs)), static_cast<float>(stof(ys))));
	}
	file.close();

	//for (unsigned i = 0; i < sn_list.size() - 1; i++) {
	//	for (unsigned j = i + 1; j < sn_list.size(); j++) {
	//		if (sn_list[i].pos.isCoincide(sn_list[j].pos)) {
	//			throw invalid_argument("Exist duplicates in sensor node deployment !");
	//		}
	//	}
	//}

	return p_list;
}

/*! @fn			void saveDataToCsv(int pdv_num, float* pct_list, vector<SensorNode<T>> sn_list, string fname = "")
 *  @brief			Save all needed sensor node information (same format as input file) after recharging.
 *  @param pdv_num	The number of PDVs.
 *  @param pct_list     A @c float array of PDVs' charged percentage.
 *  @param sn_list	A vector of all sensor nodes.
 *  @param fname		The absolute path to save the CSV file.
 *  @param info_fname	The file name of three key points to evaluate the performance
 *  @exception runtime_error		No such file or directory.
 */
template <class T>
void saveDataToCsv(int pdv_num, float* pct_list, double* pdv_eng_cost, double* charged_eng,
	double* flight_ds, double spent_t, vector<SensorNode<T>> sn_list, string fname = "",
	string info_fname = "") {
	string final_fname = "../output/final_output.csv";
	if (fname.length()) final_fname = fname;

	fstream file;
	try {
		file.open(final_fname, fstream::out);
		if (!file.is_open())
			throw runtime_error(strerror(errno));
	}
	catch (const exception& e) {
		cerr << e.what() << "\n";
	}

	file << "x_pos,y_pos,p_flag,volts,weights" << endl;
	for (unsigned i = 0; i < sn_list.size(); i++) {
		file << sn_list[i].pos.getX() << ","
			<< sn_list[i].pos.getY() << ","
			<< sn_list[i].p_sensor_type << ","
			<< sn_list[i].SC_V << ","
			<< sn_list[i].weight << endl;
	}
	file.close();

	string pect_path = "../output/final_info.csv";
	if (info_fname.length()) pect_path = info_fname;
	try {
		file.open(pect_path, fstream::out);
		if (!file.is_open())
			throw runtime_error(strerror(errno));
	}
	catch (const exception& e) {
		cerr << e.what() << "\n";
	}

	file << "ID,flight_distance,task_achievement,pdv_energy_cost,total_charged_energy" << endl;
	for (int i = 0; i < pdv_num; i++) {
		file << i << "," << flight_ds[i] << "," << pct_list[i]
			<< "," << pdv_eng_cost[i] << "," << charged_eng[i] << endl;
	}
	file << "alg_time," << spent_t << endl;

	file.close();
}

int main() {
	vector<SensorNode<float>> sn_list;
	sn_list.reserve(NODE_NUM);
	sn_list = readFullData(NODE_NUM);

	Annealing<float>* sa = new Annealing<float>;
	if (!sa->checkTask(sn_list)) {
		delete sa;
		cerr << "Do not have enough sensor nodes to recharge." << endl;
		return -1;
	}
	double alg_t = sa->alg_time;
	delete sa;

	int pdv_num = 0;
	fs::path path("../output/sub_path");
	auto dir_iter = fs::directory_iterator(path);
	for (auto& entry : dir_iter) {
		pdv_num++;
	}

	float* pect_list = new float[pdv_num];
	double* pdv_eng_cost = new double[pdv_num];
	double* delta_wsn_eng = new double[pdv_num];
	double* flight_ds = new double[pdv_num];
	double* flight_time = new double[pdv_num];

	vector<vector<Point<float>>> final_sols;
	final_sols.reserve(pdv_num);
	for (int i = 0; i < pdv_num; i++) {
		final_sols.push_back(readSubPath(i));
	}

	for (int i = 0; i < pdv_num; i++) {
		pect_list[i] = (float)(0.);
		pdv_eng_cost[i] = 0.;
		delta_wsn_eng[i] = 0.;
		flight_ds[i] = 0.;
		flight_time[i] = 0.;

		unique_ptr<PDV<float>> pdv = make_unique<PDV<float>>();
		float pect = pdv->flightSimulation(delta_wsn_eng[i], flight_time[i], sn_list, final_sols[i]);
		pect_list[i] = pect;
		pdv_eng_cost[i] = 187. - pdv->getPdvEnergy();
		flight_ds[i] = pdv->getPdvDistance();
	}

	double max_t = flight_time[0];
	for (int i = 1; i < pdv_num; i++) {
		if (flight_time[i] > max_t) max_t = flight_time[i];
	}

	//! Update all Sensor Energy
	for (unsigned i = 0; i < sn_list.size(); i++) {
		if (sn_list[i].SC_V > sn_list[i].getCriticalVolt()) {
			sn_list[i].updateEnergy(max_t * 3600, sn_list[i].SC_E);
			sn_list[i].updateVolt(sn_list[i].SC_V);
			sn_list[i].updateWeight(sn_list[i].SC_V, sn_list[i].weight);
		}
	}

	saveDataToCsv(pdv_num, pect_list, pdv_eng_cost, delta_wsn_eng, flight_ds, alg_t, sn_list);
	delete[] pect_list;
	delete[] pdv_eng_cost;
	delete[] delta_wsn_eng;
	delete[] flight_ds;
	delete[] flight_time;

	return 0;
}