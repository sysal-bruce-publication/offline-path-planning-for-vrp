/*! @file	genetic.h
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


#pragma once

#include "sensornode.h"

#ifndef ANNEALING_H_
#define ANNEALING_H_

using namespace std;

template <class T>
class Annealing : protected SensorNode<T>
{
public:
	Point<T>* origin = nullptr;
	vector<vector<vector<int>>> tars_idx;		//!< pop, pdv and sub path
	vector<vector<int>> trails_idx;
	vector<vector<double>> tars_met;
	vector<double> trails_met;
	vector<Point<T>> req_ps;
	double alg_time = 0.;

	Annealing();
	~Annealing();

	void initParams(vector<SensorNode<T>*>& req_sn_ptr);

	/*! @brief			Save initial guess to csv file
	 *  @param pop_num	The number of population in generation
	 *  @param pdv_num	The number of pdv id.
	 *  @param path_to_save	The path to be saved
	 *  @tparam T		The type of data to present point coordinates.
	 */
	void saveGuessToTxt(int pop_num, int pdv_num, int sn_num, vector<int> path_to_save);

	/*! @brief			Calculate the optimised number of needed PDVs.
	 *  @param sn_list	A vector of all sensor nodes.
	 *  @param req_sn_ptr	A vector of pointers to all sensor nodes to be recharged.
	 *  @param temp_req_p	A vector of pointers to all @a Point obejects of sensor nodes to be recharged.
	 *  @tparam T		The type of data to present point coordinates.
	 *  return			The optimised number of needed PDVs.
	 */
	int calcOptPdvNum(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> temp_req_p);

	/*! @brief			Initialize all target and trail vectors randomly with clusters solution from all sensors
	 *  @param r_num		The number of shortest point index.
	 *  @param pdv_num	The number of PDVs.
	 *  @param pop_num	The number of total population.
	 *  @param sn_list	A vector of all sensor nodes.
	 *  @param req_sn_ptr   A vector of pointers to all sensor nodes to be recharged.
	 *  @param req_ps		A vector of @a Point objects of all sensor nodes to be recharged.
	 *  @tparam T		The type of data to present point coordinates.
	 *  @return			If the number of nodes to be recharged can be divided exactly by @a pdv_num, return true.
	 */
	bool calcInitGuess(const int& r_num, const int& pdv_num, const int& pop_num, vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> req_ps);

	/*! @brief			Calculate the flight distance through finding the farest neighbour everytime.
	 *
	 *  Please note that this solution is used to convert the flight distance calculated in @c fitnessFun()
	 *  around 0 or 1. The returned result should not be considered as the longest one.
	 *
	 *  @param init_path    The path to be calculated.
	 *  @return the worst distance (predicted)
	 */
	double calcFarNeighDist(vector<Point<T>> init_path);

	/*! @brief			Calculate the flight distance through finding the nearest neighbour everytime.
	 *
	 *  Please note that this solution is used to convert the flight distance calculated in @c fitnessFun()
	 *  around 0 or 1. The returned result should not be considered as the shortest one.
	 *
	 *  @param init_path    The path to be calculated.
	 *  @return the best distance (predicted)
	 */
	double calcNearNeighDist(vector<Point<T>> init_path);

	/*! @brief		      Calculate the fitness of the possible solution.
	 *
	 *  The criteria is based on the amount of recharged energy and the distance of flight.
	 *  Related formula: Fitness @f[ F = \alpha * \Delta E_{WSN} + \frac{1}{\beta * d_{PDV}} @f]
	 *
	 *  @param sn_list	A vector of all sensor nodes.
	 *  @param clst_list	A vector of clusters to be calculated (can be one target or trail vector).
	 *  @param sn_address   The first element address in @a sn_list (can be used to find index).
	 *  @tparam T		The type of data to present point coordinates.
	 *  @return			The fitness metric value.
	 */
	double fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> clst_list);

	/*! @brief			Find the index of the best solution with the highest fitness metric value.
	 *  @param pdv_num	The number of needed PDVs.
	 *  @return			The index of the best solution.
	 */
	int getBestSol(const int& pdv_num);

	/*! @brief			Read the data from stored initial guess
	 *  @param pop_num	The population id.
	 *  @param pdv_num	The PDV id.
	 *  @return			A vector of solution (index of sensor nodes) with @a pop_num and @a pdv_num
	 */
	vector<int> readGuessData(int pop_num, int pdv_num);

	/*! @brief			Save sub paths with best metric 
	 *  @param sn_list	A vector of all sensor nodes.
	 *  @param path_to_save	The 2-D vector to save.
	 *  @tparam T		The type of data to present point coordinates.
	 */
	void saveSubPathToCsv(vector<SensorNode<T>> sn_list, vector<vector<int>> path_to_save);
	
	void initOneSol(const int& cur_pdv, vector<vector<int>>& idx_list, vector<Point<T>> req_ps, vector<SensorNode<T>> sn_list);

	/*! @brief			Implement all processes of the genetic algorithm.
	 *
	 *  This function includes initialization of target and trail vectors, calculation of their fitness
	 *  metric values, 'Crossover', 'Swap Mutation', 'Selection', genetic iterations. Please note that
	 *  unlike normal GA, both target and trail vectors are generated from all sensor nodes as possible
	 *  solutions. After comparsion of fitness metrics, the winner will survive for next generation,
	 *  which will keep the 'population' evolving towards to desired direction (converge to a trusted
	 *  solution).
	 *
	 *  @param sn_list	A vector of all sensor nodes.
	 *  @param candidates	A vector of pointers to requested sensor nodes.
	 *  @tparam T		The type of data to present point coordinates.
	 *  @return			A vector of points (target flight position).
	 */
	void calcFinalPath(vector<SensorNode<T>>& sn_list, vector<SensorNode<T>*> candidates);

	/*! @brief			Check if the number of requested sensor nodes is larger than @a MIN_REQUESTS .
	 *  @param sn_list	A vector of all sensor nodes.
	 *  @param path		A vector of @a Point objects to be updated.
	 *  @param pdv_id		The ID number of the PDV.
	 *  @exception runtime_error		No such file or directory.
	 *  @return			If true, keep iterating until the requested number reaches the minimum value.
	 *				If false, end iterating.
	 */
	bool checkTask(vector<SensorNode<T>>& sn_list);
};

#endif // !ANNEALING_H_