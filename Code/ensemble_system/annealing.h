/*! @file	annealing.h
*
*   @warning This is the internal header of the ODP project.
*   Do not use it directly in other code.
*
*   Copyright (C) Qiuchen Qian, 2020
*   Imperial College, London
*/


#pragma once
#include "sensornode.h"

#ifndef ANNEALING_H_
#define ANNEALING_H_

using namespace std;

/*! @class		Annealing annealing.h "annealing.h"
*   @brief		Implementation of @a Annealing class
*
*   The @a Genetic class includes basic attributes like target, trail
*   vectors, metric vectors, etc. and actions like reduce temperature,
*   calculate fitness function etc. 
*
*   @author		Qiuchen Qian
*   @version	5
*   @date		2020
*   @warning	C4996 'strerror': This function or variable may be unsafe.
*			Consider using strerror_s instead.
*   @copyright	MIT Public License
*/
template <class T>
class Annealing : protected SensorNode<T>
{
public:
	Point<T>* origin = nullptr;				/*!< The coordiante of BS */
	vector<vector<vector<int>>> tars_idx;		/*!< Target vector, 3D: pop, pdv and sub path */
	vector<vector<int>> trails_idx;			/*!< Corresponding trail vector */
	vector<vector<double>> tars_met;			/*!< Fitness metric vector of target vector */
	vector<double> trails_met;				/*!< Fitness metric vector of trail vector */
	vector<Point<T>> req_ps;				/*!< @a Point vector of SNs to be recharged */
	vector<vector<int>> best_sol;
	
	double alg_time = 0.;					/*!< Algorithm execution time */
	int coeff_wsn_eng = 50;
	int coeff_pdv_eng = 25;
	int coeff_dist = 25;
	int pop = 25;
	double init_temp = 1.2e3;
	double min_temp = 1e-5;
	double temp_factor = 0.98;
	int min_req_num = 20;
	int max_num_r = 5;

	//! A default constructer with @a origin (0, 0)
	Annealing();
	//! Another way to construct with user inputs
	Annealing(int w_wsn_eng, int w_pdv_eng, int w_pdv_dist, double init_t, double min_t, double t_fac, int n_pop, int n_req, int max_r);
	//! A default destructer which will delete @a origin.
	~Annealing();

	/*! @brief			Initialize public member variables.
	*   @param req_sn_ptr	A vector of pointers to all sensor nodes to be recharged.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void initParams(vector<SensorNode<T>*>& req_sn_ptr);

	/*! @brief			Save initial guess to csv file.
	*   @param pop_num	The number of population in generation.
	*   @param pdv_num	The number of pdv id.
	*   @param sn_num		The number of SNs.
	*   @param path_to_save	The path to be saved.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void saveGuessToTxt(int pop_num, int pdv_num, int sn_num, vector<int> path_to_save);

	/*! @brief			Calculate the minimum required number of needed PDVs.
	*   @param sn_list	A vector of all sensor nodes.
	*   @param req_sn_ptr	A vector of pointers to all sensor nodes to be recharged.
	*   @param temp_req_p	A vector of pointers to all @a Point obejects of sensor nodes to be recharged.
	*   @tparam T		The type of data to present point coordinates.
	*   @return			The optimised number of needed PDVs.
	*/
	int calcOptPdvNum(vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> temp_req_p);

	/*! @brief			Initialize all target and trail vectors randomly with clusters solution from all sensors
	*   @param r_num		The number of shortest point index.
	*   @param pdv_num	The number of PDVs.
	*   @param pop_num	The number of total population.
	*   @param sn_list	A vector of all sensor nodes.
	*   @param req_sn_ptr	A vector of pointers to all sensor nodes to be recharged.
	*   @param req_ps		A vector of @a Point objects of all sensor nodes to be recharged.
	*   @tparam T		The type of data to present point coordinates.
	*   @return			If the number of nodes to be recharged can be divided exactly by @a pdv_num, return true.
	*/
	bool calcInitGuess(const int& r_num, const int& pdv_num, const int& pop_num, vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr, vector<Point<T>> req_ps);

	/*! @brief		      Calculate the fitness of the possible solution.
	*
	*   The criteria is based on the amount of recharged energy and the distance of flight.
	*   Related formula: Fitness M = alpha * tanh(E_{wsn}) + beta * inv_tanh(d_{pdv}) + gamma * inv_tanh(E_{pdv})
	*
	*   @param sn_list	A vector of all sensor nodes.
	*   @param clst_list	A vector of clusters to be calculated (can be one target or trail vector).
	*   @tparam T		The type of data to present point coordinates.
	*   @return			The fitness metric value.
	*/
	double fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> clst_list);

	/*! @brief			Find the index of the best solution with the highest fitness metric value.
	*   @param pdv_num	The number of PDVs.
	*   @return			The index of the best solution.
	*/
	int getBestSol(const int& pdv_num);

	/*! @brief			Read the data from stored initial guess
	*   @param pop_num	The population id.
	*   @param pdv_num	The PDV id.
	*   @return			A vector of solution (index of sensor nodes) with @a pop_num and @a pdv_num
	*/
	vector<int> readGuessData(int case_num, int pop_num, int pdv_num);

	/*! @brief			Save sub paths with best metric
	*   @param sn_list	A vector of all sensor nodes.
	*   @param path_to_save	The 2-D vector to save.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void saveSubPathToCsv(vector<SensorNode<T>> sn_list, vector<vector<int>> path_to_save);

	/*! @brief			Save initial guess to csv file.
	*   @param cur_pdv	The number of pdv id.
	*   @param idx_list	The target vector to be re-generated.
	*   @param req_ps		A vector of @a Point objects to be recharged.
	*   @param sn_list	A vector of all sensor nodes.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void initOneSol(const int& cur_pdv, vector<vector<int>>& idx_list, vector<Point<T>> req_ps, vector<SensorNode<T>> sn_list);

	/*! @brief			Initialize all target and trail vectors randomly with clusters solution from all sensors
	*   @param sn_list	A vector of all sensor nodes.
	*   @param candidates	A vector of pointers to all sensor nodes to be recharged.
	*   @return			If the number of nodes to be recharged can be divided exactly by @a pdv_num, return true.
	*/
	void calcFinalPath(int case_num, vector<SensorNode<T>>& sn_list, vector<SensorNode<T>*> candidates);

	/*! @brief			Check if the number of requested sensor nodes is larger than @a MIN_REQUESTS .
	*   @param sn_list			A vector of all sensor nodes.
	*   @exception runtime_error		No such file or directory.
	*   @return			If true, keep iterating until the requested number reaches the minimum value.
	*				If false, end iterating.
	*/
	bool checkTask(int case_num, vector<SensorNode<T>>& sn_list);
};

#endif // !ANNEALING_H_