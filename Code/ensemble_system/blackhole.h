/*! @file	blackhole.h
*
*   @warning This is the internal header of the ODP project.
*   Do not use it directly in other code. 
*
*   Copyright (C) Qiuchen Qian, 2020
*   Imperial College, London
*/


#pragma once
#include "sensornode.h"

#ifndef BLACKHOLE_H_
#define BLACKHOLE_H_

using namespace std;

/*! @class		BlackHole blackhole.h "blackhole.h"
*   @brief		Implementation of @a BlackHole class
*
*   The @a BlackHole class includes basic attributes like target, trail
*   vectors, metric vectors, etc. and actions like Attraction, calculate
*   fitness function etc.
*
*   @author		Qiuchen Qian
*   @version	5
*   @date		2020
*   @warning	C4996 'strerror': This function or variable may be unsafe.
*			Consider using strerror_s instead.
*   @copyright	MIT Public License
*/
template <class T>
class BlackHole : protected SensorNode<T>
{
public:
	Point<T>* origin = nullptr;			/*!< The coordiante of BS */
	vector<vector<vector<int>>> tars_idx;	/*!< Target vector of index, 3D: pop, pdv and sub path */
	vector<vector<double>> tars_metric;		/*!< Corresponding trail vector */
	vector<Point<T>> req_ps;			/*!< @a Point vector of SNs to be recharged */
	vector<vector<int>> best_sol;
	
	double alg_time = 0.;				/*!< Algorithm execution time */
	int coeff_wsn_eng = 50;
	int coeff_pdv_eng = 25;
	int coeff_dist = 25;
	int gen = 50;
	int pop = 250;
	int ar = 50;
	int min_req_num = 20;
	int max_num_r = 5;

	//! A default constructer with @a origin (0, 0)
	BlackHole();
	//! Another way to pass parameters
	BlackHole(int w_wsn_eng, int w_pdv_eng, int w_pdv_dist, int n_gen, int n_pop, int n_ar, int n_req, int max_r);
	//! A default destructer which will delete @a origin.
	~BlackHole();

	/*! @brief			Allocate memory to public variables and initialize them.
	*   @param req_sn_ptr   A vector of pointer to @a SensorNode objects to be recharged.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void initParams(vector<SensorNode<T>*> req_sn_ptr);

	/*! @brief			According to current target vector, re-generate a new one.
	*   @param pdv_num	The number of PDVs.
	*   @param idx_list	The target vector at specific PDV id.
	*   @param req_ps		A vector of @a Point objects to be recharged.
	*   @param sn_list	A vector of all sensor nodes.
	*   @param req_sn_ptr   A vector of pointer to @a SensorNode objects to be recharged.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void initOneSol(const int& pdv_num, vector<vector<int>>& idx_list, vector<Point<T>> req_ps, vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> req_sn_ptr);
	
	/*! @brief			Save initial guess to csv file.
	*   @param pop_num	The number of population in generation.
	*   @param pdv_num_txt	The number of pdv id.
	*   @param sn_num		The number of SNs.
	*   @param path_to_save	The path to be saved.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void saveGuessToTxt(int pop_num, int pdv_num_txt, int sn_num, vector<int> path_to_save);

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
	*   @param idx_list	A index vector of targets to be calculated.
	*   @tparam T		The type of data to present point coordinates.
	*   @return			The fitness metric value.
	*/
	double fitnessFunc(vector<SensorNode<T>> sn_list, vector<int> idx_list);
	
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
	
	/*! @brief			Implement 'Attraction' to generate trail vectors.
	*   @param cur_gen	Current generation id.
	*   @param cur_pdv	Current PDV id.
	*   @param bh_num		The index of the black hole.
	*   @param sn_list	A vector of all sensor nodes.
	*   @param req_ps		A vector of @a Point objects to be recharged.
	*   @param tar_vec	The target vector at specific population id.
	*   @tparam T		The type of data to present point coordinates.
	*/
	void attraction(int cur_gen, int cur_pdv, const int& bh_num, vector<SensorNode<T>> sn_list, vector<Point<T>> req_ps, vector<vector<int>>& tar_vec);	

	/*! @brief			Implement all processes of the genetic algorithm.
	*
	*   This function includes initialization of target and trail vectors, calculation of their fitness
	*   metric values, 'Attraction' and iterations. Please note that
	*   unlike normal BH, both target and trail vectors are generated from all sensor nodes as possible
	*   solutions. After comparsion of the black hole bound, the winner star will survive for next generation,
	*   which will keep the 'population' evolving towards to desired direction (converge to a trusted
	*   solution).
	*
	*   @param sn_list	A vector of all sensor nodes.
	*   @param candidates	A vector of pointers to requested sensor nodes.
	*   @tparam T		The type of data to present point coordinates.
	*   @return			A vector of points (target flight position).
	*/
	void calcFinalPath(int case_num, vector<SensorNode<T>> sn_list, vector<SensorNode<T>*> candidates);

	/*! @brief			Check if the number of requested sensor nodes is larger than @a min_req_num.
	*   @param sn_list			A vector of all sensor nodes.
	*   @exception runtime_error		No such file or directory.
	*   @return			If true, keep iterating until the requested number reaches the minimum value.
	*				If false, end iterating.
	*/
	bool checkTask(int case_num, vector<SensorNode<T>>& sn_list);
};

#endif //! BLACKHOLE_H_
