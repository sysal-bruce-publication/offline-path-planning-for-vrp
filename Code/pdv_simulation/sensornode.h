/*! @file	sensornode.h
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
#include "point.h"

#ifndef SENSORNODE_H_
#define SENSORNODE_H_

 /*!
  *  @defgroup	SN_CONSTANTS Sensor node parameters
  *  @{
  */

//! Coefficient when calculating the acoustic channel loss co-efficient g(f, d)
#define ALPHA_MAT 3.21
//! Acoustic energy transfer efficiency constant
#define EFF_ACOUS	8.58e-3
//! Maximum acoustic transfer distance [m]
#define MAX_ACOUS_DIST 0.7
//! Circumference
#define PI 3.1415926535897932384626
//! Piezo driver energy convert efficiency [%]
#define EFF_PIEZO	0.9
//! Acoustic to DC Efficiency	[%]
#define EFF_ACOUS2DC 0.98
//! Energy Sent for acoustic transfer [J]
#define ACOUS_ENERGY_SEND 12
//! Maximum allowable sensing fail times
#define MAX_FAILS 5

/*! @} */


/*!
 *  @class		SensorNode
 *  @headerfile	SensorNode.h "SensorNode.h"
 *  @brief		Implementation of @a SensorNode class
 *
 *  The @a SensorNode class includes basic attributes like voltage, energy,
 *  weight, etc. and actions like simulating energy consumption, updating weight,
 *  etc.
 *
 *  @author		Qiuchen Qian
 *  @version	3
 *  @date		2020
 *  @warning	C4996 'strerror': This function or variable may be unsafe.
 *			Consider using strerror_s instead.
 *  @copyright	MIT Public License
 */
template <class T>
class SensorNode : protected Point<T>
{
	//! @publicsection
public:
	Point<T> pos;			/*!< A @a Point object with data type @c T for sensor node position [m] */
	double SC_V = 3.4;	/*!< Voltage of the super capacitor [v] */
	double SC_E = 17.34;	/*!< Energy of the super capacitor [J] */
	int weight = 10;	/*!< Recharge weight, 3: low, 10: full */
	int time_to_change = 1;	/*!< Sense cycle duration [s] */
	bool p_sensor_type = true;    /*!< Flag to determine pressure or temperature sensor */


	/*! A default constructer with coordinate (0, 0), 3.4 V voltage,
	 *  17.34 J energy, full weight, 0 sensing fail time and p-type
	 *  sense duration
	 */
	SensorNode();

	/*! @brief		Construct object with user-defined arguments
	 *  @param x	A @c float variable to record position
	 *  @param y	A @c float variable to record position
	 *  @param v	A @c double variable to record voltage
	 *  @param w	A @c int variable to record weight
	 *  @param p_type A @c bool variable to record sensor type
	 */
	SensorNode(const float& x, const float& y, const double& v, int w, bool p_type);

	//! A default destructer.
	~SensorNode();


	/*! @brief		Access @c protected member variable @a SC_Vmin
	 *  @return		@c protected member variable @a SC_Vmin
	 */
	double getMinVolt() const { return this->SC_Vmin; }

	/*! @brief		Access @c protected member variable @a SC_Vmax
	 *  @return		@c protected member variable @a SC_Vmax
	 */
	double getMaxVolt() const { return this->SC_Vmax; }

	/*! @brief		Access @c protected member variable @a SC_Vcritical
	 *  @return		@c protected member variable @a SC_Vcritical
	 */
	double getCriticalVolt() const { return this->SC_Vcritical; }

	/*! @brief		Access @c protected member variable @a V_sense
	 *  @return		@c protected member variable @a V_sense
	 */
	double getSenseVolt() const { return this->V_sense; }

	/*! @brief		Access @c protected member variable @a I_idle
	 *  @return		@c protected member variable @a I_idle
	 */
	double getIdleAmp() const { return this->I_idle; }

	/*! @brief		Access @c protected member variable @a idle_cycle
	 *  @return		@c protected member variable @a idle_cycle
	 */
	double getIdleCycle() const { return this->idle_cycle; }

	/*! @brief		Access @c protected member variable @a time_to_reset
	 *  @return		@c protected member variable @a time_to_reset
	 */
	int getTimeToReset() const { return this->time_to_reset; }

	/*! @brief		Access @c protected member variable @a SC_C
	 *  @return		@c protected member variable @a SC_C
	 */
	int getCapacitance() const { return this->SC_C; }

	/*! @brief		Access @c protected member variable @a fails
	 *  @return		@c protected member variable @a fails
	 */
	int getFails() const { return this->fails; }


	//! @brief		Auto-increment of @c protected member variable @a fails
	void addOneFail() { this->fails++; }

	//! @brief		Reset @c protected member variable @a fails	
	void resetFail() { this->fails = 0; }

	/*! @brief		Calculate needed energy to fully recharge a Sensor from its current volatge
	 *
	 *  Related formula: energy change @f[ \Delta E = \frac{1}{2} * C * (V_{max} - V)^2 @f]
	 *
	 *  @return		Needed energy
	 */
	double calcPackage() const { return 0.5 * this->SC_C * (pow(this->SC_Vmax, 2) - pow(this->SC_V, 2)); }

	/*! @brief		Update voltage according to current energy
	 *
	 *  Related formula: Voltage level @f[ V = \sqrt(2 * \frac{E}{C}) @f]
	 *
	 *  @param v	Voltage variable to be updated
	 */
	void updateVolt(double& v);

	/*! @brief		Update energy according to current voltage
	 *
	 *  Related formula: Energy level @f[ E = \frac{1}{2} * C * V^2) @f]
	 *
	 *  @param e	Energy variable to be updated
	 */
	virtual void updateEnergy(double& e);

	/*! @brief		Update sensor node energy according to spent time [s]
	 *
	 *  Related formula: Energy consumption @f[ E = V * I * t @f]
	 *
	 *  @param t	time spent variable
	 *  @param e	Energy to be updated
	 */
	virtual void updateEnergy(const double& t, double& e);

	/*! @brief		Update weight according to current voltage
	 *  @param v	Current volatge
	 *  @param w	Weight to be updated
	 */
	void updateWeight(const double& v, int& w);

	/*! @brief		Implement acoustic energy transfer
	 *
	 *  Energy of end nodes will be increased with transferred acoustic energy.
	 *  Updates for energy and voltage will be implemented as well. To calculate
	 *  the acoustic channel loss co-efficient, angular frequency of acoustic waves
	 *  should be calculated first. Parameters were picked from the research of
	 *  M.E. Kiziroglou et al. See the paper for more information:
	 *  <a href="https://www.sciencedirect.com/science/article/pii/S0041624X17300471">
	 *	Acoustic power delivery to pipeline monitoring wireless sensors
	 *  </a>
	 *  Related formulas: Angular frequency @f[ \omega = 2 * \pi * f @f]
	 *  Acoustic channel loss co-efficient @f[ g = e^{-\omega^\eta * d * \alpha} @f]
	 *  Sent acoustic energy @f[ E_t = \eta_{piz} * g * E_{aco} } @f]
	 *  Received acoustic energy @f[ E_r = \eta_{piz} * \eta_{p2dc} * E_t @f]
	 *
	 *  @param d	Distance between two sensor nodes
	 */
	double acousTransfer(const double& d);

	//! Print all information about sensor nodes
	void printSensorNodeInfo();

	//! @protectedsection
protected:
	int fails = 0;				/*!< Sensing fail times for a sensor node */
	int time_to_reset = 1;			/*!< Sensing cycle to reset [s] */
	int SC_C = 3;				/*!< Super Capacitor capacitance [F] */
	double SC_Vmax = 5.;			/*!< Maximum voltage of super capacitor [V] */
	double SC_Vmin = 3.5;			/*!< Minimum voltage of super capacitor [V] */
	double SC_Vcritical = 3.3;		/*!< Critical voltage of super capacitor [V] */
	double V_sense = 3.3;			/*!< Voltage when sensing data [V] */
	double I_sense = 1.2e-3;		/*!< Current when sensing data [A] */
	double I_idle = 4e-6;			/*!< Current when in idle cycle [A] */
	double sense_cycle = 2e-3;		/*!< Sense cycle duration [s] */
	double idle_cycle = 9.498;		/*!< Idle cycle duration [s] */
	double comm_cycle = 0.5;		/*!< Communication cycle duration [s] */
};							//! End of class Point


/*!
 *  @class		Cluster
 *  @headerfile	Cluster.h "Cluster.h"
 *  @brief		Implementation of @c Cluster class
 *
 *  The @c Cluster class includes a center node and a list of end nodes (if in
 *  acoustic distance limit). Note that for better memory management, pointers
 *  are used .This class is created for further genetic algorithm and necessary
 *  acoustic energy transfer.
 *
 *  @author		Qiuchen Qian
 *  @version	3
 *  @date		2020
 *  @bug		Current version cannot delete pointers in the class properly.
 *  @warning	If delete pointers as commented in destructor, @c _CtrlsValidHeapPointer(block)
 *			@c is_block_type_valid(header->_block_use) error will be reported by the debugger.
 *  @copyright	MIT Public License
 */

template <class T>
struct Cluster
{
	Point<T>* center;					/*!< Position of the center node */
	std::vector<SensorNode<T>*> contains;	/*!< Pointers of contained end nodes */

	//! A default constructer with default @a Point and @a SensorNode constructor.
	Cluster();
	//! Construct the object with specific @a Point and @a SensorNode.
	Cluster(Point<T>* c, SensorNode<T>* s);
	//! A default destructor
	~Cluster();

	/*! @brief			Assign end nodes to one center node
	 *  @param sn_list	A vector of all sensor nodes
	 */
	void assignEndNodes(std::vector<SensorNode<T>>& sn_list);
};								//! End of class Cluster

template <class T>
Cluster<T>::Cluster() {
	this->center = &Point<T>();
	this->contains.clear();
	this->contains.push_back(&SensorNode<T>());
}

template <class T>
Cluster<T>::Cluster(Point<T>* c, SensorNode<T>* s) {
	this->center = c;
	this->contains.clear();
	this->contains.push_back(s);
}

template <class T>
Cluster<T>::~Cluster() {
	//delete this->center;
	//for (auto p : this->contains) {
	//	if (p != NULL) delete p;
	//}
	//this->contains.clear();
}

template <class T>
void Cluster<T>::assignEndNodes(std::vector<SensorNode<T>>& sn_list) {
	this->contains.clear();

	for (unsigned i = 0; i < sn_list.size(); i++) {
		double d = this->center->calcDist(sn_list[i].pos);
		if (d >= MAX_ACOUS_DIST || d <= 0.05) continue;
		else {
			this->contains.push_back(&sn_list[i]);
		}
	}
} 

#endif // !SENSORNODE_H_



