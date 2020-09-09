/*! @file	pdv.h
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

#ifndef PDV_H_
#define PDV_H_

 /*!
 *  @class		PDV pdv.h "pdv.h"
 *  @brief		Implementation of @a PDV class
 *
 *  The @a PDV class includes basic attributes like PDV position, flight time
 *  and energy. Some PDV actions like fly approaching to next target node through
 *  GPS localization and UWB localization, return-to-home process, inductive
 *  power transfer and flight simulation are included as well.
 *
 *  @author		Qiuchen Qian
 *  @version	5
 *  @date		2020
 *  @copyright	MIT Public License
 */
template <class T>
class PDV : protected SensorNode<T>
{
//! @publicsection
public:
	Point<T> pos;	/*!< A @a Point object with data type @c T for sensor node position [m]*/

	//! A default constructer with coordinate(0, 0), 0 flight time and full energy.
	PDV();
	//! A default destructer which sets PDV attributes to initial state.
	~PDV();

	//! @brief		Reset PDV status to initial value
	void resetPdvStatus();

	/*! @brief		Access @c protected member variable @a pdv_energy_
	*   @return		@c protected member variable @a pdv_energy_
	*/
	double getPdvEnergy() const { return this->pdv_energy_; }

	/*! @brief		Access @c protected member variable @a flight_time_
	*   @return		@c protected member variable @a flight_time_
	*/
	double getPdvTime() const { return this->flight_time_; }

	/*! @brief		Access @c protected member variable @a flight_distance_
	*   @return		@c protected member variable @a flight_distance_
	*/
	double getPdvDistance() const { return this->flight_distance_; }

	/*! @brief		Get randomly generated @c float variable ranging from -1 to 1.
	*
	*   Please note that the function is used to simulate the error due to GPS
	*   localization precision. The offset around 1 m is a reasonable
	*   range in real life.
	*
	*   @return		Random @c float variable ranging from -1 to 1 [m]
	*/
	float getRandOffset() const;

	/*! @brief			Check if the number of requested sensor nodes is larger than @a MIN_REQUESTS .
	*   @param sn_list	A vector of all sensor nodes.
	*   @return			If true, the PDV can fly for one task.
	*				If false, end the process.
	*/
	bool taskCheck(std::vector<SensorNode<T>>& sn_list);

	/*! @brief			Calculate energy consumption according to power and spent time.
	*
	*   Related formula: Energy E = P * t 
	*
	*   @param t		Spent time [h].
	*   @return			the consumed energy [Wh].
	*/
	double calcEnergyCost(const double& t) const { return this->pdv_power * t; }

	/*! @brief			Calculate energy consumption after ascent
	*
	*   Related formula: Time t = 20 / V_{asc} 
	*
	*   @param t		Spent time [h].
	*   @param e		Energy variable to be updated [Wh].
	*/
	void ascentEnergyCost(double& t, double& e);

	/*! @brief			Calculate energy consumption after descent
	*
	*   Related formula: Time t = 20 / V_{des}
	*
	*   @param t		Spent time [h].
	*   @param e		Energy variable to be updated [Wh].
	*/
	void descentEnergyCost(double& t, double& e);

	/*! @brief			Calculate energy consumption after UWB approaching
	*
	*   Related formula: Time t = (10 - offset) / V_{uwb}
	*
	*   @param rand_offset	Random @c float variable ranging from -1 to 1 [m]
	*   @param t		Spent time [h].
	*   @param e		Energy variable to be updated [Wh].
	*/
	void uwbEnergyCost(const float& rand_offset, double& t, double& e);

	/*! @brief			Calculate energy consumption after GPS approaching
	*
	*   Related formula: Time t = (d - (10 - offset)) / V_{gps}
	*
	*   @param rand_offset	Random @c float variable ranging from -1 to 1 [m]
	*   @param next_p		Next target position
	*   @param t		Spent time [h].
	*   @param e		Energy variable to be updated [Wh].
	*/
	void gpsEnergyCost(const float& rand_offset, const Point<T>& next_p, double& t, double& e);

	/*! @brief			Pre-calculate energy consumption from next target point to origin point.
	*
	*   Related formula: Time t = (d - (10 - offset))(V_{gps})
	*
	*   @param rand_offset	Random @c float variable ranging from -1 to 1 [m].
	*   @param next_p		Next target position.
	*   @param t		Spent time [h].
	*   @param e		Energy variable to be updated [Wh].
	*/
	void rthEnergyCost(const float& rand_offset, Point<T> next_p, double& t, double& e);

	/*! @brief			Calculate energy consumption after inductive power transfer.
	*
	*   Related formula: Energy E = 1 / (2 * n_{rf2dc} * 3600) * C * (V_{max} - V)^2
	*
	*   @param next_sn	The next target sensor node to charge
	*   @param e		Energy variable to be updated [Wh].
	*/
	void iptEnergyCost(const SensorNode<T>& next_sn, double& e);

	/*! @brief			Update @a flight_time_ according to @a t .
	*   @param t		Spent time [h].
	*/
	void updateFlightTime(const double& t) { this->flight_time_ += t; }

	/*! @brief			Update @a flight_time_ according to @a t1 and @a t2 .
	*   @param t1		Spent time [h].
	*   @param t2		Spent time [h].
	*/
	void updateFlightTime(const double& t1, const double& t2) { this->flight_time_ += (t1 + t2); }

	/*! @brief			Update @a pdv_energy_ according to @a e .
	*   @param e		Energy consumption [Wh]
	*/
	void updateEnergy(const double& e) { this->pdv_energy_ -= e; }

	/*! @brief			Update @a pdv_energy_ according to @a e1 and @a e2 .
	*   @param e1		Energy consumption [Wh]
	*   @param e2		Energy consumption [Wh]
	*/
	void updateEnergy(const double& e1, const double& e2) { this->pdv_energy_ -= (e1 + e2); }

	/*! @brief			Update @a flight_distance_ according to @a d .
	*   @param d		Flight distance [m]
	*/
	void updateFlightDist(const double& d) { this->flight_distance_ += d; }

	/*! @brief			Update @a pos according to @a p .
	*   @param p		A @a Point object for updating position
	*/
	void updatePos(const Point<T>& p) { this->pos.setX(p.getX()); this->pos.setY(p.getY()); }

	/*! @brief			Simulate the whole process.
	*
	*   The PDV will depart from the base station and visit the sensor nodes in @a path
	*   one by one, and once it completed all targets in the recharging list, it will
	*   return to the base station. Everytime before the PDV visiting the next target
	*   sensor node, it will check its remain energy, if not enough for next visit, power
	*   transfer and RTH implementation, it execute RTH immediately.
	*
	*   @param charged_e	Total charged energy to be recorded.
	*   @param pdv_t		Total flight time of the PDV.
	*   @param sn_list	A vector of all sensor nodes
	*   @param path		A vector of all points to visit
	*   @return			Task achievement percenetage [%]
	*/
	float flightSimulation(double& charged_e, double& pdv_t, std::vector<SensorNode<T>>& sn_list, std::vector<Point<T>>& path);

//! @privatesection
private:
	double flight_time_ = 0.;			/*!< PDV flight time [h]*/
	double pdv_energy_ = 0.;			/*!< PDV remain energy [Wh]*/
	double flight_distance_ = 0.;			/*!< PDV flight distance [m]*/

	int min_charge_num = 20;			/*!< Minimum number of sensor nodes to charge*/
	int flight_altitude = 20;			/*!< PDV flight altitude [m]*/
	double pdv_power = 363.888;			/*!< PDV power rating [W]*/
	double max_ascent_speed = 1.8e4;		/*!< Maximum ascent speed [m/h]*/
	double max_descent_speed = 1.44e4;		/*!< Maximum descent speed [m/h]*/
	double max_gps_speed = 2.16e4;		/*!< Maximum approaching speed through GPS localization [m/h]*/
};

#endif // !PDV_H_
