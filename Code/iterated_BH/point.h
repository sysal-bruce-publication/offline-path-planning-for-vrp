/*! @file point.h
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
#include <vector>

#ifndef POINT_H_
#define POINT_H_

 /*!
  *  @class		Point
  *  @headerfile	point.h "point.h"
  *  @brief		Implementation of Point (class)
  *
  *  The @a Point  class includes the coordinate information, get and
  *  set function, calculate distance between two points or a list of points,
  *  and determine if two points are overlapped.
  *
  *  @author		Qiuchen Qian
  *  @version	3
  *  @date		2020
  *  @copyright	MIT Public License
  */
template <class T>
class Point
{
	//! @publicsection
public:
	//! A default constructer with coordinate (0, 0).
	Point();

	/*! @brief		Construct object with user-defined arguments
	 *  @param x	A @c float variable to record position
	 *  @param y	A @c float variable to record position
	 */
	Point(const double& x, const double& y);

	//! A default destructer.
	~Point();


	/*! @brief		Access @c protected member variable @a x_
	 *  @tparam T	The type of data to present point coordinates
	 *  @return		@c protected member variable @a x_
	 */
	T getX() const { return x_; }

	/*! @brief		Access @c protected member variable @a y_
	 *  @tparam T	The type of data to present point coordinates
	 *  @return		@c protected member variable @a y_
	 */
	T getY() const { return y_; }

	/*! @brief		Set @c protected member variable @a x_ with @c float @a x
	 *  @param x	A variable to assign value to @a x_
	 *  @tparam T	The type of data to present point coordinates
	 */
	void setX(const T& x) { this->x_ = x; }

	/*! @brief		Set @c protected member variable @a y_ with @c float @a y
	 *  @param y	A variable to assign value to @a y_
	 *  @tparam T	The type of data to present point coordinates
	 */
	void setY(const T& y) { this->y_ = y; }


	/*! @brief		Calculate the distance between two points
	 *  @details	Related formula: Distance @f[ d = \sqrt((x_1 - x_2)^2 + (y_1 + y_2)^2) @f]
	 *  @param p	A @a Point object with @c T data type
	 *  @tparam T	The type of data used to present point coordinates
	 *  @return		The distance between two points
	 */
	double calcDist(const Point<T>& p);

	/*! @brief		Calculate the distance between one point and specified @a x and @a y
	 *  @details	Related formula: Distance @f[ d = \sqrt((x_1 - x_2)^2 + (y_1 + y_2)^2) @f]
	 *  @param x	A @c float variable to calculate distance
	 *  @param y	A @c float variable to calculate distance
	 *  @tparam T	The type of data used to present point coordinates
	 *  @return		The @c float distance
	 */
	double calcDist(const float& x, const float& y);

	/*! @brief		Calculate the list of distances between one point and a list of points
	 *  @details	Related formula: Distance @f[ d = \sqrt((x_1 - x_2)^2 + (y_1 + y_2)^2) @f]
	 *  @param p_list A @c vector of points with type @c T to be calculated
	 *  @tparam T	The type of data used to present point coordinates
	 *  @return		A vector stored distances
	 */
	std::vector<double> calcDist(const std::vector<Point<T>>& p_list);

	/*! @brief		Determine if two points are overlapped
	 *  @param p	A @a Point object with @c T data type
	 *  @tparam T	The type of data used to present point coordinates
	 *  @return		true or false
	 */
	bool isCoincide(const Point<T>& p);

	//! print the point coordiante
	void printPointLoc();

	//! @protectedsection
protected:
	T x_;			/*!< x coordinate (@c float in this case) */
	T y_;			/*!< y coordinate (@c float in this case) */
};				//! End of class Point

#endif // !POINT_H_
