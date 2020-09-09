/*! @file point.cpp
 *
 *  @warning This is the internal cpp file of the ODP project.
 *  Do not use it directly in other code. Please note that this file
 *  is based on the open source code from
 *  <a href="https://github.com/achu6393/dynamicWeightedClustering">
 *	dynamicWeightedClustering
 *  </a>
 *  Copyright (C) Qiuchen Qian, 2020
 *  Imperial College, London
 */


#include <iostream>
#include "point.h"

template <class T>
Point<T>::Point() : x_(static_cast<T>(0)), y_(static_cast<T>(0)) {}

template <class T>
Point<T>::Point(float x, float y) : x_(static_cast<T>(x)), y_(static_cast<T>(y)) {}

template <class T>
Point<T>::~Point() {}

template <class T>
double Point<T>::calcDist(const Point<T>& p) {
	return std::hypot(this->x_ - p.x_, this->y_ - p.y_);
}

template <class T>
double Point<T>::calcDist(const float& x, const float& y) {
	return std::hypot(this->x_ - x, this->y_ - y);
}

template <class T>
std::vector<double> Point<T>::calcDist(const std::vector<Point<T>>& p_list) {
	std::vector<double> distances;
	for (const Point& this_p : p_list) {
		distances.push_back(calcDist(this_p));
	}
	return distances;
}

template <class T>
bool Point<T>::isCoincide(const Point<T>& p) {
	if (this->calcDist(p) <= 0.1) return true;
	return false;
}

template <class T>
void Point<T>::printPointLoc() {
	std::cout << "Coordinate: [ " << this->x_ << " , " << this->y_ << " ]"
		<< std::endl << std::endl;
}
