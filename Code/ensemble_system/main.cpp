/*! @file	main.cpp
*
*   @warning This is the internal header of the ODP project.
*   Do not use it directly in other code. 
*
*   Copyright (C) Qiuchen Qian, 2020
*   Imperial College, London
*   Users should ensure that there are no overlapped sensor nodes.
*
*   @pre Total number of sensor nodes @a NODE_NUM needs to be set at the beginning. \\
*	  C++ language version ISO C++17. \\
*	  Set SDL check to NO.
*/

#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>

//! Uncomment following code for user interface usage
//#include "interface.h"

//! Uncomment following code for ensemble test
#include "cases.h"
#include "cases.cpp"
#include "point.cpp"
#include "sensornode.cpp"
#include "interface.cpp"
#include "genetic.cpp"
#include "blackhole.cpp"
#include "annealing.cpp"
#include "pdv.cpp"
#include "funcs.cpp"

using namespace std;

int main() {
	//! Uncomment following codes for user interface usage
	//auto* ui = new Interface<double>();
	//delete ui;

	//! Uncomment following codes for ensemble test
	auto* cases = new Cases<double>();
	
	//！PDV number test
	//cases->single_test(5);
	//cases->single_test(8);

	cases->ensemble_test();
	delete cases;

	return 0;
}