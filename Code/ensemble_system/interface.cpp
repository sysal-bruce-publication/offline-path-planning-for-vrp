#include <fstream>
#include <iomanip>
#include <iostream>
#include "interface.h"
#include "blackhole.h"
#include "annealing.h"
#include "pdv.h"
#include "genetic.h"

using namespace std;

template <class T>
Interface<T>::Interface() {
	interfaceIntro();
}

template <class T>
Interface<T>::~Interface() {}

template <class T>
void Interface<T>::interfaceInvalid(string message) {
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cerr << endl << endl
		<< ">\t ERROR:\t " << message
		<< endl << endl;
	system("pause");
}


template <class T>
void Interface<T>::resetParams() {
	this->alg_t = 0.;
	this->n_alg = -1;
}

template <class T>
void Interface<T>::interfaceIntro() {
	system("CLS");
	cout << endl << endl
		<< ">\t Optimal Drone Recharging Scheduling (ODRS) System"
		<< endl << endl
		<< ">\t\t Designed by Qiuchen Qian"
		<< endl << endl << endl;

	cout << endl << endl << endl
		<< ">\t The system is aimed at solving ODRS problem"
		<< endl << endl
		<< ">\t The optimization objective is to MAXIMIZE total recharged" << endl
		<< " \t energy and MINIMIZE energy cost of all PDVs"
		<< endl << endl
		<< ">\t Applied optimization algorithms:" << endl
		<< " \t Genetic Algorithm" << endl
		<< " \t Black Hole Algorithm" << endl
		<< " \t Simulated Annealing Algorithm"
		<< endl << endl;

	cout << endl << endl << endl
		<< ">\t Would you like to continue?"
		<< endl << endl
		<< " \t y: continue" << endl
		<< " \t n: exit" 
		<< endl << endl << ">\t INPUT:\t";

	string in_str;
	cin >> in_str;

	char in_char = in_str[0];
	if (in_str.size() != 1) in_char = '#';

	switch (in_char) {
	case 'y': resetParams(); break;
	case 'n': exit(0);
	default: interfaceInvalid("Invalid input !"); interfaceIntro(); break;
	}

	inputCheck();
}

template <class T>
void Interface<T>::readInputFromCsv(vector<SensorNode<T>>& input_sns) {
	system("CLS");
	cout << endl << endl
		<< ">\t Please enter the CSV file path of input sensor data:" << endl
		<< ">\t EXAMPLE:\t ../input/inputs.csv"
		<< endl << endl << ">\t INPUT:\t";

	fstream file;
	string in_path;
	cin >> in_path;

	file.open(in_path, fstream::in);
	if (!file) {
		interfaceInvalid("No such file or directory !");
		inputCheck();
	}

	string line;
	if (file.good()) getline(file, line);

	string xs, ys, flags, volts, weights;
	while (getline(file, xs, ',')) {
		getline(file, ys, ',');
		getline(file, flags, ',');
		getline(file, volts, ',');
		getline(file, weights);

		input_sns.push_back(SensorNode<T>(static_cast<float>(stof(xs) / 100.),
			static_cast<float>(stof(ys) / 100.), stof(volts), stoi(weights),
			static_cast<bool>(stoi(flags))));
	}
	file.close();
}


template <class T>
void Interface<T>::inputCheck() {
	this->sn_list.clear();
	readInputFromCsv(this->sn_list);

	system("CLS");
	cout << endl << endl
		<< ">\t Please enter the minimum voltage of pressure sensor:" << endl
		<< ">\t EXAMPLE:\t 3.0"
		<< endl << endl << ">\t INPUT:\t";
	double vmin_p = 0.;
	cin >> vmin_p;
	while (cin.fail() || vmin_p <= 0.) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input !"
			<< endl << endl << endl
			<< ">\t Please enter the minimum voltage of pressure sensor:" << endl
			<< ">\t EXAMPLE:\t 3.0"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> vmin_p;
	}

	cout << endl << endl << endl
		<< ">\t Please enter the maximum voltage of pressure sensor:" << endl
		<< ">\t EXAMPLE:\t 5.0"
		<< endl << endl << ">\t INPUT:\t";
	double vmax_p = 0.;
	cin >> vmax_p;
	while (cin.fail() || vmax_p <= 0. || vmax_p <= vmin_p) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input !"
			<< endl << endl << endl
			<< ">\t Please enter the maximum voltage of pressure sensor:" << endl
			<< ">\t EXAMPLE:\t 5.0"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> vmax_p;
	}

	cout << endl << endl << endl
		<< ">\t Please enter the minimum voltage of temperature sensor:" << endl
		<< ">\t EXAMPLE:\t 1.75"
		<< endl << endl << ">\t INPUT:\t";
	double vmin_t = 0.;
	cin >> vmin_t;
	while (cin.fail() || vmin_t <= 0.) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input !"
			<< endl << endl << endl
			<< ">\t Please enter the minimum voltage of temperature sensor:" << endl
			<< ">\t EXAMPLE:\t 1.75"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> vmin_t;
	}

	cout << endl << endl << endl
		<< ">\t Please enter the maximum voltage of temperature sensor:" << endl
		<< ">\t EXAMPLE:\t 2.5"
		<< endl << endl << ">\t INPUT:\t";
	double vmax_t = 0.;
	cin >> vmax_t;
	while (cin.fail() || vmax_t <= 0. || vmax_t <= vmin_t) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input !"
			<< endl << endl << endl
			<< ">\t Please enter the maximum voltage of temperature sensor:" << endl
			<< ">\t EXAMPLE:\t 2.5"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> vmax_t;
	}

	cout << endl << endl << endl
		<< ">\t Validating input data...";
	int n_rec = 0;

	for (unsigned i = 0; i < this->sn_list.size() - 1; i++) {
		for (unsigned j = i + 1; j < this->sn_list.size(); j++) {
			if (this->sn_list[i].pos.isCoincide(this->sn_list[j].pos)) {
				interfaceInvalid("Exist duplicates in sensor node deployment !");
				inputCheck();
			}
		}

		if (this->sn_list[i].p_sensor_type) {
			if (this->sn_list[i].SC_V < 0. || this->sn_list[i].SC_V > vmax_p) {
				interfaceInvalid("Sensor " + to_string(i + 1) + "\t Pressure sensor voltage out of range!");
				inputCheck();
			}

			if (this->sn_list[i].SC_V < vmin_p) {
				n_rec++;
			}
			
		}
		else {
			if (this->sn_list[i].SC_V < 0. || this->sn_list[i].SC_V > vmax_t) {
				interfaceInvalid("Sensor " + to_string(i + 1) + "\t Temperature sensor voltage out of range!");
				inputCheck();
			}

			if (this->sn_list[i].SC_V < vmin_t) {
				n_rec++;
			}
		}
	}

	cout << endl << endl << endl
		<< ">\t Pass. There are " << to_string(n_rec) 
		<< " sensor nodes to be recharged." << endl << endl;

	system("pause");

	interfaceSelectAlg();
}

template <class T>
void Interface<T>::interfaceSelectAlg() {
	system("CLS");
	cout << endl << endl
		<< ">\t Please select the optimization algorithm to be applied:"
		<< endl << endl
		<< " \t 1: Genetic Algorithm" << endl
		<< " \t 2: Black Hole Algorithm" << endl
		<< " \t 3: Simulated Annealing Algorithm" << endl
		<< " \t b: back" << endl
		<< " \t x: exit"
		<< endl << endl
		<< ">\t INPUT:\t";

	string in_str;
	cin >> in_str;

	char in_char = in_str[0];
	if (in_str.size() != 1) in_char = '#';

	switch (in_char) {
	case '1': this->n_alg = 0; interfaceGA(this->sn_list); break;
	case '2': this->n_alg = 1; interfaceBH(this->sn_list); break;
	case '3': this->n_alg = 2; interfaceSA(this->sn_list); break;
	case 'b': resetParams(); inputCheck(); break;
	case 'x': exit(0);
	default: interfaceInvalid("Invalid input !"); resetParams(); interfaceSelectAlg();  break;
	}
}

template <class T>
void Interface<T>::interfaceGA(vector<SensorNode<T>>& input_sns) {
	system("CLS");
	cout << endl << endl
		<< ">\t Please select the optimization mode:" << endl
		<< ">\t NOTICE:\t This should be decided by the optimization objective." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with shortest flight distance scheduling, select mode 1." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with minimum PDV energy cost scheduling, select mode 2." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with maximum total recharged energy, select mode 3." << endl
		<< endl << endl
		<< " \t 1: Prioritize shortest flight distances" << endl
		<< " \t 2: Prioritize minimum PDV energy cost" << endl
		<< " \t 3: Prioritize maximum recharged energy" << endl
		<< " \t 4: Balance mode (equal priority)" << endl
		<< " \t b: back" << endl
		<< " \t x: exit"
		<< endl << endl
		<< ">\t INPUT:\t";
	
	string in_str;
	cin >> in_str;

	char in_char = in_str[0];
	if (in_str.size() != 1) in_char = '#';

	int w_rec = 0, w_pdv = 0, w_dist = 0;

	switch (in_char) {
	case '1': w_rec = w_pdv = 15; w_dist = 70; break;
	case '2': w_rec = w_dist = 15; w_pdv = 70; break;
	case '3': w_pdv = w_dist = 15; w_rec = 70; break;
	case 'b': interfaceSelectAlg(); break;
	case 'x': exit(0);
	default: interfaceInvalid("Invalid input !"); interfaceGA(this->sn_list); break;
	}

	cout << endl << endl << endl
		<< ">\t Please select the level of computational cost of executing the algorithm:" << endl
		<< ">\t NOTICE:\t This decides the computational time of the algroithm." << endl
		<< ">\t NOTICE:\t With higher computational cost, the calculated solution MAY BE BETTER." << endl
		<< endl << endl
		<< " \t 1: Very low" << endl
		<< " \t 2: Low" << endl
		<< " \t 3: Medium" << endl
		<< " \t 4: High" << endl
		<< " \t 5: Very high"
		<< endl << endl
		<< ">\t INPUT:\t";

	int in_num;
	cin >> in_num;

	while (cin.fail() || in_num <= 0 || in_num >= 6) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input!"
			<< endl << endl << endl
			<< ">\t Please select the level of computational cost of executing the algorithm:"
			<< endl << endl
			<< " \t 1: Very low" << endl
			<< " \t 2: Low" << endl
			<< " \t 3: Medium" << endl
			<< " \t 4: High" << endl
			<< " \t 5: Very high"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> in_num;
	}

	int gen_num = 0, pop_num = 0;

	switch (in_num) {
	case 1: gen_num = 20; pop_num = 20; break;
	case 2: gen_num = 50; pop_num = 50; break;
	case 3: gen_num = 100; pop_num = 100; break;
	case 4: gen_num = 150; pop_num = 150; break;
	case 5: gen_num = 200; pop_num = 300; break;
	default: interfaceInvalid("Invalid input !"); interfaceGA(this->sn_list); break;
	}

	cout << endl << endl << endl
		<< ">\t Please select the randomness level of the algorithm:" << endl
		<< ">\t NOTICE:\t The level will decide the possibility to execute `crossover`."
		<< endl << endl 
		<< " \t 0: No crossover (possibility = 0%)" << endl
		<< " \t 1: Low (possibility = 25%)" << endl
		<< " \t 2: Medium (possibility = 50%)" << endl
		<< " \t 3: High (possibility = 75%)" << endl
		<< " \t 4: Crossover must happen (possibility = 100%)" 
		<< endl << endl << ">\t INPUT:\t";
	int in_num2 = 0;
	cin >> in_num2;
	while (cin.fail() || in_num2 < 0 || in_num2 >= 5) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input!"
			<< endl << endl << endl
			<< ">\t Please select the randomness of the algorithm:" << endl
			<< ">\t NOTICE:\t The level will decide the possibility to execute `crossover`."
			<< endl << endl
			<< " \t 0: No crossover (possibility = 0%)" << endl
			<< " \t 1: Low (possibility = 25%)" << endl
			<< " \t 2: Medium (possibility = 50%)" << endl
			<< " \t 3: High (possibility = 75%)" << endl
			<< " \t 4: Crossover must happen (possibility = 100%)"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> in_num2;
	}

	int cr_num = 0;
	switch (in_num2) {
	case 0: cr_num = 100; break;
	case 1: cr_num = 75; break;
	case 2: cr_num = 50; break;
	case 3: cr_num = 25; break;
	case 4: cr_num = -1; break;
	default: interfaceInvalid("Invalid input !"); interfaceGA(this->sn_list); break;
	}

	int rec_num = 30;
	int max_neigh = 5;

	cout << endl << endl
		<< ">\t Setting completed."
		<< endl << endl;

	system("pause");
	system("CLS");
	cout << endl << endl
		<< ">\t SYSTEM SETTINGS:" << endl << endl
		<< " \t Optimization mode:\t" << in_str << endl
		<< " \t\t Fitness metric factor for total recharged energy:\t"
		<< w_rec << " %" << endl
		<< " \t\t Fitness metric factor for energy cost of the PDV:\t"
		<< w_pdv << " %" << endl
		<< " \t\t Fitness metric factor for PDV flight distance:\t"
		<< w_dist << " %" << endl << endl
		<< " \t Computation level:\t" << to_string(in_num) << endl 
		<< " \t\t The number of generations:\t"
		<< gen_num << endl
		<< " \t\t The number of populations:\t"
		<< pop_num << endl << endl
		<< " \t Randomness level:\t" << to_string(in_num2) << endl
		<< " \t\t Crossover Ratio:\t"
		<< cr_num << " %" << endl << endl
		<< " \t Other default settings:" << endl
		<< " \t\t Minimum required number of sensors to be recharged:\t"
		<< rec_num << endl
		<< " \t\t Maximum number of neighbours to find:\t"
		<< max_neigh << endl << endl;

	system("pause");
	system("CLS");
	cout	<< ">\t Executing Genetic Algorithm ... (please wait)"
		<< endl << endl;

	Genetic<T>* ga = new Genetic<T>(w_rec, w_pdv, w_dist, gen_num, pop_num, cr_num, rec_num, max_neigh);
	if (!ga->checkTask(input_sns)) {
		delete ga;
		interfaceInvalid("No enough sensor nodes to be recharged !");
		inputCheck();
	}

	this->opt_sol = ga->best_sol;
	this->alg_t = ga->alg_time;
	delete ga;

	cout << ">\t Done." << endl << endl;
	system("pause");

	saveSubPathToCsv(this->n_alg, this->opt_sol);
}

template <class T>
void Interface<T>::interfaceBH(vector<SensorNode<T>>& input_sns) {
	system("CLS");
	cout << endl << endl
		<< ">\t Please select the optimization mode:" << endl
		<< ">\t NOTICE:\t This should be decided by the optimization objective." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with shortest flight distance scheduling, select mode 1." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with minimum PDV energy cost scheduling, select mode 2." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with maximum total recharged energy, select mode 3." << endl
		<< endl << endl
		<< " \t 1: Prioritize shortest flight distances" << endl
		<< " \t 2: Prioritize minimum PDV energy cost" << endl
		<< " \t 3: Prioritize maximum recharged energy" << endl
		<< " \t 4: Balance mode (equal priority)" << endl
		<< " \t b: back" << endl
		<< " \t x: exit"
		<< endl << endl
		<< ">\t INPUT:\t";

	string in_str;
	cin >> in_str;

	char in_char = in_str[0];
	if (in_str.size() != 1) in_char = '#';

	int w_rec = 0, w_pdv = 0, w_dist = 0;

	switch (in_char) {
	case '1': w_rec = w_pdv = 15; w_dist = 70; break;
	case '2': w_rec = w_dist = 15; w_pdv = 70; break;
	case '3': w_pdv = w_dist = 15; w_rec = 70; break;
	case 'b': interfaceSelectAlg(); break;
	case 'x': exit(0);
	default: interfaceInvalid("Invalid input !"); interfaceGA(this->sn_list); break;
	}

	cout << endl << endl << endl
		<< ">\t Please select the level of computational cost of executing the algorithm:" << endl
		<< ">\t NOTICE:\t This decides the computational time of the algroithm." << endl
		<< ">\t NOTICE:\t With higher computational cost, the calculated solution MAY BE BETTER." << endl
		<< endl << endl
		<< " \t 1: Very low" << endl
		<< " \t 2: Low" << endl
		<< " \t 3: Medium" << endl
		<< " \t 4: High" << endl
		<< " \t 5: Very high"
		<< endl << endl
		<< ">\t INPUT:\t";

	int in_num;
	cin >> in_num;

	while (cin.fail() || in_num <= 0 || in_num >= 6) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input!"
			<< endl << endl << endl
			<< ">\t Please select the level of computational cost of executing the algorithm:"
			<< endl << endl
			<< " \t 1: Very low" << endl
			<< " \t 2: Low" << endl
			<< " \t 3: Medium" << endl
			<< " \t 4: High" << endl
			<< " \t 5: Very high"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> in_num;
	}

	int gen_num = 0, pop_num = 0;

	switch (in_num) {
	case 1: gen_num = 20; pop_num = 20; break;
	case 2: gen_num = 50; pop_num = 50; break;
	case 3: gen_num = 100; pop_num = 100; break;
	case 4: gen_num = 150; pop_num = 150; break;
	case 5: gen_num = 200; pop_num = 300; break;
	default: interfaceInvalid("Invalid input !"); interfaceGA(this->sn_list); break;
	}

	cout << endl << endl << endl
		<< ">\t Please select the randomness level of the algorithm:" << endl
		<< ">\t NOTICE:\t The level will decide the possibility to execute `attraction`."
		<< endl << endl
		<< " \t 0: No crossover (possibility = 0%)" << endl
		<< " \t 1: Low (possibility = 25%)" << endl
		<< " \t 2: Medium (possibility = 50%)" << endl
		<< " \t 3: High (possibility = 75%)" << endl
		<< " \t 4: Crossover must happen (possibility = 100%)"
		<< endl << endl << ">\t INPUT:\t";
	int in_num2 = 0;
	cin >> in_num2;
	while (cin.fail() || in_num2 < 0 || in_num2 >= 5) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input!"
			<< endl << endl << endl
			<< ">\t Please select the randomness of the algorithm:" << endl
			<< ">\t NOTICE:\t The level will decide the possibility to execute `attraction`."
			<< endl << endl
			<< " \t 0: No attraction (possibility = 0%)" << endl
			<< " \t 1: Low (possibility = 25%)" << endl
			<< " \t 2: Medium (possibility = 50%)" << endl
			<< " \t 3: High (possibility = 75%)" << endl
			<< " \t 4: Attraction must happen (possibility = 100%)"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> in_num2;
	}

	int ar_num = 0;
	switch (in_num2) {
	case 0: ar_num = 100; break;
	case 1: ar_num = 75; break;
	case 2: ar_num = 50; break;
	case 3: ar_num = 25; break;
	case 4: ar_num = -1; break;
	default: interfaceInvalid("Invalid input !"); interfaceBH(this->sn_list); break;
	}

	int rec_num = 30;
	int max_neigh = 5;

	cout << endl << endl
		<< ">\t Setting completed."
		<< endl << endl;

	system("pause");
	system("CLS");

	cout << endl << endl 
		<< ">\t SYSTEM SETTINGS:" << endl << endl
		<< " \t Optimization mode:\t" << in_str << endl
		<< " \t\t Fitness metric factor for total recharged energy:\t"
		<< w_rec << " %" << endl
		<< " \t\t Fitness metric factor for energy cost of the PDV:\t"
		<< w_pdv << " %" << endl
		<< " \t\t Fitness metric factor for PDV flight distance:\t"
		<< w_dist << " %" << endl << endl
		<< " \t Computation level:\t" << to_string(in_num) << endl
		<< " \t\t The number of generations:\t"
		<< gen_num << endl
		<< " \t\t The number of populations:\t"
		<< pop_num << endl << endl
		<< " \t Randomness level:\t" << to_string(in_num2) << endl
		<< " \t\t Attraction Ratio:\t"
		<< ar_num << " %" << endl << endl
		<< " \t Other default settings:" << endl
		<< " \t\t Minimum required number of sensors to be recharged:\t"
		<< rec_num << endl
		<< " \t\t Maximum number of neighbours to find:\t"
		<< max_neigh << endl << endl;

	system("pause");

	cout << endl << endl << endl
		<< ">\t Executing Black Hole Algorithm ... (please wait)"
		<< endl << endl;

	BlackHole<T>* bh = new BlackHole<T>(w_rec, w_pdv, w_dist, gen_num, pop_num, ar_num, rec_num, max_neigh);
	if (!bh->checkTask(input_sns)) {
		delete bh;
		interfaceInvalid("No enough sensor nodes to be recharged !");
		inputCheck();
	}

	this->opt_sol = bh->best_sol;
	this->alg_t = bh->alg_time;
	delete bh;

	cout << ">\t Done." << endl << endl;
	system("pause");

	saveSubPathToCsv(this->n_alg, this->opt_sol);
}

template <class T>
void Interface<T>::interfaceSA(vector<SensorNode<T>>& input_sns) {
	system("CLS");
	cout << endl << endl
		<< ">\t Please select the optimization mode:" << endl
		<< ">\t NOTICE:\t This should be decided by the optimization objective." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with shortest flight distance scheduling, select mode 1." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with minimum PDV energy cost scheduling, select mode 2." << endl
		<< ">\t NOTICE:\t To calculate optimal solution with maximum total recharged energy, select mode 3." << endl
		<< endl << endl
		<< " \t 1: Prioritize shortest flight distances" << endl
		<< " \t 2: Prioritize minimum PDV energy cost" << endl
		<< " \t 3: Prioritize maximum recharged energy" << endl
		<< " \t 4: Balance mode (equal priority)" << endl
		<< " \t b: back" << endl
		<< " \t x: exit"
		<< endl << endl
		<< ">\t INPUT:\t";

	string in_str;
	cin >> in_str;

	char in_char = in_str[0];
	if (in_str.size() != 1) in_char = '#';

	int w_rec = 0, w_pdv = 0, w_dist = 0;

	switch (in_char) {
	case '1': w_rec = w_pdv = 15; w_dist = 70; break;
	case '2': w_rec = w_dist = 15; w_pdv = 70; break;
	case '3': w_pdv = w_dist = 15; w_rec = 70; break;
	case 'b': interfaceSelectAlg(); break;
	case 'x': exit(0);
	default: interfaceInvalid("Invalid input !"); interfaceGA(this->sn_list); break;
	}

	cout << endl << endl << endl
		<< ">\t Please select the level of computational cost of executing the algorithm:" << endl
		<< ">\t NOTICE:\t This decides the computational time of the algroithm." << endl
		<< ">\t NOTICE:\t With higher computational cost, the calculated solution MAY BE BETTER." << endl
		<< endl << endl
		<< " \t 1: Very low" << endl
		<< " \t 2: Low" << endl
		<< " \t 3: Medium" << endl
		<< " \t 4: High" << endl
		<< " \t 5: Very high"
		<< endl << endl
		<< ">\t INPUT:\t";

	int in_num;
	cin >> in_num;

	while (cin.fail() || in_num <= 0 || in_num >= 6) {
		cerr << endl << endl
			<< ">\t ERROR:\t Invalid input!"
			<< endl << endl << endl
			<< ">\t Please select the level of computational cost of executing the algorithm:"
			<< endl << endl
			<< " \t 1: Very low" << endl
			<< " \t 2: Low" << endl
			<< " \t 3: Medium" << endl
			<< " \t 4: High" << endl
			<< " \t 5: Very high"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> in_num;
	}

	double init_temp = 0., min_temp = 0., temp_factor = 0.;
	int pop_num = 0;

	switch (in_num) {
	case 1: init_temp = 1e4; min_temp = 1e-5; temp_factor = 0.95; pop_num = 10; break;
	case 2: init_temp = 3e4; min_temp = 1e-5; temp_factor = 0.96; pop_num = 20; break;
	case 3: init_temp = 5e4; min_temp = 1e-5; temp_factor = 0.97; pop_num = 30; break;
	case 4: init_temp = 7e4; min_temp = 1e-5; temp_factor = 0.98; pop_num = 40; break;
	case 5: init_temp = 1e5; min_temp = 1e-5; temp_factor = 0.99; pop_num = 50; break;
	default: interfaceInvalid("Invalid input !"); interfaceSA(this->sn_list); break;
	}

	int rec_num = 30;
	int max_neigh = 5;

	cout << endl << endl
		<< ">\t Setting completed."
		<< endl << endl;

	system("pause");
	system("CLS");

	cout << endl << endl
		<< ">\t SYSTEM SETTINGS:" << endl << endl
		<< " \t Optimization mode:\t" << in_str << endl
		<< " \t\t Fitness metric factor for total recharged energy:\t"
		<< w_rec << " %" << endl
		<< " \t\t Fitness metric factor for energy cost of the PDV:\t"
		<< w_pdv << " %" << endl
		<< " \t\t Fitness metric factor for PDV flight distance:\t"
		<< w_dist << " %" << endl << endl
		<< " \t Computation level:\t" << to_string(in_num) << endl
		<< " \t\t Initial starting temperature:\t"
		<< init_temp << endl
		<< " \t\t Minimal allowed temperature:\t"
		<< min_temp << endl 
		<< " \t\t Temperature reducing factor:\t"
		<< temp_factor << endl
		<< " \t\t The number of populations:\t"
		<< pop_num << endl << endl
		<< " \t Other default settings:" << endl
		<< " \t\t Minimum required number of sensors to be recharged:\t"
		<< rec_num << endl
		<< " \t\t Maximum number of neighbours to find:\t"
		<< max_neigh << endl << endl;

	system("pause");

	cout << endl << endl << endl
		<< ">\t Executing Simulated Annealing Algorithm ... (please wait)"
		<< endl << endl;

	Annealing<T>* sa = new Annealing<T>(w_rec, w_pdv, w_dist, init_temp, min_temp, temp_factor, pop_num, rec_num, max_neigh);
	if (!sa->checkTask(input_sns)) {
		delete sa;
		interfaceInvalid("No enough sensor nodes to be recharged !");
		inputCheck();
	}

	this->opt_sol = sa->best_sol;
	this->alg_t = sa->alg_time;
	delete sa;

	cout << ">\t Done." << endl << endl;
	system("pause");

	saveSubPathToCsv(this->n_alg, this->opt_sol);
}

template <class T>
void Interface<T>::saveSubPathToCsv(int alg_num, vector<vector<int>> out_paths) {
	cout << endl << endl << endl
		<< ">\t Would you like to save assigned sub paths of each PDV ?" << endl
		<< ">\t NOTICE:\t Sub paths files can be used for further result visualization part."
		<< endl << endl
		<< " \t y: save file to specific directory" << endl
		<< " \t n: skip" << endl
		<< " \t b: back to algorithm selection page" << endl
		<< " \t x: exit"
		<< endl << endl
		<< ">\t INPUT:\t";

	string in_str;
	cin >> in_str;

	char in_char = in_str[0];
	if (in_str.size() != 1) in_char = '#';

	switch (in_char) {
	case 'y': break;
	case 'n': PdvFlightSimulation(this->opt_sol); break;
	case 'b': resetParams(); interfaceSelectAlg(); break;
	case 'x': exit(0);
	default: interfaceInvalid("Invalid input !"); saveSubPathToCsv(this->n_alg, this->opt_sol); break;
	}

	cout << endl << endl << endl
		<< ">\t Save calculated best solution for further use." << endl
		<< ">\t Please enter the path to store optimal sub-paths." << endl
		<< ">\t EXAMPLE:\t ../output/sub_path/"
		<< endl << endl << ">\t INPUT:\t";

	string out_dir;
	cin >> out_dir;

	for (unsigned i = 0; i < this->opt_sol.size(); i++) {
		fstream file;
		if (alg_num == 0) {
			file.open(out_dir + "ga_path" + to_string(i) + ".csv", fstream::out);
			if (!file) {
				interfaceInvalid("When saving ga_path" + to_string(i) + ".csv found ERROR:\t No such file or directory !");
				saveSubPathToCsv(this->n_alg, this->opt_sol);
			}
		}
		else if (alg_num == 1) {
			file.open(out_dir + "bh_path" + to_string(i) + ".csv", fstream::out);
			if (!file) {
				interfaceInvalid("When saving bh_path" + to_string(i) + ".csv found ERROR:\t No such file or directory !");
				saveSubPathToCsv(this->n_alg, this->opt_sol);
			}
		}
		else {
			file.open(out_dir + "sa_path" + to_string(i) + ".csv", fstream::out);
			if (!file) {
				interfaceInvalid("When saving sa_path" + to_string(i) + ".csv found ERROR:\t No such file or directory !");
				saveSubPathToCsv(this->n_alg, this->opt_sol);
			}
		}

		file << "x_pos,y_pos" << endl;
		for (unsigned j = 0; j < out_paths[i].size(); j++) {
			file << this->sn_list[out_paths[i][j]].pos.getX() << ","
				<< this->sn_list[out_paths[i][j]].pos.getY() << endl;
		}
		file.close();
	}

	PdvFlightSimulation(this->opt_sol);
}

template <class T>
void Interface<T>::PdvFlightSimulation(vector<vector<int>> out_paths) {
	system("CLS");
	cout << endl << endl
		<< ">\t Please select the recharging strategy of flight simulation." << endl
		<< ">\t NOTICE:\t There are two types of power transfer approaches." << endl
		<< ">\t NOTICE:\t Inductive Power Transfer (IPT) & Acoustic Power Transfer (APT)." << endl
		<< ">\t NOTICE:\t In sparse network, only performing IPT will be more efficient." << endl
		<< ">\t NOTICE:\t In dense network, performing both IPT and APT will be more efficient."
		<< endl << endl
		<< " \t 1: Only perform Inductive Power Transfer." << endl
		<< " \t 2: Perform Inductive Power Transfer and Acoustic Power Transfer." << endl
		<< " \t b: back to path saving page" << endl
		<< " \t x: exit"
		<< endl << endl
		<< ">\t INPUT:\t";

	string in_str;
	cin >> in_str;

	bool flag2 = false;

	char in_char = in_str[0];
	if (in_str.size() != 1) in_char = '#';

	switch (in_char) {
	case '1': flag2 = false; break;
	case '2': flag2 = true; break;
	case 'b': saveSubPathToCsv(this->n_alg, this->opt_sol); break;
	case 'x': exit(0);
	default: interfaceInvalid("Invalid input !"); PdvFlightSimulation(this->opt_sol); break;
	}
	
	int n_pdv = out_paths.size();
	vector<vector<Point<T>>> final_sols;
	final_sols.resize(n_pdv);
	int n_sns = 0;
	for (int i = 0; i < n_pdv; i++) {
		n_sns = out_paths[i].size();
		final_sols[i].resize(n_sns);
		for (unsigned j = 0; j < out_paths[i].size(); j++) {
			final_sols[i][j] = this->sn_list[out_paths[i][j]].pos;
		}
	}

	float* pect_list = new float[n_pdv];
	double* pdv_eng_cost = new double[n_pdv];
	double* delta_wsn_eng = new double[n_pdv];
	double* flight_ds = new double[n_pdv];
	double* flight_time = new double[n_pdv];

	for (int i = 0; i < n_pdv; i++) {
		pect_list[i] = (float)(0.);
		pdv_eng_cost[i] = 0.;
		delta_wsn_eng[i] = 0.;
		flight_ds[i] = 0.;
		flight_time[i] = 0.;

		unique_ptr<PDV<T>> pdv = make_unique<PDV<T>>();
		float pect = 0.;
		if (flag2) {
			pect = pdv->flightSimulation(delta_wsn_eng[i], flight_time[i], this->sn_list, final_sols[i]);
		}
		else {
			pect = pdv->singleStageFlight(delta_wsn_eng[i], flight_time[i], this->sn_list, final_sols[i]);

		}
		pect_list[i] = pect;
		pdv_eng_cost[i] = 187. - pdv->getPdvEnergy();
		flight_ds[i] = pdv->getPdvDistance();
	}

	double max_t = flight_time[0];
	for (int i = 1; i < n_pdv; i++) {
		if (flight_time[i] > max_t) max_t = flight_time[i];
	}

	//! Update all Sensor Energy
	for (unsigned i = 0; i < this->sn_list.size(); i++) {
		if (sn_list[i].SC_V > this->sn_list[i].getCriticalVolt()) {
			this->sn_list[i].updateEnergy(max_t * 3600, sn_list[i].SC_E);
			this->sn_list[i].updateVolt(sn_list[i].SC_V);
			this->sn_list[i].updateWeight(sn_list[i].SC_V, sn_list[i].weight);
		}
	}

	cout << endl << endl
		<< ">\t Flight simulation completed !" 
		<< endl << endl;
	system("pause");

	saveOutputToCsv(n_pdv, pect_list, pdv_eng_cost, delta_wsn_eng, flight_ds, this->alg_t);
	delete[] pect_list;
	delete[] pdv_eng_cost;
	delete[] delta_wsn_eng;
	delete[] flight_ds;
	delete[] flight_time;
}

template <class T>
void Interface<T>::saveOutputToCsv(int pdv_num, float* pct_list, double* pdv_eng_cost, 
	double* charged_eng, double* flight_ds, double spent_t) {
	system("CLS");
	cout << endl << endl
		<< ">\t Please enter the CSV file path of output sensor data:" << endl
		<< ">\t EXAMPLE:\t ../output/updated_sns.csv"
		<< endl << endl << ">\t INPUT:\t";

	string out_sns_dir;
	cin >> out_sns_dir;

	fstream file;
	do {
		file.open(out_sns_dir, fstream::out);
		if (file) break;

		cerr << endl << endl
			<< ">\t ERROR:\t No such file or directory !"
			<< endl << endl << endl
			<< ">\t Please enter the CSV file path of output sensor data:" << endl
			<< ">\t EXAMPLE:\t ../output/updated_sns.csv"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> out_sns_dir;
	} while (!file);

	cout << endl << endl
		<< ">\t Writing...";
	file << "x_pos,y_pos,p_flag,volts,weights" << endl;
	for (unsigned i = 0; i < this->sn_list.size(); i++) {
		file << sn_list[i].pos.getX() << ","
			<< sn_list[i].pos.getY() << ","
			<< sn_list[i].p_sensor_type << ","
			<< sn_list[i].SC_V << ","
			<< sn_list[i].weight << endl;
	}
	file.close();
	cout << endl << endl << ">\t Saved !";

	cout << endl << endl << endl
		<< ">\t Please enter the CSV file path of recharging summary:" << endl
		<< ">\t EXAMPLE:\t ../output/rec_summary.csv"
		<< endl << endl << ">\t INPUT:\t";

	string out_rec_dir;
	cin >> out_rec_dir;

	do {
		file.open(out_rec_dir, fstream::out);
		if (file) break;

		cerr << endl << endl
			<< ">\t ERROR:\t No such file or directory !"
			<< endl << endl << endl
			<< ">\t Please enter the CSV file path of recharging summary." << endl
			<< ">\t EXAMPLE:\t ../output/rec_summary.csv"
			<< endl << endl << ">\t INPUT:\t";
		cin.clear();
		cin.ignore(numeric_limits<streamsize>::max(), '\n');
		cin >> out_rec_dir;
	} while (!file);

	cout << endl << endl
		<< ">\t Writing...";
	file << "ID,flight_distance,throughput,pdv_energy_cost,total_charged_energy" << endl;
	for (int i = 0; i < pdv_num; i++) {
		file << i << "," << flight_ds[i] << "," << pct_list[i]
			<< "," << pdv_eng_cost[i] << "," << charged_eng[i] << endl;
	}
	file << "alg_time," << spent_t << endl;
	file.close();

	cout << endl << endl << ">\t Saved !" << endl << endl;
	system("pause");

	interfaceIntro();
}