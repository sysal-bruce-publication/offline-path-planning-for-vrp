#include <set>
#include <random>
#include <chrono>
#include "funcs.h"

using namespace std;

template <class T>
int Funcs<T>::getRandIndex(int limit) {
	unsigned seed = static_cast<int>(chrono::system_clock::now().time_since_epoch().count());
	static default_random_engine generator_int(seed);

	uniform_int_distribution<int> distribution(0, limit);
	return distribution(generator_int);
}

template <class T>
float Funcs<T>::getRandFloat() {
	mt19937_64 rng;
	//! initialize the random number generator with time-dependent seed
	uint64_t timeSeed = chrono::high_resolution_clock::now().time_since_epoch().count();
	seed_seq ss{ uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed >> 32) };
	rng.seed(ss);
	//! initialize a uniform distribution between 0 and 1
	uniform_real_distribution<float> unif(0, 1);

	return unif(rng);
}

template <class T>
double Funcs<T>::tanhFunc(const double& x) {
	return 2. / (double)(1. + exp(-2. * x)) - 1.;
}

template <class T>
double Funcs<T>::invTanhFunc(const double& x) {
	return 1. - (2. / (double)(1. + exp(-2. * x)) - 1);
}

template <class T>
bool Funcs<T>::checkDuplicates(vector<int> source) {
	set<int> s(source.begin(), source.end());
	return s.size() != source.size();
}

template <class T>
void Funcs<T>::delDuplicates(Cluster<T>& cluster) {
	for (unsigned i = 0; i < cluster.contains.size() - 1; i++) {
		for (unsigned j = i + 1; j < cluster.contains.size(); j++) {
			//! If found two same points, delete one
			if (cluster.contains[i] == cluster.contains[j]) {
				cluster.contains.erase(cluster.contains.begin() + j);
				j--;
			}
		}
	}
}

template <class T>
void Funcs<T>::delDuplicates(vector<Point<T>>& p_list) {
	for (unsigned i = 0; i < p_list.size() - 1; i++) {
		for (unsigned j = i + 1; j < p_list.size(); j++) {
			if (p_list[i].isCoincide(p_list[j])) {
				p_list.erase(p_list.begin() + j);
				j--;
			}
		}
	}
}

template <class T>
void Funcs<T>::delDuplicates(vector<int>& idx_list) {
	for (unsigned i = 0; i < idx_list.size() - 1; i++) {
		for (unsigned j = i + 1; j < idx_list.size(); j++) {
			if (idx_list[i] == idx_list[j]) {
				idx_list.erase(idx_list.begin() + j);
				j--;
			}
		}
	}
}
