#include "pch.h"
#include "CppUnitTest.h"
#include "../iterated_GA/point.h"
#include "../iterated_GA/sensornode.h"
#include "../iterated_GA/pdv.h"
#include "../iterated_GA/genetic.h"
#include "../iterated_GA/point.cpp"
#include "../iterated_GA/sensornode.cpp"
#include "../iterated_GA/pdv.cpp"
#include "../iterated_GA/genetic.cpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace unittestga
{
	TEST_CLASS(unittestga)
	{
	public:
		
		TEST_CLASS_INITIALIZE(TestClassInit) {
			//! Test @a Point constructor
			float x = 0.;
			float y = 0.;
			Point<float> test_p(x, y);
			Assert::AreEqual(x, test_p.getX());
			Assert::AreEqual(x, test_p.getY());

			//! Test @a SensorNode constructor
			double v = 3.4;
			int w = 10;
			bool p_type = true;
			SensorNode<float> test_sn(x, y, v, w, p_type);
			Assert::AreEqual(x, test_sn.pos.getX());
			Assert::AreEqual(y, test_sn.pos.getY());
			Assert::AreEqual(v, test_sn.SC_V);
			Assert::AreEqual(w, test_sn.weight);
			Assert::AreEqual(p_type, test_sn.p_sensor_type);

			//! Test @a PDV constructor
			PDV<float> test_pdv;
			Assert::AreEqual(x, test_pdv.pos.getX());
			Assert::AreEqual(y, test_pdv.pos.getY());
			Assert::AreEqual(0., test_pdv.getPdvTime());
			Assert::AreEqual(187., test_pdv.getPdvEnergy());
			Assert::AreEqual(0., test_pdv.getPdvDistance());

			//! Test @a Genetic constructor
			int pop_num = 50;
			Genetic<float> test_ga(pop_num);
			Assert::AreEqual(pop_num, static_cast<int>(test_ga.tars_int.size()));
			Assert::AreEqual(pop_num, static_cast<int>(test_ga.trail_int.size()));
			Assert::AreEqual(x, test_ga.origin->getX());
			Assert::AreEqual(y, test_ga.origin->getY());
			Assert::AreEqual(0., test_ga.alg_time);
			Assert::IsTrue(test_ga.targets_metric.empty());
			Assert::IsTrue(test_ga.trails_metric.empty());
		}

		TEST_METHOD(TestPointMethod) {
			//! Test if can calculate correct distance between two points
			float x1 = 1., x2 = 2., x3 = 4., x4= 6.;
			Point<float> test_p1(x1, x2);
			Point<float> test_p2(x3, x4);
			double test_d = test_p1.calcDist(test_p2);
			Assert::AreEqual(5., test_d);
		}

		TEST_METHOD(TestSensorNodeMethod) {
			SensorNode<float> test_sn;
			test_sn.SC_V -= 0.1;

			//! Test if can update energy correctly
			test_sn.updateEnergy(test_sn.SC_E);
			Assert::IsTrue(abs(test_sn.SC_E - 16.335) <= 1e-10);

			//! Test if can update voltage correctly
			test_sn.updateVolt(test_sn.SC_V);
			Assert::IsTrue(abs(test_sn.SC_V - 3.3) <= 1e-10);
		}

		TEST_METHOD(TestPdvMethod) {
			PDV<float> test_pdv;

			//! Test if can generate random offset correctly
			float test_offset1 = test_pdv.getRandOffset();
			Assert::IsTrue(test_offset1 < 1 && test_offset1 > -1);

			//! Test if can calculate time and energy of asecent correctly
			double test_t = 0., test_e = 0.;
			test_pdv.ascentEnergyCost(test_t, test_e);
			Assert::IsTrue(abs(test_t - 0.0012345678901) <= 1e-10);
			Assert::IsTrue(abs(test_e - 0.4492444444444) <= 1e-10);

			//! Test if can calculate time and energy of GPS approaching correctly
			float x1 = 10., y1 = 10.;
			test_t = test_e = 0.;
			test_pdv.gpsEnergyCost(0.5, Point<float>(x1, y1), test_t, test_e);
			Assert::IsTrue(abs(test_t - 2.1491368628384e-4) <= 1e-10);
			Assert::IsTrue(abs(test_e - 0.07820451147445409) <= 1e-10);

			//! Test if can update position correctly
			test_pdv.updatePos(Point<float>(x1, y1));
			Assert::AreEqual(x1, test_pdv.pos.getX());
			Assert::AreEqual(y1, test_pdv.pos.getY());

			//! Test if can implement IPT correctly
			test_e = 0.;
			SensorNode<float> test_sn = SensorNode<float>(100., 100., 3.2134, 3, true);
			test_pdv.iptEnergyCost(test_sn, test_e);
			Assert::IsTrue(abs(test_e - 0.0122283837) <= 1e-10);

			//! Test if can simulate flight correctly
			Point<float> test_p1 = Point<float>(100., 100.);
			Point<float> test_p2 = Point<float>(200., 200.);
			Point<float> test_p3 = Point<float>(300., 300.);
			vector<Point<float>> test_path = { test_p1, test_p2, test_p3 };

			SensorNode<float> test_sn1 = SensorNode<float>(100., 100., 3.2134, 3, true);
			SensorNode<float> test_sn2 = SensorNode<float>(200., 200., 1.1221, 5, false);
			SensorNode<float> test_sn3 = SensorNode<float>(300., 300., 3.1109, 3, true);
			SensorNode<float> test_sn4 = SensorNode<float>(400., 400., 2.3333, 10, false);
			vector<SensorNode<float>> test_sn_list = { test_sn1, test_sn2, test_sn3, test_sn4 };
			test_t = test_e = 0.;
			float throughput = 0.;
			test_pdv.resetPdvStatus();
			throughput = test_pdv.flightSimulation(test_e, test_t, test_sn_list, test_path);
			Assert::AreEqual(static_cast<float>(100.), throughput);
			Assert::IsTrue(abs(test_e - 59.967217215) <= 1e-10);
			Assert::IsTrue(abs(test_t - 0.19250130264737733) <= 1e-2);
			Assert::IsTrue(abs(test_pdv.getPdvDistance() - 888.5281374238571) <= 1e-5);
			Assert::IsTrue(abs(test_pdv.getPdvEnergy() - 171.50097085752446) <= 1e-1);
		}

		TEST_METHOD(TestGeneticMethod) {
			//! Test if can calculate minimum required guess
			Point<float> test_p1 = Point<float>(100., 100.);
			Point<float> test_p2 = Point<float>(200., 200.);
			Point<float> test_p3 = Point<float>(300., 300.);
			vector<Point<float>> test_path = { test_p1, test_p2, test_p3 };

			SensorNode<float> test_sn1 = SensorNode<float>(100., 100., 3.2134, 3, true);
			SensorNode<float> test_sn2 = SensorNode<float>(200., 200., 1.1221, 5, false);
			SensorNode<float> test_sn3 = SensorNode<float>(300., 300., 3.1109, 3, true);
			SensorNode<float> test_sn4 = SensorNode<float>(400., 400., 2.3333, 10, false);
			vector<SensorNode<float>> test_sn_list = { test_sn1, test_sn2, test_sn3, test_sn4 };
			vector<SensorNode<float>*> test_sn_ptr_list = { &test_sn1, &test_sn2, &test_sn3 };

			Genetic<float> test_ga;

			int pdv_num = test_ga.calcOptPdvNum(test_sn_list, test_sn_ptr_list, test_path);
			Assert::AreEqual(1, pdv_num);
			
			//! Test if can implement crossover correctly
			int pop_num = 3;
			vector<vector<vector<int>>> test_tar_vec = { { {0, 1, 2} }, { {0, 2, 1} }, { {1, 2, 0} } };
			vector<vector<vector<int>>> test_trail_vec = { { {-1, -1, -1} }, { {-1, -1, -1} }, { {-1, -1, -1} } };
			test_ga.crossover(0, pop_num, true, test_tar_vec, test_trail_vec, test_sn_list);	
			for (unsigned i = 0; i < test_trail_vec.size(); i++) {
				for (unsigned j = 0; j < test_trail_vec[i][0].size(); j++) {
					Assert::IsTrue(test_trail_vec[i][0][j] < 4 && test_trail_vec[i][0][j] >= 0);
				}
			}

			//! Test if can implement fitness function correctly
			double test_tar_fit = test_ga.fitnessFunc(test_sn_list, test_tar_vec[0][0]);
			Assert::IsTrue(abs(test_tar_fit - 78.48915598283966) <= 5.);
		}
	};
}
