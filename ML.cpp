/**
 * CMA-ES, Covariance Matrix Adaptation Evolution Strategy
 * Copyright (c) 2014 Inria
 * Author: Emmanuel Benazera <emmanuel.benazera@lri.fr>
 *
 * This file is part of libcmaes.
 *
 * libcmaes is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * libcmaes is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libcmaes.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cmaes.h"
#include "cmastrategy.h"
#include <fstream>
#include <iostream>
#include <string>
#include <set>
#include <map>
#include <stack>
#include <queue>
#include <vector>
#include <utility>
#include <iomanip>
#include <sstream>
#include <bitset>
#include <cstdlib>
#include <iterator>
#include <algorithm>
#include <streambuf>
/// C Header Files
#include <cstdio>
#include <cctype>
#include <cmath>
#include <fstream>
#include <math.h>
#include <ctime>
#include <cstring>

using namespace libcmaes;
using namespace std;
const double PI = 3.14159265359;
const char BASH[] = "./start_all.sh";
const int lambda = 160;
double sigma = 0.5;
const int dim = 23;
int MAITER = 200;
ofstream ofs;

#define SET(a)                memset( a, -1,    sizeof a )
#define CLR(a)                memset( a,  0,    sizeof a )
#define MEM(a,val)            memset( a,  val,  sizeof(a) )
#define pb                    push_back
#define all(x)                (x).begin(), (x).end()
#define sz(a)                 int((a).size())
#define ff                    first
#define ss                    second
#define mp                    make_pair
#define nl                    "\n"
double rad2deg(double rad) {
	return rad * 180.0 / PI;
}
double deg2rad(double deg) {
	return deg * PI / 180.0;
}
string toString(double x) {
	stringstream ss;
	ss << setprecision(12) << fixed << x;
	return ss.str();
}
string toString(int x) {
	stringstream ss;
	ss << x;
	return ss.str();

}
void GenerateParamFile(const double *params, int i, int j, const string PATH =
		"") {
	vector<string> ret;
	double utwalk_max_step_size_angle, utwalk_max_step_size_x,
			utwalk_max_step_size_y, utwalk_shift_amount, utwalk_walk_height,
			utwalk_fraction_still, utwalk_fraction_on_ground,
			utwalk_step_height, utwalk_phase_length,
			//next three are equal
			utwalk_pid_step_size_x, utwalk_pid_step_size_y,
			utwalk_pid_step_size_rot,
			//equality done
			utwalk_max_normal_com_error, utwalk_max_acceptable_com_error,
			utwalk_fwd_offset, utwalk_fwd_offset_factor,
			utwalk_swing_ankle_offset, utwalk_pid_tilt, utwalk_pid_roll,
			utwalk_toe_amplitude, utwalk_toe_phase_offset,
			utwalk_ankle_const_offset, utwalk_ankle_amplitude,
			utwalk_ankle_phase_offset, utwalk_toe_const_offset;
//			utwalk_pid_com_x, utwalk_pid_com_y, utwalk_pid_com_z,
//			utwalk_pid_arm_x, utwalk_pid_arm_y;

	utwalk_max_step_size_angle = params[0];
	utwalk_max_step_size_x = rad2deg(params[1]);
	utwalk_max_step_size_y = rad2deg(params[2]);
	utwalk_shift_amount = rad2deg(params[3]);
	utwalk_step_height = rad2deg(params[4]);
	utwalk_fraction_still = params[5];
	utwalk_fraction_on_ground = params[6];
	utwalk_walk_height = rad2deg(params[7]);
	utwalk_phase_length = params[8];
	//next three are equal NO THEY ARENT
	utwalk_pid_step_size_x = params[9];
	utwalk_pid_step_size_y = params[10];
	utwalk_pid_step_size_rot = params[11];
	//equality done  YES IT IS
	utwalk_max_normal_com_error = rad2deg(params[12]);
	utwalk_max_acceptable_com_error = rad2deg(params[13]);
	utwalk_fwd_offset = rad2deg(params[14]);
	utwalk_fwd_offset_factor = params[15];
	utwalk_swing_ankle_offset = params[16];
	//type4
	utwalk_toe_const_offset = params[17];
	utwalk_toe_amplitude = params[18];
	utwalk_toe_phase_offset = params[19];
	utwalk_ankle_const_offset = params[20];
	utwalk_ankle_amplitude = params[21];
	utwalk_ankle_phase_offset = params[22];

//	utwalk_pid_tilt = params[23];
//	utwalk_pid_roll = params[24];
//	utwalk_pid_com_x = params[19];
//	utwalk_pid_com_y = params[20];
//	utwalk_pid_com_z = params[21];
	//calculations

	double utwalk_fraction_in_air = abs(1 - utwalk_fraction_on_ground);
	double utwalk_fraction_moving = abs(1 - utwalk_fraction_still);
	string h = "utwalk_max_step_size_angle\t";
	h += toString(utwalk_max_step_size_angle);
	ret.pb(h);
	h = "utwalk_max_step_size_x\t";
	h += toString(utwalk_max_step_size_x);
	ret.pb(h);
	h = "utwalk_max_step_size_y\t";
	h += toString(utwalk_max_step_size_y);
	ret.pb(h);
	h = "utwalk_shift_amount\t";
	h += toString(utwalk_shift_amount);
	ret.pb(h);
	h = "utwalk_walk_height\t";
	h += toString(utwalk_walk_height);
	ret.pb(h);
	h = "utwalk_step_height\t";
	h += toString(utwalk_step_height);
	ret.pb(h);
	h = "utwalk_fraction_still\t";
	h += toString(utwalk_fraction_still);
	ret.pb(h);
	h = "utwalk_fraction_on_ground\t";
	h += toString(utwalk_fraction_on_ground);
	ret.pb(h);
	h = "utwalk_phase_length\t";
	h += toString(utwalk_phase_length);
	ret.pb(h);
	//next three are equalGenerateParamFile
	h = "utwalk_pid_step_size_x\t";
	h += toString(utwalk_pid_step_size_x);
	ret.pb(h);
	h = "utwalk_pid_step_size_y\t";

	h += toString(utwalk_pid_step_size_y);
	ret.pb(h);
	h = "utwalk_pid_step_size_rot\t";
	h += toString(utwalk_pid_step_size_rot);
	ret.pb(h);
	//equality done
	h = "utwalk_max_normal_com_error\t";
	h += toString(utwalk_max_normal_com_error);
	ret.pb(h);
	h = "utwalk_max_acceptable_com_error\t";
	h += toString(utwalk_max_acceptable_com_error);
	ret.pb(h);
	h = "utwalk_fwd_offset\t";
	h += toString(utwalk_fwd_offset);
	ret.pb(h);
	h = "utwalk_fwd_offset_factor\t";
	h += toString(utwalk_fwd_offset_factor);
	ret.pb(h);
	h = "utwalk_fraction_in_air\t";
	h += toString(utwalk_fraction_in_air);
	ret.pb(h);
	h = "utwalk_fraction_moving\t";
	h += toString(utwalk_fraction_moving);
	ret.pb(h);
	h = "utwalk_swing_ankle_offset\t";
	h += toString(utwalk_swing_ankle_offset);
	ret.pb(h);
//	h = "utwalk_pid_tilt\t";
//	h += toString(utwalk_pid_tilt);
//	ret.pb(h);
//	h = "utwalk_pid_roll\t";
//	h += toString(utwalk_pid_roll);
//	ret.pb(h);
//	h = "utwalk_pid_roll\t";
//	h += toString(utwalk_pid_roll);
//	ret.pb(h);

	h = "utwalk_toe_const_offset\t";
	h += toString(utwalk_toe_const_offset);
	ret.pb(h);

	h = "utwalk_toe_amplitude\t";
	h += toString(utwalk_toe_amplitude);
	ret.pb(h);

	h = "utwalk_toe_phase_offset\t";
	h += toString(utwalk_toe_phase_offset);
	ret.pb(h);

	h = "utwalk_ankle_const_offset\t";
	h += toString(utwalk_ankle_const_offset);
	ret.pb(h);

	h = "utwalk_ankle_amplitude\t";
	h += toString(utwalk_ankle_amplitude);
	ret.pb(h);

	h = "utwalk_ankle_phase_offset\t";
	h += toString(utwalk_ankle_phase_offset);
	ret.pb(h);

//	h = "utwalk_pid_com_x\t";
//	h += toString(utwalk_pid_com_x);
//	ret.pb(h);
//	h = "utwalk_pid_com_y\t";
//	h += toString(utwalk_pid_com_y);
//	ret.pb(h);
//	h = "utwalk_pid_com_z\t";
//	h += toString(utwalk_pid_com_z);
//	ret.pb(h);
	ifstream f;
	f.open("allo4.txt");
	string s;
	while (getline(f, s)) {
		ret.pb(s);
	}
	f.close();
	ofstream ff;
	string path = "inputrc" + toString(j) + "_" + toString(i) + ".txt";
	ff.open(path.c_str());
	for (string ss : ret) {
		ff << ss << nl;
	}
	ff.close();
	return;
}

FitFunc getFitness = [](const double *x, const int N)
{
	for(int i =0;i < 1; i++)
	GenerateParamFile(x, i, 0);
	system(BASH);
	double anss = 0;
	ifstream ifs;
	for(int i = 0; i < 1; i++)
	{
		double x;
		string path = "rc0_"+toString(i)+".txt";

		ifs.open(path.c_str());
		ifs>>x;
		anss += x;
	}
	return -anss / 1.0;

};

class parallelCMAStrategy: public CMAStrategy<CovarianceUpdate,
		GenoPheno<pwqBoundStrategy> > {
public:
	parallelCMAStrategy(FitFunc &func,
			CMAParameters<GenoPheno<pwqBoundStrategy> > &parameters) :
			CMAStrategy<CovarianceUpdate, GenoPheno<pwqBoundStrategy> >(func,
					parameters) {
	}

	~parallelCMAStrategy() {
	}

	dMat ask() {
		return CMAStrategy<CovarianceUpdate, GenoPheno<pwqBoundStrategy> >::ask();
	}

	void eval(const dMat &candidates,
			const dMat &phenocandidates = dMat(0, 0)) {
		// custom eval5.
		int pc = -1;
		for (int r = 0; r < candidates.cols(); r++) {
			if (r % 20 == 0)
				pc++;
//			_solutions.get_candidate(r).set_x(candidates.col(r));

			if (phenocandidates.size()) // if candidates in phenotype space are given
				GenerateParamFile(phenocandidates.col(r).data(), r % 20, pc);
			else
				GenerateParamFile(candidates.col(r).data(), r % 20, pc);

			//std::cerr << "candidate x: " << _solutions.get_candidate(r).get_x_dvec().transpose() << std::endl;
		}
		string bashWithPar = BASH;
		bashWithPar += " ";
		bashWithPar += toString(_niter);
		bashWithPar += " ";
		bashWithPar += "walk";
		system(bashWithPar.c_str());

		vector<double> anss;
		ifstream ifs;
		for (int j = 0; j < 8; j++)
			for (int i = 0; i < 20; i++) {
				double x;
				string path = "outputrc" + toString(j) + "_" + toString(i)
						+ ".txt";

				ifs.open(path.c_str());
				ifs >> x;
				//cout << "fitness: " << i << " " << x << endl;
				ifs.close();
				anss.pb(-1 * x);
			}

		for (int r = 0; r < candidates.cols(); r++) {
			_solutions.get_candidate(r).set_x(candidates.col(r));
			if (phenocandidates.size()) // if candidates in phenotype space are given
				_solutions.get_candidate(r).set_fvalue(anss[r]);
			else
				_solutions.get_candidate(r).set_fvalue(anss[r]);

			//std::cerr << "candidate x: " << _solutions.get_candidate(r).get_x_dvec().transpose() << std::endl;
		}
		update_fevals(candidates.cols());
	}

	void tell() {
		CMAStrategy<CovarianceUpdate, GenoPheno<pwqBoundStrategy>>::tell();
	}

	bool stop() {
		return CMAStrategy<CovarianceUpdate, GenoPheno<pwqBoundStrategy>>::stop();
	}
//	int opt() {
//		if (_initial_elitist || _parameters._initial_elitist
//				|| _parameters._elitist || _parameters._initial_fvalue) {
//			_solutions._initial_candidate = Candidate(
//					_func(_parameters._gp.pheno(_solutions._xmean).data(),
//							_parameters._dim), _solutions._xmean);
//			_solutions._best_seen_candidate = _solutions._initial_candidate;
//			this->update_fevals(1);
//		}
//		while (!stop()) {
//			dMat candidates = ask();
//			eval(candidates, _parameters._gp.pheno(candidates));
//			tell();
//			inc_iter();
//		}
//		if (_parameters._initial_elitist_on_restart) {
//			if (_parameters._initial_elitist_on_restart
//					&& _solutions._best_seen_candidate.get_fvalue()
//							< _solutions.best_candidate().get_fvalue()
//					&& _niter - _solutions._best_seen_iter >= 3) // elitist
//							{
//
//				this->set_initial_elitist(true);
//
//				// reinit solution and re-optimize.
//				_parameters.set_x0(
//						_solutions._best_seen_candidate.get_x_dvec_ref());
//				_solutions = CMASolutions(_parameters);
//				_solutions._nevals = _nevals;
//				_niter = 0;
//				optimize();
//			}
//		}
//		if(_solutions._run_status >= 0)
//			return 0;
//		return _solutions._run_status;
//	}
};
int main(int argc, char *argv[]) {
	std::vector<double> x0(dim);

	if (argc < 3)
		return -1;
	sigma = atof(argv[2]);
	MAITER = atoi(argv[1]);

//	vector<double> sigmas(dim, 0.3);
	/* DRIBBLING PARAMTERS*/
//	x0[0] = 0.8545494920155963;
//	x0[1] = 97.3370999925245 * PI / 180.0;
//	x0[2] = 86.05383913184158 * PI / 180.0;
//	x0[3] = -6.28025527312195 * PI / 180.0;
//	x0[4] = 99.78154903105181 * PI / 180.0;
//	x0[5] = 0.3335746450599291;
//	x0[6] = -0.02207231908990039;
//	x0[7] = 164.73062881817913 * PI / 180.0;
//	x0[8] = 0.06252433781071613;
//	x0[9] = 0.015223307475066804;
//	x0[10] = 0.049536000043868426;
//	x0[11] = 0.10935016019620925;
//	x0[12] = 29.541917028506017 * PI / 180.0;
//	x0[13] = 168.19797533625513 * PI / 180.0;
//	x0[14] = 5.383050376364819 * PI / 180.0;
//	x0[15] = 1.186447305495243;
//	x0[16] = -0.12017916982142124;
//	x0[17] = 0.19220698479426948;
//	x0[18] = 0.0683336940436052;
//	x0[19] = 1.3388420239842675;
//	x0[20] = 1.0002121242348725;
//	x0[21] = 0.04401545283135938;
	// INITIAL PARAMATERS
	x0[0] = 1.22;
	x0[1] = 50 * PI / 180.0;
	x0[2] = 40 * PI / 180.0;
	x0[3] = 20 * PI / 180.0;
	x0[4] = 20 * PI / 180.0;
	x0[5] = 0.2;
	x0[6] = 0.2;
	x0[7] = 175 * PI / 180.0;
	x0[8] = 0.38;
	x0[9] = 0.03;
	x0[10] = 0.03;
	x0[11] = 0.03;
	x0[12] = 7.5 * PI / 180.0;
	x0[13] = 12.5 * PI / 180.0;
	x0[14] = 2.5 * PI / 180.0;
	x0[15] = 0.5;
	x0[16] = -0.087266463;
	//type4
	x0[17] = 0.018836427341286186;
	x0[18] = -0.3033274567908883;
	x0[19] = -0.028719513936251818;
	x0[20] = -0.023659361728239663;
	x0[21] = 0.08590805524680685;
	x0[22] = -0.11237480724789313;
//	x0[17] = 0.15;
//	x0[18] = 0.2;
//	x0[19] = 1.0;
//	x0[20] = 1.0;
//	x0[21] = 0.0;
//	double sigma = 0.15;
//	sigmas[3] = 3.0;
//	sigmas[14] = 3.0;
//	sigmas[1] = 1.0;
//
//	sigmas[2] = 1.0;
//	sigmas[4] = 1.0;
//	sigmas[7] = 1.0;
//	sigmas[12] = 1.0;
//	sigmas[13] = 1.0;

	double lbounds[dim], ubounds[dim]; // arrays for lower and upper parameter bounds, respectively
	for (int i = 0; i < dim; i++) {
		lbounds[i] = -2.0 * PI;
		ubounds[i] = 2.0 * PI;
	}
//	lbounds[3] = -3.0 * PI;
//	ubounds[3] = 3.0 * PI;
//	lbounds[14] = -3.0 * PI;
//	ubounds[14] = 3.0 * PI; //      ofs<<optim.get_solutions()<<endl;

	lbounds[1] = 0;
	ubounds[1] = 4.0 * PI;

//	lbounds[17] = -1;
//	ubounds[17] = 1;
//	lbounds[18] = -1;
//	ubounds[18] = 1;
//	lbounds[19] = -2;
//	ubounds[19] = 2;
//	lbounds[20] = -2;
//	ubounds[20] = 2;
//	lbounds[21] = -2;
//	ubounds[21] = 2;

	lbounds[6] = -2;
	ubounds[6] = 2;
	lbounds[8] = 0;
	ubounds[8] = 2 * PI;
	lbounds[9] = 0;

	ubounds[9] = 2.0 * PI;
	lbounds[10] = 0;
	ubounds[10] = 2.0 * PI;
	lbounds[11] = 0;
	ubounds[11] = 2.0 * PI;
	lbounds[2] = 0;
	ubounds[2] = 4.0 * PI;
	lbounds[4] = 0;
	ubounds[4] = 4.0 * PI;
	lbounds[7] = 0;
	ubounds[7] = 4.0 * PI;
	lbounds[12] = 0;
	ubounds[12] = 4.0 * PI;
	lbounds[13] = 0;
	ubounds[13] = 4.0 * PI;
	lbounds[15] = 0;
	ubounds[15] = 2;
	lbounds[16] = -1;
	ubounds[16] = 1;

	//type4
	lbounds[17] = -0.2;
	ubounds[17] = 0.2;
	lbounds[18] = -0.5;
	ubounds[18] = 0.5;

	lbounds[19] = -0.2;
	ubounds[19] = 0.2;
	lbounds[20] = -0.2;
	ubounds[20] = 0.2;
	lbounds[21] = -0.2;
	ubounds[21] = 0.2;
	lbounds[22] = -0.2;
	ubounds[22] = 0.2;

	/* SINGLE ITERATION TEST
	 * //	CMASolutions cmasols = cmaes<GenoPheno<pwqBoundStrategy>>(getFitness,cmaparams, select_time);
	 */

	time_t start = time(0);

	GenoPheno<pwqBoundStrategy> gp(lbounds, ubounds, dim);
	CMAParameters<GenoPheno<pwqBoundStrategy> > cmaparams(x0, sigma, lambda, 0,
			gp);
	cmaparams.set_algo(aCMAES);
	cmaparams.set_max_iter(MAITER);
	ESOptimizer<parallelCMAStrategy, CMAParameters<GenoPheno<pwqBoundStrategy> > > optim(
			getFitness, cmaparams);
	ofs.open("outputs.txt");

	while (!optim.stop()) {
		dMat candidates = optim.ask();
		optim.eval(candidates, gp.pheno(candidates));
		optim.tell();
		stringstream test;
		optim.get_solutions().print(test, 0, gp);
		ofs << test.str() << endl;
		optim.inc_iter(); // important step: signals next iteration.
	}
	ofs.close();

	CMASolutions cmasols = optim.get_solutions();
	double* lastxx = cmasols.get_best_seen_candidate().get_x_pheno_dvec(
			cmaparams).data();

	GenerateParamFile(lastxx, 1337, 1337);

	std::cout << "best solution: ";
	cmasols.print(cout, 0, gp);
	cout << std::endl;
	std::cout << "worst solution: ";
	double *worst = cmasols.get_worst_seen_candidate().get_x_pheno_dvec(
			cmaparams).data();
	for (int i = 0; i < dim; i++)
		cout << worst[i] << " ";
	cout << endl;
	time_t end = time(0);
	std::cout << "optimization took " << (double) (end - start) / 1000
			<< " seconds\n";

	cout << cmasols.run_status() << endl;
	return 0;

}
