#ifndef _OPTIMIZATION_BEHAVIORS_H
#define _OPTIMIZATION_BEHAVIORS_H

#include "../behaviors/naobehavior.h"

bool isBallMoving(const WorldModel *worldModel);

class OptimizationBehaviorFixedKick: public NaoBehavior {
	const string outputFile;

	double timeStart;
	bool hasKicked;
	bool beamChecked;
	bool backwards;
	bool ranIntoBall;
	bool fallen;

	int kick;

	double INIT_WAIT_TIME;

	VecPosition ballInitPos;
	void initKick();
	void writeFitnessToOutputFile(double fitness);

public:

	OptimizationBehaviorFixedKick(const std::string teamName, int uNum,
			const map<string, string>& namedParams_, const string& rsg_,
			const string& outputFile_);

	virtual void beam(double& beamX, double& beamY, double& beamAngle);
	virtual SkillType selectSkill();
	virtual void updateFitness();

};
struct args{
	VecPosition x;
	double angle;
	bool stop;
	args(VecPosition y, double f){
		x = y;
		angle = f;
		stop = false;
	}
	args(){
		x = VecPosition(0,0,0);
			angle = 0;
			stop = false;
	}
	args(bool sp){
		x = VecPosition(0,0,0);
		angle = 0;
		stop = sp;
	}
};
class OptimizationBehaviorWalkForward: public NaoBehavior {
	const string outputFile;
	int run;
	double startTime;
	bool beamChecked;
	bool hasFallen;
	int falls;
	double INIT_WAIT;
	double DIST_TO_WALK;
	double COST_OF_FALL;
	double orientation[30];
	double START_POSITION_X[5];
	double START_POSITION_Y[5];

	double DIS[30];
	double spiralAngle;
	bool startSet[30];
	double times[30];
	double moveTypes[30];
	VecPosition targets[30];
	VecPosition startt[30];
	bool costSet[4];
	bool x;
	VecPosition ends[30];
	bool turned;
	double angle;
	double endsAngles[30];
	bool targetSet[30];
	bool standing;
	bool startCalc;
	double expectedPerTarget;
	vector< pair<args,double> > movements[12];
	vector<int> movementsTypes[12];
	double sumOfWeights;
	double sumOfWeightsOverDistance;
	double totalWalkDist;
	double targetStartTime;
	double randomIsSetTime;
	VecPosition randomTarget;
	bool flip;
	double flipTime;
	VecPosition maxP;
	double weights[8];
	args spiralDirection;
	int currentTarget;


	int oldTarget;
	void init();
	void populateMovements();
	double fitnessSpiral(VecPosition start);
	double fitnessCircle(VecPosition start);
	SkillType performMoves();
	SkillType weave(double moveStartTime, double timePerWeave);
	SkillType randomMove(double moveStartTime, double timePerMove);
	double fitnessDiagonal(VecPosition start);
	double fitnessTurn(double startAngle, VecPosition start, int dir);
	double fitnessDirect(VecPosition start);
	double fitnessLat(VecPosition start);
	double fitnessStop(VecPosition start);
	VecPosition getTarget();
	bool checkBeam();

public:

	OptimizationBehaviorWalkForward(const std::string teamName, int uNum,
			const map<string, string>& namedParams_, const string& rsg_,
			const string& outputFile_);

	virtual void beam(double& beamX, double& beamY, double& beamAngle);
	virtual SkillType selectSkill();
	virtual void updateFitness();

};

class OptimizationBehaviorStand: public NaoBehavior
{
	 const string outputFile;

	    double timeStart;
	    bool hasKicked;
	    bool beamChecked;
	    bool backwards;
	    bool ranIntoBall;
	    bool fallen;

	    int stand;

	    double INIT_WAIT_TIME;
	    double stand_up_time;
	    double totalFitness;
	    VecPosition ballInitPos;
	    void initStand();
	    void writeFitnessToOutputFile(double fitness);

	public:

	    OptimizationBehaviorStand(const std::string teamName, int uNum, const map<
	                                  string, string>& namedParams_, const string& rsg_, const string& outputFile_);

	    virtual void beam(double& beamX, double& beamY, double& beamAngle);
	    virtual SkillType selectSkill();
	    virtual void updateFitness();
	    bool checkBeam();
};

#endif
