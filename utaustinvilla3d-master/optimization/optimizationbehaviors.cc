#include "optimizationbehaviors.h"
#include <fstream>

/*
 *
 *
 * Fixed Kick optimization agent
 *
 *
 */
OptimizationBehaviorFixedKick::OptimizationBehaviorFixedKick(
		const std::string teamName, int uNum,
		const map<string, string>& namedParams_, const string& rsg_,
		const string& outputFile_) :
		NaoBehavior(teamName, uNum, namedParams_, rsg_), outputFile(
				outputFile_), kick(0), INIT_WAIT_TIME(3.0) {
	initKick();
}

void OptimizationBehaviorFixedKick::beam(double& beamX, double& beamY,
		double& beamAngle) {
	beamX = atof(namedParams.find("kick_xoffset")->second.c_str());
	beamY = atof(namedParams.find("kick_yoffset")->second.c_str());
	beamAngle = atof(namedParams.find("kick_angle")->second.c_str());
}

SkillType OptimizationBehaviorFixedKick::selectSkill() {
	double time = worldModel->getTime();
	if (timeStart < 0) {
		initKick();
		return SKILL_STAND;
	}

	// Wait a bit before attempting kick
	if (time - timeStart <= INIT_WAIT_TIME) {
		return SKILL_STAND;
	}

	if (!hasKicked) {
		hasKicked = true;
		return SKILL_KICK_LEFT_LEG; // The kick skill that we're optimizing
	}

	return SKILL_STAND;
}

void OptimizationBehaviorFixedKick::updateFitness() {
	static double totalFitness = 0.0;
	if (kick == 10) {
		writeFitnessToOutputFile(totalFitness / (double(kick)));
		return;
	}

	double time = worldModel->getTime();
	VecPosition meTruth = worldModel->getMyPositionGroundTruth();
	meTruth.setZ(0);

	if (time - timeStart <= INIT_WAIT_TIME) {
		return;
	}

	static bool failedLastBeamCheck = false;
	if (!beamChecked) {
		beamChecked = true;
		meTruth.setZ(0);
		double beamX, beamY, beamAngle;
		beam(beamX, beamY, beamAngle);
		VecPosition meDesired = VecPosition(beamX, beamY, 0);
		double distance = meTruth.getDistanceTo(meDesired);
		double angle = worldModel->getMyAngDegGroundTruth();
		VecPosition ballPos = worldModel->getBallGroundTruth();
		double ballDistance = ballPos.getMagnitude();
		failedLastBeamCheck = false;
		string msg = "(playMode KickOff_Left)";
		setMonMessage(msg);
	}

	if (!hasKicked && worldModel->getPlayMode() == PM_PLAY_ON) {
		ranIntoBall = true;
	}

	if (!hasKicked) {
		return;
	}

	VecPosition ballTruth = worldModel->getBallGroundTruth();
	if (ballTruth.getX() < -.25) {
		backwards = true;
	}

	if (worldModel->isFallen()) {
		fallen = true;
	}

	if (time - (timeStart + INIT_WAIT_TIME) > 15
			&& !isBallMoving(this->worldModel)) {
		double angleOffset = abs(
				VecPosition(0, 0, 0).getAngleBetweenPoints(
						VecPosition(20, 0, 0), ballTruth));
		double distance = ballTruth.getX();
		double fitness = distance;

		if (backwards || distance <= 0.1 || ranIntoBall) {
			fitness = -100;
		}
		cout << "kick " << kick << " = " << fitness << endl;

		totalFitness += fitness;
		kick++;
		initKick();
		return;
	}
}

void OptimizationBehaviorFixedKick::initKick() {
	hasKicked = false;
	beamChecked = false;
	backwards = false;
	ranIntoBall = false;
	timeStart = worldModel->getTime();
	initialized = false;
	initBeamed = false;
	fallen = false;
	resetSkills();

	// Beam agent and ball
	double beamX, beamY, beamAngle;
	beam(beamX, beamY, beamAngle);
	VecPosition beamPos = VecPosition(beamX, beamY, 0);
	string msg = "(playMode BeforeKickOff)";
	setMonMessage(msg);
}

void OptimizationBehaviorFixedKick::writeFitnessToOutputFile(double fitness) {
	static bool written = false;
	if (!written) {
		fstream file;
		file.open(outputFile.c_str(), ios::out);
		file << fitness << endl;
		file.close();
		written = true;
		//string msg = "(killsim)";
		//setMonMessage(msg);
	}
}

/* Checks if the ball is currently moving */
bool isBallMoving(const WorldModel *worldModel) {
	static VecPosition lastBall = worldModel->getBallGroundTruth();
	static double lastTime = worldModel->getTime();

	double thisTime = worldModel->getTime();
	VecPosition thisBall = worldModel->getBallGroundTruth();

	thisBall.setZ(0);
	lastBall.setZ(0);

	if (thisBall.getDistanceTo(lastBall) > 0.01) {
		// the ball moved!
		lastBall = thisBall;
		lastTime = thisTime;
		return true;
	}

	if (thisTime - lastTime < 0.5) {
		// not sure yet if the ball has settled
		return true;
	} else {
		return false;
	}
}

void writeToOutputFile(const string &filename, const string &output) {
//  static bool written = false;
//  assert(!written);
	//  LOG(output);
	fstream file;
	file.open(filename.c_str(), ios::out);
	file << output;
	file.close();
//  written = true;
}

/*
 *
 *
 * WALK FORWARD OPTIMIZATION AGENT
 *
 *
 *
 */
OptimizationBehaviorWalkForward::OptimizationBehaviorWalkForward(
		const std::string teamName, int uNum,
		const map<string, string>& namedParams_, const string& rsg_,
		const string& outputFile_) :
		NaoBehavior(teamName, uNum, namedParams_, rsg_), outputFile(outputFile_) {
	INIT_WAIT = 1;
	run = 0;
	for (int i = 0; i < 6; i++)
		DIS[i] = 0.0, startSet[i] = false;
	sumOfWeights = sumOfWeightsOverDistance = 0;
	expectedPerTarget = 5.0;
	DIST_TO_WALK = 2.5;
	totalWalkDist = 0;
	flip = 1;
	START_POSITION_X[0] = -HALF_FIELD_X + 7;
	START_POSITION_Y[0] = -HALF_FIELD_Y + 3;
	startCalc = false;
	for (int i = 0; i < 5; i++)
		START_POSITION_X[i] = -HALF_FIELD_X + 7, START_POSITION_Y[i] =
				-HALF_FIELD_Y + 3;
//	START_POSITION_Y[3] = 0;
	spiralDirection = args(VecPosition(0, 0, 0), 0);
//	START_POSITION_X[1] = -6;
//	START_POSITION_Y[1] = 0;
//	START_POSITION_X[2] = -1;
//	START_POSITION_Y[2] = 0;
//	START_POSITION_X[3] = -HALF_FIELD_X + 7;
//	START_POSITION_Y[3] = -HALF_FIELD_Y + 3;
//	START_POSITION_X[4] = -HALF_FIELD_X + 3;
//	START_POSITION_Y[4] = 0;
	COST_OF_FALL = 4;
	maxP = VecPosition(-50, -50, -50);
	// Use ground truth localization for behavior
	worldModel->setUseGroundTruthDataForLocalization(true);
	for (int i = 0; i < 12; i++) {
		movements[i].clear();
		movementsTypes[i].clear();
	}
	populateMovements();

	init();
}

void OptimizationBehaviorWalkForward::init() {

	startTime = worldModel->getTime();
	initialized = false;
	initBeamed = false;
	flip = 1;
	hasFallen = false;
	falls = 0;
	oldTarget = 0;
	beamChecked = false;
	for (int i = 0; i < 12; i++) {
		DIS[i] = 0.0;
		startSet[i] = false;
	}
	standing = false;
	srand(time(0));
	startCalc = false;
	randomTarget = VecPosition(HALF_FIELD_X, HALF_FIELD_Y, 0);
	targetStartTime = startTime + INIT_WAIT;
	double currentTime = worldModel->getTime();
	randomIsSetTime = currentTime - 0.5;
	double x, y, z;
	beam(x, y, z);
	startt = VecPosition(x, y, 0);
	string msg = "(playMode BeforeKickOff)";
	setMonMessage(msg);
}
void OptimizationBehaviorWalkForward::populateMovements() {
	args forward = args(VecPosition(1, 0, 0), 0);
	args backwards = args(VecPosition(-1, 0, 0), 0);
	args left_lat = args(VecPosition(0, 1, 0), 0);
	args right_lat = args(VecPosition(0, -1, 0), 0);
	args circle = args(VecPosition(1, 0, 0), 7);
	args weave45_for = args(VecPosition(1, 1, 0), 45);
	args weave45_back = args(VecPosition(-1, -1, 0), 45);
	args downRight = args(VecPosition(1, -1, 0), 0);
	spiralDirection = args(VecPosition(1, 0, 0), 200);
	args turn_counter_clockwise = args(VecPosition(0, 1, 0), 90);
	args turn_clockwise = args(VecPosition(0, -1, 0), -90);
	args stop = args(true);

	movements[0].push_back(pair<args, double>(forward, 6));
	movementsTypes[0].push_back(0);

	movements[0].push_back(pair<args, double>(backwards, 15.0));
	movementsTypes[0].push_back(0);

	weights[0] = 5;

	movements[1].push_back(pair<args, double>(right_lat, 7));
	movementsTypes[1].push_back(4);
	movements[1].push_back(pair<args, double>(left_lat, 14));
	movementsTypes[1].push_back(4);
	movements[1].push_back(pair<args, double>(stop, 15));
	movementsTypes[1].push_back(5);

	weights[1] = 4;

	movements[2].push_back(pair<args, double>(circle, 15));
	movementsTypes[2].push_back(1);

	weights[2] = 1;

	movements[3].push_back(pair<args, double>(forward, 15));
	movementsTypes[3].push_back(0);

	weights[3] = 5;

	movements[4].push_back(pair<args, double>(turn_counter_clockwise, 7.5));
	movementsTypes[4].push_back(3);
	movements[4].push_back(pair<args, double>(turn_clockwise, 15));
	movementsTypes[4].push_back(3);

	weights[4] = 2;
//	movements[3].push_back(pair<args, double>(backwards, 10));
//	movementsTypes[3].push_back(0);
//	movements[3].push_back(pair<args, double>(forward, 14));
//	movementsTypes[3].push_back(0);
//	movements[3].push_back(pair<args, double>(stop, 15));
//	movementsTypes[3].push_back(5);
//
//	movements[4].push_back(pair<args, double>(circle, 15));
//	movementsTypes[4].push_back(1);

	for (double x = 0.2; x <= 15.0 - 0.2; x += 0.4) {
		movements[5].push_back(pair<args, double>(forward, x));
		movementsTypes[5].push_back(0);
		movements[5].push_back(pair<args, double>(left_lat, x + 0.2));
		movementsTypes[5].push_back(4);
	}
	for (double x = 0; x <= 15.0 - 0.1; x += 0.1) {
		movements[6].push_back(pair<args, double>(forward, x + 0.05));
		movementsTypes[6].push_back(0);
		movements[6].push_back(
				pair<args, double>(turn_counter_clockwise, x + 0.1));
		movementsTypes[6].push_back(3);
	}
	weights[5] = 3;
	weights[6] = 2;
	weights[7] = 1;
//	circle.angle = 15;
//	movements[6].push_back(pair<args, double>(circle, 15));
//	movementsTypes[6].push_back(1);
//	movements[5].push_back(pair<args, double>(forward, 4));
//	movementsTypes[5].push_back(0);
//	movements[5].push_back(pair<args, double>(stop, 5));
//	movementsTypes[5].push_back(5);
//	movements[5].push_back(pair<args, double>(right_lat, 12));
//	movementsTypes[5].push_back(4);
//	movements[5].push_back(pair<args, double>(stop, 13));
//	movementsTypes[5].push_back(5);
//	movements[5].push_back(pair<args, double>(left_lat, 15));
//	movementsTypes[5].push_back(4);

//
//
//	movements[7].push_back(pair<args, double>(forward, 2));
//	movementsTypes[7].push_back(0);
//	movements[7].push_back(pair<args, double>(right_lat, 4));
//	movementsTypes[7].push_back(4);
//	movements[7].push_back(pair<args, double>(left_lat, 6));
//	movementsTypes[7].push_back(4);
//	movements[7].push_back(pair<args, double>(backwards, 8));
//	movementsTypes[7].push_back(0);
//	movements[7].push_back(pair<args, double>(right_lat, 10));
//	movementsTypes[7].push_back(4);
//	movements[7].push_back(pair<args, double>(left_lat, 14));
//	movementsTypes[7].push_back(4);
//	movements[7].push_back(pair<args, double>(stop, 15));
//	movementsTypes[7].push_back(5);
//
//	movements[8].push_back(pair<args, double>(right_lat, 7));
//	movementsTypes[8].push_back(4);
//
//	movements[8].push_back(pair<args, double>(left_lat, 14));
//	movementsTypes[8].push_back(4);
//
//	movements[8].push_back(pair<args, double>(stop, 15));
//	movementsTypes[8].push_back(5);

}
double OptimizationBehaviorWalkForward::fitnessCircle(VecPosition start) {
	double ret = 0;
	VecPosition me = worldModel->getMyPositionGroundTruth();
	me.setZ(0);
	if (me.getX() < maxP.getX()) {
		ret = maxP.getX() - start.getX() + maxP.getDistanceTo(me);
	} else {
		ret = start.getDistanceTo(me);
	}
	if (maxP.getX() <= me.getX())
		maxP = me;
	if (maxP.getX() - (-2.46) >= 0.1)
		ret = -2;
	return ret;
}
double OptimizationBehaviorWalkForward::fitnessDirect(VecPosition start) {
	double ret = 0;
	VecPosition me = worldModel->getMyPositionGroundTruth();
	me.setZ(0);

	ret = me.getDistanceTo(start);
	return ret;
}

double OptimizationBehaviorWalkForward::fitnessLat(VecPosition start) {
	double ret = 0;
	VecPosition me = worldModel->getMyPositionGroundTruth();
	me.setZ(0);
	ret = abs(me.getY() - start.getY());
	return ret;
}

double OptimizationBehaviorWalkForward::fitnessSpiral(VecPosition start) {
	VecPosition me = worldModel->getMyPositionGroundTruth();
	me.setZ(0);

	if (me.getY() == start.getY() && abs(me.getX() - start.getX()) >= 0.7) {
		return 1;
	} else
		return 0;
}
double OptimizationBehaviorWalkForward::fitnessStop(VecPosition start) {
	VecPosition me = worldModel->getMyPositionGroundTruth();
	if (me.getZ() <= 0.45)
		return 1;
	else
		return 0;
}
double OptimizationBehaviorWalkForward::fitnessDiagonal(VecPosition start) {
	double ret = 0;
	VecPosition me = worldModel->getMyPositionGroundTruth();
	me.setZ(0);

	ret = me.getDistanceTo(start);
	double deltay = fabs(me.getY() - start.getY());
	double deltax = fabs((start.getX() - me.getX()));
	if (deltax == 0 || deltay == 0 || fabs(deltay / deltax - 1) > 0.55)
		ret *= 0.5;
	return ret;
}
double OptimizationBehaviorWalkForward::fitnessTurn(double startAngle,
		VecPosition start) {
	double ang = worldModel->getMyAngDegGroundTruth();
	if (fabs(ang - startAngle) >= 5)
		return 1;
	else
		return 0;

}

SkillType OptimizationBehaviorWalkForward::selectSkill() {
	double currentTime = worldModel->getTime();
	if (currentTime - startTime < INIT_WAIT || startTime < 0 || standing) {
		return SKILL_STAND;
	}
	srand(time(0));
	if (run != 7 && run != 8) {
		for (int j = movements[run].size() - 1; j >= 0; j--) {
			if (currentTime - targetStartTime <= movements[run][j].second
					&& currentTime - targetStartTime
							> ((j == 0) ? -1 : movements[run][j - 1].second)) {
				return (movements[run][j].first.stop) ?
						SKILL_STAND :
						goToTargetRelative(movements[run][j].first.x,
								movements[run][j].first.angle);
			}
		}
	}
	if (run == 7) {
		if (currentTime - randomIsSetTime >= 0.9) {
			randomTarget = VecPosition(randomTarget.getX() * -1,
					randomTarget.getY() * -1, 0);
			randomIsSetTime = currentTime;
		}
		return goToTarget(randomTarget);

	} else {
		VecPosition center = VecPosition(-HALF_FIELD_X / 2.0, 0, 0);
		double circleRadius = 7.0;
		double rotateRate = 3;
		VecPosition localCenter = worldModel->g2l(center);
		SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(),
				localCenter.getX());

		// Our desired target position on the circle
		// Compute target based on uniform number, rotate rate, and time
		VecPosition target = center
				+ VecPosition(circleRadius, 0, 0).rotateAboutZ(
						360.0 / (NUM_AGENTS - 1) * (worldModel->getUNum())
								+ worldModel->getTime() * rotateRate);

		// Adjust target to not be too close to teammates or the ball
		target = collisionAvoidance(true /*teammate*/, false/*opponent*/,
				true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/,
				target, true/*keepDistance*/);

		if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
			// Close enough to desired position and orientation so just stand
			return SKILL_STAND;
		} else if (me.getDistanceTo(target) < .5) {
			// Close to desired position so start turning to face center
			return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
		} else {
			// Move toward target location
			return goToTarget(target);
		}
	}

}

void OptimizationBehaviorWalkForward::updateFitness() {
	static bool written = false;

	if (run == 9) {
		if (!written) {
			double fitness = totalWalkDist / (double) run;
			fstream file;
			file.open(outputFile.c_str(), ios::out);
			file << fitness << endl;
			file.close();
			written = true;
		}
		return;
	}

	if (startTime < 0) {
		init();
		return;
	}

	double currentTime = worldModel->getTime();
	if (currentTime - startTime < INIT_WAIT) {
		return;
	}

	if (!beamChecked) {
		static bool failedLastBeamCheck = false;
		if (!checkBeam()) {
			// Beam failed so reinitialize everything
			if (failedLastBeamCheck) {
				// Probably something bad happened
				//if we failed the beam twice in a row (perhaps the agent can't stand) so give a bad score and
				// move on
				totalWalkDist -= 100;
				run++;
			}
			failedLastBeamCheck = true;
			init();
			return;
		} else {
			failedLastBeamCheck = false;
			// Set playmode to PlayOn to start run and move ball out of the way
			string msg = "(playMode PlayOn) (ball (pos 0 -9 0) (vel 0 0 0))";
			setMonMessage(msg);
		}
	}
	double angle = 0;
	VecPosition me = worldModel->getMyPositionGroundTruth();
	me.setZ(0);
	startt.setZ(0);
	if (worldModel->isFallen() && !hasFallen) {
		hasFallen = true;
		falls++;
	}
	if (worldModel->isFallen() == false && hasFallen) {
		hasFallen = false;
	}
	int idx = run;
	if (run < 5) {
		for (int j = movements[idx].size() - 1; j >= 0; j--) {
			if (currentTime - targetStartTime <= movements[idx][j].second
					&& currentTime - targetStartTime
							> ((j == 0) ? -1 : movements[idx][j - 1].second)) {
				if (!startSet[j]) {
					startt = me;
					angle = worldModel->getMyAngDegGroundTruth();
					startSet[j] = 1;
					startCalc = false;
				}
				if (movementsTypes[idx][j] == 0) {
					DIS[j] = fitnessDirect(startt);
				} else if (movementsTypes[idx][j] == 1) {
					DIS[j] = fitnessCircle(startt);
				} else if (movementsTypes[idx][j] == 2) {
					DIS[j] = fitnessDiagonal(startt);
				} else if (movementsTypes[idx][j] == 3) {
					DIS[j] += fitnessTurn(angle, startt);
				} else if (movementsTypes[idx][j] == 4)
					DIS[j] = fitnessLat(startt);
				else if (movementsTypes[idx][j] == 5)
					DIS[j] += fitnessStop(startt);
			}
		}
	}

	if (currentTime - targetStartTime > 15.0) {
		double fit = 0;
		fit += (falls * COST_OF_FALL * -1);
		if (run < 4) {
			bool x = false;
			for (int i = movements[idx].size() - 1; i >= 0; i--) {
				if (movementsTypes[idx][i] == 5)
					fit += (DIS[i] > 0) * -3;
				else {
					if (DIS[i] <= 0.1)
						x = true;
					fit += DIS[i];
				}
			}
			fit -= x * 2;
		} else if (run == 4) {
			if(falls == 0)
				fit += 1.5*(DIS[0]>0) + 1.5*(DIS[1]>0);
			else
				fit -= 3;
		} else if (run == 5) {
			double dist = fitnessDiagonal(startt);
			if (dist <= 0.1)
				fit -= 2;
			fit += dist;
		} else {
			double dist = fitnessDirect(startt);
			if (dist <= 0.1)
				fit -= 2;
			fit += dist * 0.8;
		}
		totalWalkDist += fit * ((run == 7 || run == 6) ? 1.5 : 1);
		run++;
		cout << "Run " << run << " : distance walked " << fit << endl;
		init();
	}
}

void OptimizationBehaviorWalkForward::beam(double& beamX, double& beamY,
		double& beamAngle) {
	if (run != 8) {
		beamX = START_POSITION_X[run % 5];
		beamY = START_POSITION_Y[run % 5];
		beamAngle = 0;
	} else {
		VecPosition center = VecPosition(-HALF_FIELD_X / 2.0, 0, 0);
		double circleRadius = 7.0;
		double rotateRate = 3;
		VecPosition localCenter = worldModel->g2l(center);
		SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(),
				localCenter.getX());

		// Our desired target position on the circle
		// Compute target based on uniform number, rotate rate, and time
		VecPosition target = center
				+ VecPosition(circleRadius, 0, 0).rotateAboutZ(
						360.0 / (NUM_AGENTS - 1) * (worldModel->getUNum())
								+ worldModel->getTime() * rotateRate);

		// Adjust target to not be too close to teammates or the ball
		target = collisionAvoidance(true /*teammate*/, false/*opponent*/,
				true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/,
				target, true/*keepDistance*/);

		beamX = -2;
		beamY = 0;
		beamAngle = 0;
	}
}

bool OptimizationBehaviorWalkForward::checkBeam() {
	LOG_STR("Checking whether beam was successful");
	VecPosition meTruth = worldModel->getMyPositionGroundTruth();
	meTruth.setZ(0);
	double beamX, beamY, beamAngle;
	beam(beamX, beamY, beamAngle);
	VecPosition meDesired = VecPosition(beamX, beamY, 0);
	double distance = meTruth.getDistanceTo(meDesired);
	double angleOffset = abs(worldModel->getMyAngDegGroundTruth() - beamAngle);
	if (distance > 0.05 || angleOffset > 5) {
		LOG_STR("Problem with the beam!");
		LOG(distance);
		LOG(meTruth);
		return false;
	}
	beamChecked = true;
	return true;
}
/*
 *
 *
 * STAND OPTIMIZATION AGENT
 *
 *
 *
 */

OptimizationBehaviorStand::OptimizationBehaviorStand(const std::string teamName,
		int uNum, const map<string, string>& namedParams_, const string& rsg_,
		const string& outputFile_) :
		NaoBehavior(teamName, uNum, namedParams_, rsg_), outputFile(
				outputFile_), stand(0), INIT_WAIT_TIME(3.0) {
	totalFitness = 0.0;
	initStand();
}

void OptimizationBehaviorStand::beam(double& beamX, double& beamY,
		double& beamAngle) {
	beamX = -5.0;
	beamY = -5.0;
	beamAngle = 0.0;
}

SkillType OptimizationBehaviorStand::selectSkill() {
	double time = worldModel->getTime();
	if (timeStart < 0) {
		initStand();
		return SKILL_STAND;
	}

// Wait a bit before attempting kick
	if (time - timeStart <= INIT_WAIT_TIME) {
		return SKILL_STAND;
	}

	if (time - timeStart < 5)
		return SKILL_DIVE_LEFT;

	return SKILL_STAND;

}

void OptimizationBehaviorStand::updateFitness() {

	if (stand == 10) {
		writeFitnessToOutputFile(totalFitness / (double(stand)));
		return;
	}

	double time = worldModel->getTime();
	if (time - timeStart <= INIT_WAIT_TIME) {
		return;
	}

	if (!beamChecked) {
		beamChecked = true;

		// Set playmode to PlayOn to start run and move ball out of the way
		string msg = "(playMode PlayOn)";
		setMonMessage(msg);
	}

	if (worldModel->getMyPositionGroundTruth().getZ() < 0.47)
		stand_up_time = 0.0;

	if (worldModel->getMyPositionGroundTruth().getZ() >= 0.48
			&& (time - (timeStart + INIT_WAIT_TIME)) > 2) {
		if (stand_up_time == 0)
			stand_up_time = time;

		else if (time - stand_up_time > 2) {
			cout << "Stand " << stand << " Fitness = "
					<< (time - (timeStart + INIT_WAIT_TIME)) << endl;
			totalFitness += (time - timeStart);
			stand++;
			initStand();
			return;
		}
	}

	VecPosition meTruth = worldModel->getMyPositionGroundTruth();
	meTruth.setZ(0);

	if (time - (timeStart + INIT_WAIT_TIME) > 15) {

		totalFitness += 100;
		stand++;
		cout << "Stand " << stand << " Fitness = "
				<< (time - (timeStart + INIT_WAIT_TIME)) << endl;
		initStand();
		return;

	}
}

void OptimizationBehaviorStand::initStand() {

	beamChecked = false;
	timeStart = worldModel->getTime();
	initialized = false;
	initBeamed = false;
	fallen = false;
	stand_up_time = 0.0;
	resetSkills();

// Beam agent and ball
	double beamX, beamY, beamAngle;
	beam(beamX, beamY, beamAngle);
	VecPosition beamPos = VecPosition(beamX, beamY, 0);
	string msg = "(playMode BeforeKickOff)";
	setMonMessage(msg);
}

void OptimizationBehaviorStand::writeFitnessToOutputFile(double fitness) {
	static bool written = false;
	if (!written) {
		LOG(fitness);
		LOG(stand);
		fstream file;
		file.open(outputFile.c_str(), ios::out);
		file << fitness << endl;
		file.close();
		written = true;
		//string msg = "(killsim)";
		//setMonMessage(msg);
	}
}

bool OptimizationBehaviorStand::checkBeam() {
	LOG_STR("Checking whether beam was successful");
	VecPosition meTruth = worldModel->getMyPositionGroundTruth();
	meTruth.setZ(0);
	double beamX, beamY, beamAngle;
	beam(beamX, beamY, beamAngle);
	VecPosition meDesired = VecPosition(beamX, beamY, 0);
	double distance = meTruth.getDistanceTo(meDesired);
	double angleOffset = abs(worldModel->getMyAngDegGroundTruth() - beamAngle);
	if (distance > 0.05 || angleOffset > 5) {
		LOG_STR("Problem with the beam!");
		LOG(distance);
		LOG(meTruth);
		return false;
	}
	beamChecked = true;
	return true;
}

