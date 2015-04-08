/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Your Institution
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Your Institution nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Matthias Behnisch */

#define BOOST_TEST_MODULE "task_level_planning_geometric"
#include <boost/test/unit_test.hpp>


#include "../../../../../tests/base/StateSpaceTest.h"
#include "../../../base/spaces/SE2StateSpace.h"
#include "../../../base/spaces/RealVectorStateSpace.h"
#include "../../../geometric/SimpleSetup.h"
#include "../../../base/StateValidityChecker.h"
#include "../../../geometric/planners/rrt/RRT.h"

#include "../TaskStateSpace.h"
#include "../TaskMotionValidator.h"

using namespace ompl;
using namespace ompl::base;



OMPL_CLASS_FORWARD(SimpleGeometricTaskStateSpace);
class SimpleGeometricTaskStateSpace : public TaskGeometricStateSpace
{
	// a simple and pointless geometric task state space
	// configuration:   SE2 (x,y,theta) bounds [-1..1, -1..1, -1..1]
	// task:            2D vector space (x,y) bounds [-1..1, -1..1]
	// forward mapping: just take the (x,y) component
	// inverse mapping: deterministicly set theta in dependency of (x,y)

public:

	void setRandomStart(State *state)
	{
		// initialize configuration
		State *confState = state->as<StateType>()->confState_;

		// random sample between [-0.5..1, -1..1, -pi..pi]
		while(true) {
			confSpace_->allocDefaultStateSampler()->sampleUniform(confState);
			if(confState->as<SE2StateSpace::StateType>()->getX() > -0.5) break;
		}
		
		setStateConfValid(state);
		setStateTaskInvalid(state);
	}

	void setRandomGoal(State *state)
	{
		// initialize task
		State *taskState = state->as<StateType>()->taskState_;

		// random sample between [-0.5..1, -1..1]
		while(true) {
			taskSpace_->allocDefaultStateSampler()->sampleUniform(taskState); 
			if(taskState->as<RealVectorStateSpace::StateType>()->values[0] > -0.5) break;
		}
		
		setStateTaskValid(state);
		setStateConfInvalid(state);
	}
	
protected:

	virtual StateSpace* createTaskSpace()
	{
		RealVectorStateSpace *space = new RealVectorStateSpace(2);
		space->setBounds(-1,1);
		return space;
	}
	
	virtual StateSpace* createConfSpace()
	{
		SE2StateSpace *space = new SE2StateSpace();
		RealVectorBounds bounds(2);
		bounds.setLow(-1);
		bounds.setHigh(1);
		space->setBounds(bounds);
		return space;
	}
	
	virtual void forwardMapping(const State *conf, State *task) const
	{
		const SE2StateSpace::StateType *confCast = conf->as<SE2StateSpace::StateType>();
		RealVectorStateSpace::StateType *taskCast = task->as<RealVectorStateSpace::StateType>();

		taskCast->values[0] = confCast->getX();
		taskCast->values[1] = confCast->getY();
	}
			
	virtual void inverseMapping(const State *task, State *conf) const
	{
		const RealVectorStateSpace::StateType *taskCast = task->as<RealVectorStateSpace::StateType>();
		SE2StateSpace::StateType *confCast = conf->as<SE2StateSpace::StateType>();

		confCast->setX(taskCast->values[0]);
		confCast->setY(taskCast->values[1]);
		confCast->setYaw(taskCast->values[0] * taskCast->values[1]);
	}

};



// dummy state validity checker
// checks if state is checkable at all, i.e. has
// associated configuration
// checks if task x component > -0.5 
class DummyStateValidityChecker : public StateValidityChecker
{
public:
	DummyStateValidityChecker(const SpaceInformationPtr &si)
	: StateValidityChecker(si)
	{
	}

	virtual bool isValid(const State *state) const
	{
		const TaskStateSpace::StateType *stateCast = state->as<TaskStateSpace::StateType>();
		const RealVectorStateSpace::StateType *taskState = stateCast->taskState_->as<RealVectorStateSpace::StateType>();
		if(taskState->values[0] < -0.5) return false;
		else return si_->getStateSpace()->as<TaskStateSpace>()->isStateConfValid(state);
	}
};









BOOST_AUTO_TEST_CASE(GeometricStateSpace)
{
    SimpleGeometricTaskStateSpacePtr space(new SimpleGeometricTaskStateSpace());

	// run provided state space tests
    space->setup();
    space->sanityChecks();

    boost::shared_ptr<ompl::StateSpaceTest> spaceTest(new StateSpaceTest(space));
    spaceTest->test();

	space->printSettings(std::cout);

    // check double representation
    State *state = space->allocState();

	double d[] = {1,2,3,4,5};
	std::vector<double> data(d, d + 5);
	
	space->copyFromReals(state, data);
    //space->printState(state,std::cout);

    space->copyToReals(data, state); 
    for(int i = 0; i < 5; ++i) {
		//std::cout << data[i] << "\n";
		BOOST_CHECK(data[i] == d[i]);
	}

	space->freeState(state);
}




BOOST_AUTO_TEST_CASE(GeometricPlanning)
{
	// planning setup using SimpleSetup
	SimpleGeometricTaskStateSpacePtr space(new SimpleGeometricTaskStateSpace());
	geometric::SimpleSetupPtr simpleSetup(new geometric::SimpleSetup(space));
	StateValidityCheckerPtr stateChecker(new DummyStateValidityChecker(simpleSetup->getSpaceInformation()));
	simpleSetup->setStateValidityChecker(stateChecker);
	TaskMotionValidatorPtr motionValidator(new TaskMotionValidator(simpleSetup->getSpaceInformation()));
	simpleSetup->getSpaceInformation()->setMotionValidator(motionValidator);

	// set uni-directional RRT planner
	PlannerPtr planner(new geometric::RRT(simpleSetup->getSpaceInformation()));
	simpleSetup->setPlanner(planner);

	// call StateSpace setup() before ScopedState
	space->setup();

	ScopedState<> startState(space);
	ScopedState<> goalState(space);
	double goalThreshold = 0.01;

	for(int i=0; i < 100; ++i) {
		space->setRandomStart(startState.get());
		space->setRandomGoal(goalState.get());

		simpleSetup->setStartState(startState);
		simpleSetup->setGoalState(goalState, goalThreshold);
		
		PlannerStatus solved = simpleSetup->solve(5);	
		BOOST_CHECK(solved);

		geometric::PathGeometric path = simpleSetup->getSolutionPath();
		std::vector<State*> stateVec = path.getStates();
		State *lastState = stateVec.back();
		BOOST_CHECK_SMALL(space->distance(lastState, goalState.get()), goalThreshold);

		simpleSetup->clear();
	}
	
	MotionValidatorPtr validator = simpleSetup->getSpaceInformation()->getMotionValidator();
	std::cout << "motion validator: " << validator->getValidMotionCount() << " valid, " 
	                                  << validator->getInvalidMotionCount() << " invalid\n";
}



