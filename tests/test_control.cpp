/* Author: Matthias Behnisch, mbehnisc@cor-lab.uni-bielefeld.de */

#define BOOST_TEST_MODULE "task_level_planning_control"
#include <boost/test/unit_test.hpp>

#include "../../../base/spaces/SE2StateSpace.h"
#include "../../../base/spaces/RealVectorStateSpace.h"
#include "../../../control/spaces/RealVectorControlSpace.h"
#include "../../../geometric/SimpleSetup.h"
#include "../../../control/SimpleSetup.h"
#include "../../../base/StateValidityChecker.h"
#include "../../../geometric/planners/rrt/RRT.h"

#include "../TaskStateSpace.h"
#include "../TaskControlStatePropagator.h"

using namespace ompl;
using namespace ompl::base;



OMPL_CLASS_FORWARD(SimpleTaskStateSpace);
class SimpleTaskStateSpace : public TaskStateSpace
{
	// a simple and pointless task state space
	// configuration:   SE2 (x,y,theta) bounds [-1..1, -1..1, -pi..pi]
	// task:            2D vector phase space (x,y,x_vel,y_vel) bounds [-1..1, -1..1]
	// forward mapping: just take the (x,y) component

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
		ensureTaskValidity(state->as<StateType>());
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

	double getMeasure() const {
		return 0;
	}

protected:

	virtual StateSpace* createTaskSpace()
	{
		RealVectorStateSpace *space = new RealVectorStateSpace(4);
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


OMPL_CLASS_FORWARD(SimpleTaskControlStatePropagator);
class SimpleTaskControlStatePropagator : public control::TaskControlStatePropagator
{
public:

	// state transition, operating in SimpleTaskStateSpace states

	SimpleTaskControlStatePropagator(const control::SpaceInformationPtr &si)
		: control::TaskControlStatePropagator(si)
	{
	}

protected:

	void taskControlTransition(const State *taskState, const control::Control* control,
	                           const double duration, State *resultTaskState) const
	{
		const RealVectorStateSpace::StateType *taskCast = taskState->as<RealVectorStateSpace::StateType>();
		const control::RealVectorControlSpace::ControlType *controlCast = control->as<control::RealVectorControlSpace::ControlType>();
		RealVectorStateSpace::StateType *resultTaskCast = resultTaskState->as<RealVectorStateSpace::StateType>();

		double dt = si_->getPropagationStepSize() * duration;

		// set task velocities
		resultTaskCast->values[2] = (controlCast->values[0] - taskCast->values[0]) * dt;
		resultTaskCast->values[3] = (controlCast->values[1] - taskCast->values[1]) * dt;

		// set task position, after transition
		resultTaskCast->values[0] = taskCast->values[0] + resultTaskCast->values[2];
		resultTaskCast->values[1] = taskCast->values[1] + resultTaskCast->values[3];
	}

	void confTowardsTaskTransition(const State *confState, const State *taskState,
	                               const double duration, State *resultConfState) const
	{
		const SE2StateSpace::StateType *confCast = confState->as<SE2StateSpace::StateType>();
		const RealVectorStateSpace::StateType *taskCast = taskState->as<RealVectorStateSpace::StateType>();
		SE2StateSpace::StateType *resultConfCast = resultConfState->as<SE2StateSpace::StateType>();

		// set configuration equal to task
		resultConfCast->setX(taskCast->values[0]);
		resultConfCast->setY(taskCast->values[1]);
		resultConfCast->setYaw(taskCast->values[0] * taskCast->values[1]);
	}
};




BOOST_AUTO_TEST_CASE(ControlStateSpace)
{
	// state space
	SimpleTaskStateSpacePtr stateSpace(new SimpleTaskStateSpace());
	stateSpace->setup();
	stateSpace->printSettings(std::cout);
	stateSpace->sanityChecks();

	// control space
	boost::shared_ptr<control::RealVectorControlSpace> controlSpace(new control::RealVectorControlSpace(stateSpace, 2));
	RealVectorBounds bounds(2);
	bounds.setLow(-1);
	bounds.setHigh(1);
	controlSpace->setBounds(bounds);
	controlSpace->setup();
	controlSpace->printSettings(std::cout);

	// test directed control sampling
	control::SpaceInformationPtr spaceInfo(new control::SpaceInformation(stateSpace, controlSpace));
	boost::shared_ptr<SimpleTaskControlStatePropagator> propagator(new SimpleTaskControlStatePropagator(spaceInfo));
	spaceInfo->setStatePropagator((control::StatePropagatorPtr)propagator);
	control::DirectedControlSamplerPtr controlSampler = spaceInfo->allocDirectedControlSampler();
	StateSamplerPtr stateSampler = spaceInfo->allocStateSampler();

	control::Control *cntrl = controlSpace->allocControl();
	State *startState = stateSpace->allocState();
	State *toState = stateSpace->allocState();

	stateSampler->sampleUniform(startState);
	stateSampler->sampleUniform(toState);
	controlSampler->sampleTo(cntrl, startState, toState);

	stateSpace->printState(startState, std::cout);
	stateSpace->printState(toState, std::cout);
	controlSpace->printControl(cntrl, std::cout);

	controlSpace->freeControl(cntrl);
	stateSpace->freeState(startState);
	stateSpace->freeState(toState);
}



BOOST_AUTO_TEST_CASE(ControlPropagation)
{
	// state and control space
	SimpleTaskStateSpacePtr stateSpace(new SimpleTaskStateSpace());
	boost::shared_ptr<control::RealVectorControlSpace> controlSpace(new control::RealVectorControlSpace(stateSpace, 2));
	RealVectorBounds bounds(2);
	bounds.setLow(-1);
	bounds.setHigh(1);
	controlSpace->setBounds(bounds);

	stateSpace->setup();
	controlSpace->setup();

	// create control::SpaceInformation
	control::SpaceInformationPtr spaceInfo(new control::SpaceInformation(stateSpace, controlSpace));
	boost::shared_ptr<SimpleTaskControlStatePropagator> propagator(new SimpleTaskControlStatePropagator(spaceInfo));
	spaceInfo->setStatePropagator((control::StatePropagatorPtr)propagator);
	StateValidityCheckerPtr stateChecker(new DummyStateValidityChecker(spaceInfo));
	spaceInfo->setStateValidityChecker(stateChecker);

	// set control duration and stepsize
	spaceInfo->setMinMaxControlDuration(1,100);
	spaceInfo->setPropagationStepSize(0.1);

	spaceInfo->setup();

	// test propagation:
	// alloc states and control
	State *startState = spaceInfo->allocState();
	State *endState1 = spaceInfo->allocState();
	State *endState2 = spaceInfo->allocState();
	State *tmpState = spaceInfo->allocState();
	control::Control *controlInput = spaceInfo->allocControl();

	// initial state needed
	// [task_x, task_y, task_x_vel, task_y_vel, conf_x, conf_y, conf_theta]
	double stateData[] = {-0.4, 0, 0, 0, -0.4, 0, 0};
	std::vector<double> stateDataVec(stateData, stateData + 7);
	stateSpace->copyFromReals(startState, stateDataVec);
	stateSpace->setStateTaskValid(startState);
	stateSpace->setStateConfValid(startState);

	// set control
	// [control_x, control_y]
	control::RealVectorControlSpace::ControlType *controlCast = controlInput->as<control::RealVectorControlSpace::ControlType>();
	controlCast->values[0] = 0.8;
	controlCast->values[1] = -0.9;

	// propagation check:
	// propagte n steps == propagate n times 1 step
	int steps = 500;
	spaceInfo->propagate(startState, controlInput, steps, endState1);

	spaceInfo->copyState(tmpState, startState);
	for(int i=0; i < steps; ++i) {

		spaceInfo->propagate(tmpState, controlInput, 1, endState2);
		spaceInfo->copyState(tmpState, endState2);
	}

	BOOST_CHECK_SMALL(spaceInfo->distance(endState1, endState2), 0.0001);

	spaceInfo->printState(endState1);
	spaceInfo->printState(endState2);
}



BOOST_AUTO_TEST_CASE(ControlPlanning)
{
	// planning setup using SimpleSetup
	SimpleTaskStateSpacePtr stateSpace(new SimpleTaskStateSpace());
	boost::shared_ptr<control::RealVectorControlSpace> controlSpace(new control::RealVectorControlSpace(stateSpace, 2));
	RealVectorBounds bounds(2);
	bounds.setLow(-1);
	bounds.setHigh(1);
	controlSpace->setBounds(bounds);

	control::SimpleSetupPtr simpleSetup(new control::SimpleSetup(controlSpace));
	StateValidityCheckerPtr stateChecker(new DummyStateValidityChecker(simpleSetup->getSpaceInformation()));
	simpleSetup->setStateValidityChecker(stateChecker);

	SimpleTaskControlStatePropagatorPtr propagator(new SimpleTaskControlStatePropagator(simpleSetup->getSpaceInformation()));
	simpleSetup->setStatePropagator((control::StatePropagatorPtr)propagator);

	// set control duration and stepsize
	simpleSetup->getSpaceInformation()->setMinMaxControlDuration(1,1000);
	simpleSetup->getSpaceInformation()->setPropagationStepSize(0.1);

	// call StateSpace setup() before ScopedState
	stateSpace->setup();
	ScopedState<> startState(stateSpace);
	ScopedState<> goalState(stateSpace);
	// accepting a large goal distance right now because
	// task space distance includes velocities
	// TODO compute task space goal distance without velocities
	double goalThreshold = 2;

	for(int i=0; i < 10; ++i) {
		stateSpace->setRandomStart(startState.get());
		stateSpace->setRandomGoal(goalState.get());

		simpleSetup->setStartState(startState);
		simpleSetup->setGoalState(goalState, goalThreshold);

		startState.print();
		goalState.print();

		PlannerStatus solved = simpleSetup->solve(5);
		BOOST_CHECK(solved);

		control::PathControl path = simpleSetup->getSolutionPath();
		path.print(std::cout);

		std::vector<State*> stateVec = path.getStates();
		State *lastState = stateVec.back();
		stateSpace->printState(startState.get(), std::cout);
		stateSpace->printState(lastState, std::cout);
		BOOST_CHECK_SMALL(stateSpace->distance(lastState, goalState.get()), goalThreshold);
	}
}
