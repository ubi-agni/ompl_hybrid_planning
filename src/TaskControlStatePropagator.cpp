/* Author: Matthias Behnisch, mbehnisc@cor-lab.uni-bielefeld.de */

#include "ompl/contrib/task_level_planning/TaskControlStatePropagator.h"

using namespace ompl;
using namespace ompl::control;

TaskControlStatePropagator::TaskControlStatePropagator(SpaceInformation* si)
: StatePropagator(si)
{
	//controlSpace_ = dynamic_cast<TaskControlSpace*>(si->getStateSpace().get());
	//if(!controlSpace_) throw Exception("TaskControlSpace needed for TaskControlStatePropagator");
}

TaskControlStatePropagator::TaskControlStatePropagator(const SpaceInformationPtr &si)
: StatePropagator(si)
{
}

TaskControlStatePropagator::~TaskControlStatePropagator()
{
}

bool TaskControlStatePropagator::canPropagateBackward(void) const
{
	// override default to false
	return false;
}

void TaskControlStatePropagator::propagate(const base::State *state, const Control* control,
                                           const double duration, base::State *result) const
{
	// assumption: there is a TaskControlSpace and an associated TaskStateSpace
	// separate task and configuration state
	base::State *taskState = state->as<base::TaskStateSpace::StateType>()->taskState_;
	base::State *confState = state->as<base::TaskStateSpace::StateType>()->confState_;

	base::State *resultTaskState = result->as<base::TaskStateSpace::StateType>()->taskState_;
	base::State *resultConfState = result->as<base::TaskStateSpace::StateType>()->confState_;
	
	// two step propagation:
	// 1. task level propagation
	taskControlTransition(taskState, control, duration, resultTaskState);

	// 2. configuration level propagation
	confTowardsTaskTransition(confState, resultTaskState, duration, resultConfState);

	// TODO
	
	// result should have valid task and conf component now
	//result->as<base::TaskStateSpace::StateType>()->validity_ = base::TaskStateSpace::StateType::ALL_VALID;

	// result has valid conf but desired task might not be reached, task invalid
	result->as<base::TaskStateSpace::StateType>()->validity_ = base::TaskStateSpace::StateType::CONF_VALID;
}
