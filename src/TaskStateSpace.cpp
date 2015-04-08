/* Author: Matthias Behnisch, mbehnisc@cor-lab.uni-bielefeld.de */

#include "ompl/contrib/task_level_planning/TaskStateSpace.h"

using namespace ompl::base;


void TaskStateSampler::sampleUniform(State *state)
{
	taskSampler_->sampleUniform(state->as<TaskStateSpace::StateType>()->taskState_);

	// only task is now valid
	state->as<TaskStateSpace::StateType>()->validity_ = TaskStateSpace::StateType::TASK_VALID; 
}

void TaskStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
	// check task validity of near state
	space_->as<TaskStateSpace>()->ensureTaskValidity(near->as<TaskStateSpace::StateType>());
	
	taskSampler_->sampleUniformNear(state->as<TaskStateSpace::StateType>()->taskState_,
	                                near->as<TaskStateSpace::StateType>()->taskState_, distance);

	state->as<TaskStateSpace::StateType>()->validity_ = TaskStateSpace::StateType::TASK_VALID;
}

void TaskStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
	// check task validity of mean state
	space_->as<TaskStateSpace>()->ensureTaskValidity(mean->as<TaskStateSpace::StateType>());
	
	taskSampler_->sampleGaussian(state->as<TaskStateSpace::StateType>()->taskState_,
	                             mean->as<TaskStateSpace::StateType>()->taskState_, stdDev);

	state->as<TaskStateSpace::StateType>()->validity_ = TaskStateSpace::StateType::TASK_VALID;
}



TaskStateSpace::TaskStateSpace()
{
	setName("Task" + getName()); 
}

TaskStateSpace::~TaskStateSpace()
{
}

void TaskStateSpace::setup(void)
{
	// create and initialize contained statespaces
	taskSpace_ = boost::shared_ptr<StateSpace>(createTaskSpace());
	confSpace_ = boost::shared_ptr<StateSpace>(createConfSpace());

	taskSpace_->setup();
	confSpace_->setup();

	StateSpace::setup();
}

unsigned int TaskStateSpace::getDimension(void) const
{
	return taskSpace_->getDimension();
}

double TaskStateSpace::getMaximumExtent(void) const
{
	return taskSpace_->getMaximumExtent();
}

void TaskStateSpace::enforceBounds(State *state) const
{
	if(!isStateTaskValid(state)) taskSpace_->enforceBounds(state->as<StateType>()->taskState_);
}

bool TaskStateSpace::satisfiesBounds(const State *state) const 
{
	if(!isStateTaskValid(state)) return true;
	else return taskSpace_->satisfiesBounds(state->as<StateType>()->taskState_);
}

void TaskStateSpace::copyState(State *destination, const State *source) const
{
	StateType *destinationCast = destination->as<StateType>();
	const StateType *sourceCast = source->as<StateType>();
	
	taskSpace_->copyState(destinationCast->taskState_, sourceCast->taskState_);
	confSpace_->copyState(destinationCast->confState_, sourceCast->confState_);

	destinationCast->validity_ = sourceCast->validity_;
}
            
double TaskStateSpace::distance(const State *state1, const State *state2) const
{
	const StateType *state1Cast = state1->as<StateType>();
	const StateType *state2Cast = state2->as<StateType>();
	
	ensureTaskValidity(state1Cast);
	ensureTaskValidity(state2Cast);
	
	return taskSpace_->distance(state1Cast->taskState_, state2Cast->taskState_);
}

bool TaskStateSpace::equalStates(const State *state1, const State *state2) const
{
	const StateType *state1Cast = state1->as<StateType>();
	const StateType *state2Cast = state2->as<StateType>();
	
	ensureTaskValidity(state1Cast);
	ensureTaskValidity(state2Cast);
		
	return taskSpace_->equalStates(state1Cast->taskState_, state2Cast->taskState_);
}

unsigned int TaskStateSpace::getSerializationLength(void) const
{
	return taskSpace_->getSerializationLength() + confSpace_->getSerializationLength();
}

void TaskStateSpace::serialize(void *serialization, const State *state) const
{
	taskSpace_->serialize(serialization, state->as<StateType>()->taskState_);
	unsigned int offset = taskSpace_->getSerializationLength();
	confSpace_->serialize(static_cast<char*>(serialization) + offset, state->as<StateType>()->confState_); 
}

void TaskStateSpace::deserialize(State *state, const void *serialization) const
{
	taskSpace_->deserialize(state->as<StateType>()->taskState_, serialization);
	unsigned int offset = taskSpace_->getSerializationLength();
	confSpace_->deserialize(state->as<StateType>()->confState_, static_cast<const char*>(serialization) + offset);

	// TODO HANDLE VALIDITY FLAG
}

StateSamplerPtr TaskStateSpace::allocDefaultStateSampler(void) const
{
	return StateSamplerPtr(new TaskStateSampler(taskSpace_.get()));
}

/*void TaskStateSpace::registerProjections(void)
{

}*/

State* TaskStateSpace::allocState(void) const
{
    StateType *state = new StateType();
    state->validity_ = StateType::NONE_VALID;
    state->taskState_ = taskSpace_->allocState();
    state->confState_ = confSpace_->allocState();
    return static_cast<State*>(state);
}

void TaskStateSpace::freeState(State *state) const
{
	confSpace_->freeState(state->as<StateType>()->confState_);
	taskSpace_->freeState(state->as<StateType>()->taskState_);
	delete static_cast<StateType*>(state);
}

double* TaskStateSpace::getValueAddressAtIndex(State *state, const unsigned int index) const
{
	// assume: indices enumerate dimensions
	if(index < taskSpace_->getDimension()) {
		unsigned int taskIndex = index;
		return taskSpace_->getValueAddressAtIndex(state->as<StateType>()->taskState_, taskIndex);
	} else {
		unsigned int confIndex = index - taskSpace_->getDimension();
		return confSpace_->getValueAddressAtIndex(state->as<StateType>()->confState_, confIndex);
	}
}

void TaskStateSpace::printState(const State *state, std::ostream &out) const
{
	out << "Flag: " << state->as<StateType>()->validity_;
	out << "\nTask: ";
	if(state->as<StateType>()->validity_ & StateType::TASK_VALID) out << " valid\n";
	else out << " not valid\n";
	taskSpace_->printState(state->as<StateType>()->taskState_, out);
	
	out << "Conf: ";
	if(state->as<StateType>()->validity_ & StateType::CONF_VALID) out << " valid\n";
	else out << " not valid\n";
	confSpace_->printState(state->as<StateType>()->confState_, out);
}

void TaskStateSpace::printSettings(std::ostream &out) const
{
	StateSpace::printSettings(out);
	out << "Task:\n";
	taskSpace_->printSettings(out);
	out << "Conf:\n";
	confSpace_->printSettings(out);
}

unsigned int TaskStateSpace::ensureTaskValidity(const StateType *state) const
{
	if(!(state->validity_ & StateType::TASK_VALID) &&
	    (state->validity_ & StateType::CONF_VALID) ) {

		// compute configuration -> task mapping
		forwardMapping(state->confState_, state->taskState_);

		state->validity_ = state->validity_ | StateType::TASK_VALID;
	}

	return state->validity_;
}

void TaskStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
	// dummy
	if(t == 0.0) copyState(state,from);
	else if(t == 1.0) copyState(state,to);
}

void TaskStateSpace::setStateTaskValid(const State *state) const
{
	state->as<StateType>()->validity_ = state->as<StateType>()->validity_ | StateType::TASK_VALID;
}

void TaskStateSpace::setStateConfValid(const State *state) const
{
	state->as<StateType>()->validity_ = state->as<StateType>()->validity_ | StateType::CONF_VALID;
}

void TaskStateSpace::setStateTaskInvalid(const State *state) const
{
	state->as<StateType>()->validity_ = state->as<StateType>()->validity_ & (~StateType::TASK_VALID);
}

void TaskStateSpace::setStateConfInvalid(const State *state) const
{
	state->as<StateType>()->validity_ = state->as<StateType>()->validity_ & (~StateType::CONF_VALID);
}
			
bool TaskStateSpace::isStateTaskValid(const State *state) const
{
	return (state->as<StateType>()->validity_ & StateType::TASK_VALID);
}

bool TaskStateSpace::isStateConfValid(const State *state) const
{
	return (state->as<StateType>()->validity_ & StateType::CONF_VALID);
}
  			


TaskGeometricStateSpace::TaskGeometricStateSpace()
{
	setName("TaskGeometric" + getName()); 
}

TaskGeometricStateSpace::~TaskGeometricStateSpace()
{
}

double TaskGeometricStateSpace::getLongestValidSegmentFraction(void) const
{
	return confSpace_->getLongestValidSegmentFraction();
}

unsigned int TaskGeometricStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
	return confSpace_->validSegmentCount(state1,state2);
}

void TaskGeometricStateSpace::setLongestValidSegmentFraction(double segmentFraction)
{
	confSpace_->setLongestValidSegmentFraction(segmentFraction);
}

void TaskGeometricStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
	// interpolation is done between configurations!
	// i.e. both states need a valid associated configuration
	// NOTE: although const here, internal StateType States are modified
	
	const StateType *fromCast = from->as<StateType>();
	const StateType *toCast = to->as<StateType>();
	StateType *resCast = state->as<StateType>();
	
	ensureConfValidity(fromCast);
	ensureConfValidity(toCast);

	confSpace_->interpolate(fromCast->confState_, toCast->confState_,
	                        t, resCast->confState_);

	// configuration of resulting state is valid, task not
	resCast->validity_ = StateType::CONF_VALID;
}

void TaskGeometricStateSpace::makeStateValid(const State *state) const
{
	const StateType *stateCast = state->as<StateType>();
	ensureTaskValidity(stateCast);
	ensureConfValidity(stateCast);
}

unsigned int TaskGeometricStateSpace::ensureConfValidity(const StateType *state) const
{
	if( (state->validity_ & StateType::TASK_VALID) &&
	   !(state->validity_ & StateType::CONF_VALID) ) {

		// compute task -> configuration mapping
		inverseMapping(state->taskState_, state->confState_);

		state->validity_ = state->validity_ | StateType::CONF_VALID;
	}

	return state->validity_;
}




