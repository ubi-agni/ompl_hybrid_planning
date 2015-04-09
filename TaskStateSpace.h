/* Author: Matthias Behnisch, mbehnisc@cor-lab.uni-bielefeld.de */

#ifndef OMPL_CONTRIB_TASK_STATE_SPACE_
#define OMPL_CONTRIB_TASK_STATE_SPACE_


#include "ompl/base/StateSpace.h"


namespace ompl
{
namespace base
{
class TaskStateSampler : public StateSampler
{
public:
	TaskStateSampler(const StateSpace *space)
		: StateSampler(space)
	{
		taskSampler_ = space->allocStateSampler();
	}

	virtual void sampleUniform(State *state);
	virtual void sampleUniformNear(State *state, const State *near, const double distance);
	virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

protected:
	StateSamplerPtr taskSampler_;
};


class TaskStateSpace : public StateSpace
{
	friend class TaskStateSampler;

public:
	class StateType : public State
	{
	public:
		mutable State *taskState_;
		mutable State *confState_;

		// flag if state components are valid, i.e.
		// if configuration has associated task,
		// if task has associated configuration
		static const unsigned int NONE_VALID = 0;
		static const unsigned int CONF_VALID = 1;
		static const unsigned int TASK_VALID = 2;
		static const unsigned int ALL_VALID  = 3;

		mutable unsigned int validity_;
	};

	TaskStateSpace();
	virtual ~TaskStateSpace();

	// from StateSpace
	virtual void setup(void);

	virtual unsigned int getDimension(void) const;
	virtual double getMaximumExtent(void) const;
	virtual void enforceBounds(State *state) const;
	virtual bool satisfiesBounds(const State *state) const;

	virtual void copyState(State *destination, const State *source) const;
	virtual double distance(const State *state1, const State *state2) const;
	virtual bool equalStates(const State *state1, const State *state2) const;

	virtual unsigned int getSerializationLength(void) const;
	virtual void serialize(void *serialization, const State *state) const;
	virtual void deserialize(State *state, const void *serialization) const;

	virtual StateSamplerPtr allocDefaultStateSampler(void) const;
	//virtual void registerProjections(void);   TODO

	virtual State* allocState(void) const;
	virtual void freeState(State *state) const;

	virtual double* getValueAddressAtIndex(State *state, const unsigned int index) const;

	virtual void printState(const State *state, std::ostream &out) const;
	virtual void printSettings(std::ostream &out) const;

	virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

	// additional
	void setStateTaskValid(const State *state) const;
	void setStateConfValid(const State *state) const;

	void setStateTaskInvalid(const State *state) const;
	void setStateConfInvalid(const State *state) const;

	bool isStateTaskValid(const State *state) const;
	bool isStateConfValid(const State *state) const;

protected:
	// implement for actual statespace creation, called during setup()
	virtual StateSpace* createTaskSpace() = 0;
	virtual StateSpace* createConfSpace() = 0;

	// implement configuration -> task mapping
	virtual void forwardMapping(const State *conf, State *task) const = 0;

	unsigned int ensureTaskValidity(const StateType *state) const;

	StateSpacePtr taskSpace_;
	StateSpacePtr confSpace_;
};


class TaskGeometricStateSpace : public TaskStateSpace
{
public:
	TaskGeometricStateSpace();
	virtual ~TaskGeometricStateSpace();

	virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

	// configuration space segments
	virtual double getLongestValidSegmentFraction(void) const;
	virtual unsigned int validSegmentCount(const State *state1, const State *state2) const;
	virtual void setLongestValidSegmentFraction(double segmentFraction);

	void makeStateValid(const State *state) const;

protected:
	// implement task -> configuration mapping
	virtual void inverseMapping(const State *task, State *conf) const = 0;

	unsigned int ensureConfValidity(const StateType *state) const;
};
}
}

#endif
