/* Author: Matthias Behnisch, mbehnisc@cor-lab.uni-bielefeld.de */

#include "../TaskMotionValidator.h"

using namespace ompl;
using namespace ompl::base;

TaskMotionValidator::TaskMotionValidator(SpaceInformation *si)
	: DiscreteMotionValidator(si)
{
	taskGeometricStateSpace_ = getAndCheckTaskGeometricStateSpace(si);
}

TaskMotionValidator::TaskMotionValidator(const SpaceInformationPtr &si)
	: DiscreteMotionValidator(si)
{
	taskGeometricStateSpace_ = getAndCheckTaskGeometricStateSpace(si.get());
}

TaskGeometricStateSpace* TaskMotionValidator::getAndCheckTaskGeometricStateSpace(SpaceInformation *si)
{
	TaskGeometricStateSpace *space = dynamic_cast<TaskGeometricStateSpace*>(si->getStateSpace().get());
	if(space) return space;
	else throw Exception("TaskMotionValidator needs a TaskGeometricStateSpace");
}

TaskMotionValidator::~TaskMotionValidator()
{
}

bool TaskMotionValidator::checkMotion(const State *s1, const State *s2) const
{
	// ensure that both states are valid
	taskGeometricStateSpace_->makeStateValid(s1);
	taskGeometricStateSpace_->makeStateValid(s2);
	// pass on to DiscreteMotionValidator
	return DiscreteMotionValidator::checkMotion(s1,s2);
}

bool TaskMotionValidator::checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
{
	// ensure that both states are valid
	taskGeometricStateSpace_->makeStateValid(s1);
	taskGeometricStateSpace_->makeStateValid(s2);
	// pass on to DiscreteMotionValidator
	return DiscreteMotionValidator::checkMotion(s1,s2,lastValid);
}
