/* Author: Matthias Behnisch, mbehnisc@cor-lab.uni-bielefeld.de */

#ifndef OMPL_CONTRIB_TASK_MOTION_VALIDATOR_
#define OMPL_CONTRIB_TASK_MOTION_VALIDATOR_

#include "ompl/base/DiscreteMotionValidator.h"
#include "TaskStateSpace.h"


namespace ompl
{
	namespace base
	{
		OMPL_CLASS_FORWARD(TaskMotionValidator);
		class TaskMotionValidator : public DiscreteMotionValidator
		{
		public:
			TaskMotionValidator(SpaceInformation *si);
			TaskMotionValidator(const SpaceInformationPtr &si);
			virtual ~TaskMotionValidator();

            virtual bool checkMotion(const State *s1, const State *s2) const;
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const;

			virtual void computeMotionCost(const State *s1, const State *s2, double &cost, std::pair<double, double> &bounds) const;

		protected:
			TaskGeometricStateSpace* getAndCheckTaskGeometricStateSpace(SpaceInformation *si);

			TaskGeometricStateSpace *taskGeometricStateSpace_;
		};
	}
}


#endif

