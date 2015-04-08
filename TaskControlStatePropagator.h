/* Author: Matthias Behnisch, mbehnisc@cor-lab.uni-bielefeld.de */

#ifndef OMPL_CONTROL_TASK_CONTROL_PROPAGATOR_
#define OMPL_CONTROL_TASK_CONTROL_PROPAGATOR_

#include "ompl/contrib/task_level_planning/TaskStateSpace.h"
#include "ompl/control/SpaceInformation.h"
#include "ompl/control/StatePropagator.h"

namespace ompl
{
	namespace control
	{
		class TaskControlStatePropagator : public StatePropagator
		{
		public:	
			TaskControlStatePropagator(SpaceInformation* si);
			TaskControlStatePropagator(const SpaceInformationPtr &si);
			virtual ~TaskControlStatePropagator();

			virtual void propagate(const base::State *state, const Control* control, const double duration, base::State *result) const;
			virtual bool canPropagateBackward(void) const;
			
		protected:
			virtual void taskControlTransition(const base::State *taskState, const Control* control,
			                                  const double duration, base::State *resultTaskState) const = 0;
			virtual void confTowardsTaskTransition(const base::State *confState, const base::State *taskState,
			                                       const double duration, base::State *resultConfState) const = 0;
		
			//TaskControlSpace *controlSpace_;

		};
	}
}


#endif
