#ifndef KUKADU
#define KUKADU

#include "../../src/learning/lwrregressor.hpp"
#include "../../src/learning/generalfitter.hpp"
#include "../../src/learning/generickernel.hpp"
#include "../../src/learning/tricubekernel.hpp"
#include "../../src/learning/gaussiankernel.hpp"
#include "../../src/learning/quadratickernel.hpp"
#include "../../src/learning/kernelregressor.hpp"
#include "../../src/learning/gaussianprocessregressor.hpp"
#include "../../src/learning/metric_learning/mahalanobis.hpp"
#include "../../src/learning/metric_learning/inftheoconstraints.hpp"
#include "../../src/learning/metric_learning/mahalanobicslearner.hpp"
#include "../../src/learning/metric_learning/inftheometriclearner.hpp"
#include "../../src/learning/metric_learning/togersonmetriclearner.hpp"

#include "../../src/learning/projective_simulation/core/projectivesimulator.hpp"
#include "../../src/learning/projective_simulation/application/manualreward.hpp"
#include "../../src/learning/projective_simulation/visualization/treedrawer.hpp"

#include "../../src/learning/reinforcement_learning/power.hpp"
#include "../../src/learning/reinforcement_learning/costcomputer.hpp"
#include "../../src/learning/reinforcement_learning/dmpreinforcer.hpp"
#include "../../src/learning/reinforcement_learning/gradientdescent.hpp"
#include "../../src/learning/reinforcement_learning/dmprewardcomputer.hpp"
#include "../../src/learning/reinforcement_learning/terminalcostcomputer.hpp"
#include "../../src/learning/reinforcement_learning/samplerewardcomputer.hpp"
#include "../../src/learning/reinforcement_learning/trajectorybasedreward.hpp"

#include "../../src/robot/controlqueue.hpp"
#include "../../src/robot/sensorstorage.hpp"
#include "../../src/robot/mounted/rosschunk.hpp"
#include "../../src/robot/kukiecontrolqueue.hpp"
#include "../../src/robot/mounted/generichand.hpp"
#include "../../src/robot/mounted/plottinghand.hpp"
#include "../../src/robot/plottingcontrolqueue.hpp"

#include "../../src/robot/kinematics/kinematics.hpp"
#include "../../src/robot/kinematics/pathplanner.hpp"
#include "../../src/robot/kinematics/pathplanner.hpp"
#include "../../src/robot/kinematics/simpleplanner.hpp"
#include "../../src/robot/kinematics/moveitkinematics.hpp"

#include "../../src/trajectory/dmpexecutor.hpp"
#include "../../src/trajectory/dmpgeneralizer.hpp"
#include "../../src/trajectory/jointdmplearner.hpp"
#include "../../src/trajectory/trajectoryexecutor.hpp"
#include "../../src/trajectory/cartesiandmplearner.hpp"
#include "../../src/trajectory/trajectorygenerator.hpp"
#include "../../src/trajectory/trajectorycomparator.hpp"
#include "../../src/trajectory/dmptrajectorygenerator.hpp"
#include "../../src/trajectory/polytrajectorygenerator.hpp"

#include "../../src/types/dmp.hpp"
#include "../../src/types/dmpbase.hpp"
#include "../../src/types/jointdmp.hpp"
#include "../../src/types/trajectory.hpp"
#include "../../src/types/sensordata.hpp"
#include "../../src/types/querypoint.hpp"
#include "../../src/types/kukadutypes.hpp"
#include "../../src/types/cartesiandmp.hpp"
#include "../../src/types/dictionarytrajectory.hpp"
#include "../../src/types/singlesampletrajectory.hpp"

#include "../../src/manipulation/controller.hpp"
#include "../../src/manipulation/complexcontroller.hpp"
#include "../../src/manipulation/sensingcontroller.hpp"
#include "../../src/manipulation/haptic/hapticplanner.hpp"

#include "../../src/utils/types.hpp"
#include "../../src/utils/utils.hpp"
#include "../../src/utils/tictoc.hpp"
#include "../../src/utils/customset.hpp"
#include "../../src/utils/kukadutokenizer.hpp"
#include "../../src/utils/conversion_utils.hpp"
#include "../../src/utils/destroyableobject.hpp"

#ifdef CPP11SUPPORTED
	#include "../../src/robot/kinematics/komoplanner.hpp"
#else
	#include "../../src/vision/kinect.hpp"
	#include "../../src/vision/pcltools.hpp"
	#include "../../src/vision/openboxfilter.hpp"
	#include "../../src/vision/visioninterface.hpp"
	#include "../../src/vision/planarcuttransformator.hpp"
#endif


#endif
