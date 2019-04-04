#include <boost/shared_ptr.hpp>
#include <qi/log.hpp>
#include <queue>
#include <fstream>
#include <alcommon/albroker.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/albasicawarenessproxy.h>
#ifdef MOVEMENTGRAPH_IS_REMOTE
# include <alproxies/alautonomouslifeproxy.h>
#endif
#include <alproxies/alrobotpostureproxy.h>
#include "primalgraph.h"

namespace AL {
class ALBroker;
}

class KernelGraph : public PrimalGraph {
 public:
  KernelGraph(boost::shared_ptr<AL::ALBroker> broker_);

  Vertex GetCurrentState() const;

  bool RunChain(const std::vector<std::string> &chain,
                int cnt,
                float acceleration = DEFAULT_ACCELERATION);

  bool Run(const std::string &v_name, float time = DEFAULT_TIME, float timeStiffness = DEFAULT_TIME_STIFFNESS);

  void Run(const Vertex *v, float time = DEFAULT_TIME, float timeStiffness = DEFAULT_TIME_STIFFNESS);

  void Rest() const;

  void Wake() const;

  void StrongRest() const;

  void StrongWake() const;

  void BehaviorOff() const;

  void GoForwardFast();

  void GoBackFast();

  void GoLeftFast();

  void GoRightFast();

  void CircumferentialMotionPrototype(float rotation_speed);

  void StopMove() const;

  void StartMove() const;

  void Rotate(float theta);

  void PullLegsTogether();

  void GetUpFront();

  void GetUpBack();

  float GetHeadVerticalAngle();

  float GetHeadHorizontalAngle();

  void SetHeadVerticalAngle(float angle);

  void SetHeadHorizontalAngle(float angle);

  void ToInit();

  void LookDown(int level);

  // Validation
  void ComplexTest();

  void SetStiffness(std::vector<std::string> motors_, std::vector<float> stiffnesses, float time) const;

  // move position robot to finish via graph
  bool ToPoint(const std::string &finish_name);
 private:

  void RunWay(std::vector<const Edge *> edges, float acceleration);
  
  void RunWayBezier(std::vector<const Edge *> edges, float acceleration);

  void RunWayDimka(std::vector<const Edge *> edges, float acceleration);

  float GetRealAngle(float theta) const;

  float SimplifyAngle(float theta) const;

 private:
  boost::shared_ptr<AL::ALBroker> broker_;
  mutable AL::ALMotionProxy motion_;
#ifdef MOVEMENTGRAPH_IS_REMOTE
  mutable AL::ALAutonomousLifeProxy life_proxy_;
#endif
  mutable AL::ALRobotPostureProxy posture_;
};