#include "kernelgraph.h"

#include <thread>
#include <chrono>

KernelGraph::KernelGraph(boost::shared_ptr<AL::ALBroker> broker) :
    broker_(broker),
    motion_(broker),
#if MOVEMENTGRAPH_IS_REMOTE
    life_proxy_(broker),
#endif
    posture_(broker) {}

Vertex KernelGraph::GetCurrentState() const {
  bool useSensors = true;

  std::vector <float> result_value    = motion_.getAngles(PARAM_NAMES, useSensors);
  std::vector <float> result_hardness = motion_.getStiffnesses(PARAM_NAMES);
  
  return Vertex(result_value, result_hardness, true);
}

bool KernelGraph::RunChain(const std::vector<std::string> &chain,
                           int cnt,
                           float acceleration) {
  assert(cnt > 0);
  assert(chain.size() > 1);

  std::vector<const Edge *> way, full_way;

  if (!FindWayThroughVertexes(chain, way)) {
    return false;
  }

  for (int i = 0; i < cnt; ++i) {
    for (int j = 0; j < way.size(); ++j) {
      full_way.push_back(way[j]);
    }
  }

  Run(full_way[0]->GetBegin());
  RunWay(full_way, acceleration);
  return true;
}

bool KernelGraph::Run(const std::string &v_name, float time, float timeStiffness) {
  if (!IsVertexContains(v_name)) {
    return false;
  }

  Run(GetVertex(v_name), time, timeStiffness);
  return true;
}

void KernelGraph::Run(const Vertex* v, float time, float timeStiffness) {
  assert(v != nullptr);
  assert(time > 0);
  
  motion_.stiffnessInterpolation(PARAM_NAMES, v->GetHardnessValues(), time);
  motion_.angleInterpolation(PARAM_NAMES, v->GetRadianValues(), timeStiffness, true);
}

void KernelGraph::Rest() const {
  motion_.rest();
}

void KernelGraph::Wake() const {
  motion_.wakeUp();
}

void KernelGraph::StrongRest() const {
  std::vector<float> param;
  for (int i = 0; i < PARAM_NUM_; ++i) {
    param.push_back(0);
  }
  motion_.setStiffnesses(PARAM_NAMES, param);
}

void KernelGraph::StrongWake() const {
  std::vector<float> param;
  for (int i = 0; i < PARAM_NUM_; ++i) {
    param.push_back(1);
  }
  motion_.setStiffnesses(PARAM_NAMES, param);
}

void KernelGraph::BehaviorOff() const {
#ifdef MOVEMENTGRAPH_IS_REMOTE
  life_proxy_.setState("disabled");
#endif
}

void KernelGraph::StopMove() const {
  motion_.stopMove();
  motion_.setMoveArmsEnabled(false, false);
}

void KernelGraph::StartMove() const {
  posture_.goToPosture("StandInit", 0.5);
}

float KernelGraph::GetHeadVerticalAngle() {
  Vertex curr = GetCurrentState();
  return -curr.GetDegreesValues()[1];
}

float KernelGraph::GetHeadHorizontalAngle() {
  Vertex curr = GetCurrentState();
  return curr.GetDegreesValues()[0];
}

void KernelGraph::SetHeadVerticalAngle(float angle) {
  angle = std::min(angle, static_cast<float>(38.5));
  angle = std::max(angle, static_cast<float>(-38.5));

  float fractionMaxSpeed = 0.3;
  motion_.setAngles(PARAM_NAMES[1], -angle * TO_RAD, fractionMaxSpeed);
}

void KernelGraph::SetHeadHorizontalAngle(float angle) {
  angle = std::min(angle, static_cast<float>(119.5));
  angle = std::max(angle, static_cast<float>(-119.5));

  float fractionMaxSpeed = 0.3;
  motion_.setAngles(PARAM_NAMES[0], angle * TO_RAD, fractionMaxSpeed);
}

void KernelGraph::ToInit() {
  float time = 1;
  Run("INIT", time);
}

void KernelGraph::LookDown(int level) {
  assert(level <= 7);
  std::vector<std::string> names({"INIT", "FB" + std::to_string(level)});
  RunChain(names, 1);
}

void KernelGraph::GetUpFront() {
  std::vector<std::string> names({"GUF0", "GUF15"});
  RunChain(names, 1);
  //posture_.goToPosture("StandInit", 0.5);
}

void KernelGraph::GetUpBack() {
  std::vector<std::string> names({"GUB0", "GUB14"});
  RunChain(names, 1);
}

bool KernelGraph::ToPoint(const std::string &finish_name) {
  std::vector<const Edge *> way;

  std::string start_name = GetNearestVertex(Vertex(GetCurrentState()))->GetName();

  if (!FindWayToVertexFromVertex(start_name, finish_name, way)) {
    return false;
  } else {
    RunWay(way, 1);
    return true;
  }
}

void KernelGraph::SetStiffness(std::vector<std::string> motors_, 
                               std::vector<float> stiffnesses, 
                               float time) const {
  motion_.stiffnessInterpolation(motors_, stiffnesses, time);
}

/*------- PRIVAT SPACE ---------*/

void KernelGraph::RunWay(std::vector <const Edge*> edges, float acceleration) {
  assert(acceleration > 0);
  if (edges.empty()) {
    return;
  }

  AL::ALValue angleLists;
  AL::ALValue timeLists;
  std::vector<float> time_list;
  float curr_time = 0;

  for (int i = 0; i < edges.size(); ++i) {
    Run(edges[i]->GetEnd(), edges[i]->GetTime());
  }
}

void KernelGraph::RunWayBezier(std::vector <const Edge*> edges, float acceleration) {
  assert(acceleration > 0);
  if (edges.empty()) {
    return;
  }

  AL::ALValue angleLists;
  AL::ALValue timeLists;
  std::vector<std::vector<float>> params_list;
  std::vector<float> time_list;
  float curr_time = 0;

  for (int i = 0; i < edges.size(); ++i) {
    curr_time += edges[i]->GetTime() * acceleration;
    time_list.push_back(curr_time);
    params_list.push_back(edges[i]->GetEnd()->GetRadianValues());
  }

  for (int i = 0; i < PARAM_NUM_; ++i) {
    std::vector<float> joint_path;
    for (int j = 0; j < params_list.size(); ++j) {
      float angle = params_list[j][i];
      joint_path.push_back(angle);
    }
    timeLists.arrayPush(time_list);
    angleLists.arrayPush(joint_path);
  }

  motion_.angleInterpolationBezier(PARAM_NAMES, timeLists, angleLists);
}

void KernelGraph::Rotate(float theta) {
  assert(fabs(theta) <= PI);

  float time = 1;
  Run("INIT", time);

  theta = GetRealAngle(theta);
  float x_speed, y_speed, t_speed, time_rotate;
  time_rotate = fabs(theta / THETA_VELOCITY);
  x_speed = 0;
  y_speed = 0;
  t_speed = theta / time_rotate;

  MoveParams params;
  params.SetParam("MaxStepFrequency", 1.0);
  params.SetParam("MaxStepX", 0.02);
  params.SetParam("MaxStepY", 0.10);
  params.SetParam("StepHeight", 0.02);

  motion_.move(x_speed, y_speed, t_speed, params.GetParams());
  sleep(time_rotate);
  motion_.stopMove();
}

void KernelGraph::GoForwardFast() {
  float time = 1;
  Run("INIT", time);

  MoveParams params;
  params.SetParam("MaxStepX", 0.04);
  params.SetParam("StepHeight", 0.027);
  params.SetParam("TorsoWy", 0.01);

  motion_.setMoveArmsEnabled(true, true);

  float X_VELOCITY_ = 0.15;

  motion_.move(X_VELOCITY_, 0, 0, params.GetParams());
}

void KernelGraph::GoBackFast() {
  float time = 1;
  Run("INIT", time);

  MoveParams params;
  params.SetParam("MaxStepX", 0.06);
  params.SetParam("StepHeight", 0.027);
  params.SetParam("TorsoWy", 0.01);

  motion_.setMoveArmsEnabled(true, true);

  float X_VELOCITY_ = 0.15;

  motion_.move(-X_VELOCITY_, 0, 0, params.GetParams());
}

void KernelGraph::GoLeftFast() {
  float time = 1;
  Run("INIT", time);

  MoveParams params;
  params.SetParam("StepHeight", 0.027);
  params.SetParam("TorsoWy", 0.01);

  motion_.setMoveArmsEnabled(true, true);

  float Y_VELOCITY_ = 0.065;

  motion_.move(0, Y_VELOCITY_, 0, params.GetParams());
}

void KernelGraph::GoRightFast() {
  float time = 1;
  Run("INIT", time);

  MoveParams params;
  params.SetParam("StepHeight", 0.027);
  params.SetParam("TorsoWy", -0.1);

  motion_.setMoveArmsEnabled(true, true);

  float Y_VELOCITY_ = 0.065;

  motion_.move(0, -Y_VELOCITY_, 0, params.GetParams());
}

float KernelGraph::GetRealAngle(float theta) const {
  float sign = (theta < 0) ? -1 : 1;
  float new_theta = (fabs(theta) + 30 * TO_RAD) * 9 / 10;
  return sign * new_theta;
}

void KernelGraph::ComplexTest() {
  Wake();
  float time = 1;
  Run("INIT", time);
  GoForwardFast();
  sleep(5);
  StopMove();
  GoBackFast();
  sleep(5);
  StopMove();
  GoLeftFast();
  sleep(5);
  StopMove();
  GoRightFast();
  sleep(5);
  StopMove();
  Rotate(45 * TO_RAD);
  Rotate(-45 * TO_RAD);
}

void KernelGraph::PullLegsTogether() {
  float time = 1;
  Run("INIT", time);
  GoLeftFast();
  std::this_thread::sleep_for(std::chrono::milliseconds(1450));
  StopMove();
}