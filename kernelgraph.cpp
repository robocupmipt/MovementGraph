#include "kernelgraph.h"

KernelGraph::KernelGraph(boost::shared_ptr<AL::ALBroker> broker) :
    broker_(broker),
    motion_(broker),
#if MOVEMENTGRAPH_IS_REMOTE
    life_proxy_(broker),
#endif
    posture_(broker) {}

Vertex KernelGraph::GetCurrentState() const {
  bool useSensors = true;

  std::vector <float> result = motion_.getAngles(PARAM_NAMES, useSensors);
  return Vertex(result, true);
}

bool KernelGraph::RunChain(const std::vector <std::string>& chain,
              int cnt,
              float acceleration) {
  assert(cnt > 0);
  assert(chain.size() > 1);

  std::vector <const Edge*> way, full_way;

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

bool KernelGraph::Run(const std::string& v_name, float time) {
  if (!IsVertexContains(v_name)) {
    return false;
  }

  Run(GetVertex(v_name), time);
  return true;
}

void KernelGraph::Run(const Vertex* v, float time) {
  assert(v != nullptr);
  assert(time > 0);

  motion_.angleInterpolation(PARAM_NAMES, v->GetRadianValues(), time, true);
}

void KernelGraph::Rest() const {
  motion_.rest();
}

void KernelGraph::Wake() const {
  motion_.wakeUp();
}

void KernelGraph::StrongRest() const {
  std::vector <float> param;
  for (int i = 0; i < PARAM_NUM_; ++i) {
    param.push_back(0);
  }
  motion_.setStiffnesses(PARAM_NAMES, param);
}

void KernelGraph::StrongWake() const {
  std::vector <float> param;
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

void KernelGraph::Move(float x, float y, float theta) {
  motion_.wakeUp();

  float first_rotate = atan2(x, y) - PI / 2;
  float len = sqrt(x * x + y * y);
  float second_rotate = theta - first_rotate;

  if (len > EPS) {
    Rotate(first_rotate);
    GoForward(len);
    Rotate(second_rotate);
  } else {
    Rotate(theta);
  }
  posture_.goToPosture("StandInit", 0.5);
}

void KernelGraph::MoveFast(float x, float y, float theta) {
  motion_.wakeUp();

  float first_rotate = atan2(x, y) - PI / 2;
  float len = sqrt(x * x + y * y);
  float second_rotate = theta - first_rotate;

  if (first_rotate > EPS) {
    Rotate(first_rotate);
  }
  if (len > EPS) {
    GoForvardFast(len);
  } 
  if (second_rotate > EPS) {
    Rotate(second_rotate);
  }
  //posture_.goToPosture("StandInit", 0.5);
}


void KernelGraph::SetTheta(float theta, float len) const {
  StopMove();

  Rotate(theta);

  GoForvardFast(len);
}

void KernelGraph::StopMove() const {
  motion_.stopMove();
  motion_.setMoveArmsEnabled(false, false);
}

void KernelGraph::StartMove() const {
  posture_.goToPosture("StandInit", 0.5);
}


void KernelGraph::RightKick() {

}


void KernelGraph::LeftKick() {
}

// end copy


void KernelGraph::Fun() {
  boost::shared_ptr<AL::ALProxy> testProxy
      = boost::shared_ptr<AL::ALProxy>(new AL::ALProxy(broker_, "MovementGraph"));
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
  assert(angle <= 38.5);
  assert(angle >= -29.5);

  float fractionMaxSpeed  = 0.3;
  motion_.setAngles(PARAM_NAMES[1], -angle * TO_RAD, fractionMaxSpeed);
}

void KernelGraph::SetHeadHorizontalAngle(float angle) {
  assert(angle <= 119.5);
  assert(angle >= -119.5);

  float fractionMaxSpeed  = 0.3;
  motion_.setAngles(PARAM_NAMES[0], angle * TO_RAD, fractionMaxSpeed);
}

void KernelGraph::ToInit() {
  float time = 1;
  Run("INIT", time);
}

void KernelGraph::LookDown(int level) {
  assert(level <= 7);
  std::vector <std::string> names({"INIT", "FB" + std::to_string(level)});
  RunChain(names, 1);
}

void KernelGraph::GetUpFront() {
  std::vector <std::string> names({"GUF0", "GUF15"});
  RunChain(names, 1);
  //posture_.goToPosture("StandInit", 0.5);
}

void KernelGraph::GetUpBack() {
  std::vector <std::string> names({"GUB0", "GUB14"});
  RunChain(names, 1);
}

/*------- PRIVAT SPACE ---------*/

bool KernelGraph::ToPoint(const std::string& finish_name) {
  std::vector <const Edge*> way;

  std::string start_name = GetNearestVertex(Vertex(GetCurrentState()))->GetName();

  if (!FindWayToVertexFromVertex(start_name, finish_name, way)) {
    return false;
  } else {
    RunWayDimka(way, 1);
    return true;
  }
}

void KernelGraph::RunWayDimka(std::vector <const Edge*> edges, float acceleration) {
  assert(acceleration > 0);
  if (edges.empty()) {
    return;
  }


  AL::ALValue angleLists;
  AL::ALValue timeLists;
  std::vector <float> time_list;
  float curr_time = 0;

  for (int i = 0; i < edges.size(); ++i) {
    Run(edges[i]->GetEnd(), edges[i]->GetTime());
  }
}

void KernelGraph::RunWay(std::vector <const Edge*> edges, float acceleration) {
  assert(acceleration > 0);
  if (edges.empty()) {
    return;
  }


  AL::ALValue angleLists;
  AL::ALValue timeLists;
  std::vector <std::vector <float> > params_list;
  std::vector <float> time_list;
  float curr_time = 0;

  for (int i = 0; i < edges.size(); ++i) {
    curr_time += edges[i]->GetTime() * acceleration;
    time_list.push_back(curr_time);
    params_list.push_back(edges[i]->GetEnd()->GetRadianValues());
  }


  for (int i = 0; i < PARAM_NUM_; ++i) {
    std::vector <float> joint_path;
    for (int j = 0; j < params_list.size(); ++j) {
      float angle = params_list[j][i];
      joint_path.push_back(angle);
    }
    timeLists.arrayPush(time_list);
    angleLists.arrayPush(joint_path);
  }

  motion_.angleInterpolationBezier(PARAM_NAMES, timeLists, angleLists);
}

void KernelGraph::Rotate(float theta) const {
  assert(fabs(theta) <= PI);

  posture_.goToPosture("StandInit", 0.5);

  theta = GetRealAngle(theta);
  float x_speed, y_speed, t_speed, time_rotate;
  time_rotate = fabs(theta / THETA_VELOCITY);
  x_speed     = 0;
  y_speed     = 0;
  t_speed     = theta / time_rotate;

  MoveParams params;
  params.SetParam("MaxStepFrequency", 1.0);
  params.SetParam("MaxStepX", 0.02);
  params.SetParam("MaxStepY", 0.10);
  params.SetParam("StepHeight", 0.02);

  motion_.move(x_speed, y_speed, t_speed, params.GetParams());
  sleep(time_rotate);
  motion_.stopMove();
}

void KernelGraph::GoForward(float len) const {
  assert(len >= 0);

  posture_.goToPosture("StandInit", 0.5);

  MoveParams params;
  params.SetParam("MaxStepX", 0.06);
  params.SetParam("StepHeight", 0.027);
  params.SetParam("TorsoWy", 0.01);
  /*
  params.SetParam("MaxStepFrequency", 0.0);

  params.SetParam("TorsoWy", 0.12);


  */

  float counting_len = len;
  motion_.setMoveArmsEnabled(true, true);
  while (counting_len > EPS) {
    float curr_len = std::min(counting_len, STEP_CHAIN);
    float time_walk = curr_len / X_VELOCITY;

    motion_.move(X_VELOCITY, 0, 0, params.GetParams());
    sleep(time_walk);
    motion_.stopMove();
    counting_len -= curr_len;
  }
  motion_.setMoveArmsEnabled(false, false);
}

void KernelGraph::GoForvardFast(float len) const {
  assert(len >= 0);
  
  MoveParams params;
  params.SetParam("MaxStepX", 0.06);
  params.SetParam("StepHeight", 0.027);
  params.SetParam("TorsoWy", 0.01);

/*
  posture_.goToPosture("StandInit", 0.5);

  MoveParams params;
  params.SetParam("MaxStepX", 0.08);
  params.SetParam("StepHeight", 0.035);
  params.SetParam("TorsoWy", 0.0122);
*/

  /*
  params.SetParam("MaxStepFrequency", 0.0);

  params.SetParam("TorsoWy", 0.12);
  */

  motion_.setMoveArmsEnabled(true, true);

  float X_VELOCITY_ = 0.1;
  float time_walk = len / X_VELOCITY_;

  motion_.move(X_VELOCITY_, 0, 0, params.GetParams());
 }

void KernelGraph::GoBackFast(float len) const {
  assert(len >= 0);
  
  MoveParams params;
  params.SetParam("MaxStepX", 0.06);
  params.SetParam("StepHeight", 0.027);
  params.SetParam("TorsoWy", 0.01);

/*
  posture_.goToPosture("StandInit", 0.5);

  MoveParams params;
  params.SetParam("MaxStepX", 0.08);
  params.SetParam("StepHeight", 0.035);
  params.SetParam("TorsoWy", 0.0122);
*/

  /*
  params.SetParam("MaxStepFrequency", 0.0);

  params.SetParam("TorsoWy", 0.12);
  */

  motion_.setMoveArmsEnabled(true, true);

  float X_VELOCITY_ = 0.1;
  float time_walk = len / X_VELOCITY_;

  motion_.move(-X_VELOCITY_, 0, 0, params.GetParams());
 }

float KernelGraph::GetRealAngle(float theta) const {
  float sign = (theta < 0) ? -1 : 1;
  float new_theta = (fabs(theta) + 30 * TO_RAD) * 9 / 10;
  return sign * new_theta;
}

