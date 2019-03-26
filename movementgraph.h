#pragma once

#include "kernelgraph.h"

class MovementGraph : public AL::ALModule {
 public:
  MovementGraph(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
  ~MovementGraph();

  virtual void init();

  // Start forward motion
  void GoForward();

  // Start backward motion
  void GoBack();

  // Start right motion
  void GoRight();

  // Start left motion
  void GoLeft();

  // Rotate theta (in radians)
  void Rotate(float theta);

  // Prepare robot for motion
  void StartMove();

  // Stop moving
  void StopMove();

  float GetHeadVerticalAngle();

  float GetHeadHorizontalAngle();

  void SetHeadVerticalAngle(float angle);

  void SetHeadHorizontalAngle(float angle);

  // Tilts the robot's body in order to find the ball
  // level is a number from 0 to 7
  void LookDown(int level);

  // Puts robot to initial position
  void ToInit();

  void GetUpFront();

  void GetUpBack();
 private:
  KernelGraph graph_;
};