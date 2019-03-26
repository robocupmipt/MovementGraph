#pragma once

#include "kernelgraph.h"

class MovementGraph : public AL::ALModule {
 public:
  MovementGraph(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
  ~MovementGraph();

  virtual void init();

  void GoForward();

  void GoBack();

  void GoRight();

  void GoLeft();

  void Rotate(float theta);

  void StartMove();

  void StopMove();

  float GetHeadVerticalAngle();

  float GetHeadHorizontalAngle();

  void SetHeadVerticalAngle(float angle);

  void SetHeadHorizontalAngle(float angle);

  void LookDown(int level);

  void ToInit();

  void GetUpFront();

  void GetUpBack();
 private:
  KernelGraph graph_;
};