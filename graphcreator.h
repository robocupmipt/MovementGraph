#include "kernelgraph.h"

class GraphCreator : public AL::ALModule {
 public:
  GraphCreator(boost::shared_ptr<AL::ALBroker> pBroker, const std::string& pName);

  ~GraphCreator();
  virtual void init();

  void Rest();

  void Wake();

  void BehaviorOff();

  void Snap();

  void SetName();

  void RunBuffer();

  void Reflect();

  void CopyToRight();

  void CopyToLeft();

  void Save();

  void Run();

  void Test();

  void RKick();

  void LKick();

  void Move();

  void MoveFast();

  void GoForvard();
  
  void GoBack();

  void Rotate();

  void StartMove();

  void StopMove();

  void SetHeadVerticalAngle();

  void SetHeadHorizontalAngle();

  void Fun();

  void GetUpFront();

  void GetUpBack();

  void ToPoint();

 private:
  bool CheckBuffer() const;

  bool IsBufferEmpty() const;

  void SetBuffer(const Vertex& v);

  void ClearBuffer();

  template <typename T=std::string>
  T SmallLog(const std::string text, size_t deep_level, bool is_reply=false) const;

 private:
  KernelGraph graph_;
  Vertex* vertex_buffer_;
};