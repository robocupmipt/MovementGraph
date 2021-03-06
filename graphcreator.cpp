#include "graphcreator.h"

GraphCreator::GraphCreator(boost::shared_ptr<AL::ALBroker> pBroker, const std::string& pName) :
  ALModule(pBroker, pName),
  graph_(pBroker),
  vertex_buffer_(nullptr) {}

GraphCreator::~GraphCreator() {
  ClearBuffer();
}

void GraphCreator::init() {
  // инициализируем граф
  graph_.Initialize();


  while (true) {
    std::string command(SmallLog("ENTER command", 1, true));

    if (command == "REST") {
      Rest();
    }
    else if (command == "WAKE") {
      Wake();
    }
    else if (command == "SNAP") {
      Snap();
    }
    else if (command == "B_OFF") {
      BehaviorOff();
    }
    else if (command == "SET_NAME") {
      SetName();
    }
    else if (command == "RUN_B") {
      RunBuffer();
    }
    else if (command == "REFL") {
      Reflect();
    }
    else if (command == "LIKE_LEFT") {
      CopyToRight();
    }
    else if (command == "LIKE_RIGHT") {
      CopyToLeft();
    }
    else if (command == "SAVE") {
      Save();
    }
    else if (command == "RUN") {
      Run();
    }
    else if (command == "TEST") {
      Test();
    }
    else if (command == "RKICK") {
      RKick();
    }
    else if (command == "LKICK") {
      LKick();
    }
    else if (command == "EXIT") {
      break;
    }
    else if (command == "MOVE") {
      Move();
    }
    else {
      SmallLog("UNKNOWN COMMAND", 2);
    }
  }
}

void GraphCreator::Rest() {
  SmallLog("Going to the rest Position", 2);

  graph_.Rest();
}

void GraphCreator::Wake() {
  SmallLog("Waking up", 2);

  graph_.Wake();
}

void GraphCreator::BehaviorOff() {
  SmallLog("Off standard behavior", 2);
  graph_.BehaviorOff();
}

void GraphCreator::Snap() {
  SmallLog("Copy current State to the buffer", 2);

  SetBuffer(graph_.GetCurrentState());
}

void GraphCreator::SetName() {
  if (CheckBuffer()) {
    return;
  }
  assert(vertex_buffer_ != nullptr);

  std::string name(SmallLog("ENTER name for buffer vertex:", 2, true));

  vertex_buffer_->SetName(name);
}

void GraphCreator::RunBuffer() {
  if (CheckBuffer()) {
    return;
  }
  assert(vertex_buffer_ != nullptr);

  SmallLog("Going to the buffer position", 2);

  graph_.Run(vertex_buffer_);
}

void GraphCreator::Reflect() {
  if (CheckBuffer()) {
    return;
  }
  assert(vertex_buffer_ != nullptr);

  SmallLog("Change Left and Right sides", 2);

  vertex_buffer_->Reflect();
}

void GraphCreator::CopyToRight() {
  if (CheckBuffer()) {
    return;
  }
  assert(vertex_buffer_ != nullptr);

  SmallLog("Copy Left state  to Right side", 2);

  vertex_buffer_->CopyFromSide(LEFT);
}

void GraphCreator::CopyToLeft() {
  if (CheckBuffer()) {
    return;
  }
  assert(vertex_buffer_ != nullptr);

  SmallLog("Copy Right state to Left side", 2);

  vertex_buffer_->CopyFromSide(RIGHT);
}

void GraphCreator::Save() {
  if (CheckBuffer()) {
    return;
  }
  if (vertex_buffer_->GetName() == "") {
    SmallLog("ERROR Buffer doesn't have a name!", 2);
    return;
  }
  assert(vertex_buffer_ != nullptr);

  SmallLog("Saving to file " + VERTEXES_FILE, 2);
  std::ofstream out(VERTEXES_FILE, std::ios_base::app);

  vertex_buffer_->PrintState(out);
  graph_.AddVertex(*vertex_buffer_);
}

void GraphCreator::Run() {
  std::string v_name(SmallLog("ENTER Vertex Name:", 2, true));


  if (!graph_.Run(v_name)) {
    SmallLog("ERROR Cant run vertex " + v_name, 2);
    return;
  }
}

void GraphCreator::Test() {
  int n, cnt, acc;
  std::vector <const Edge*> way;
  std::vector <std::string> path;

  n = SmallLog<int>("ENTER Len of chain:", 2, true);

  SmallLog("ENTER vertexes names", 2);
  std::cout << "\t\t-";

  for (int i = 0; i < n; ++i) {
    std::string s;
    std::cin >> s;
    path.push_back(s);
  }
  cnt = SmallLog<int>("ENTER repeat number:", 2, true);
  acc = SmallLog<float>("ENTER acceleration:", 2, true);

  if (!graph_.RunChain(path, cnt, acc)) {
    SmallLog("ERROR Cant run chain", 2);
    return;
  }
}

void GraphCreator::RKick() {
  graph_.RightKick();
}

void GraphCreator::LKick() {

  graph_.LeftKick();
}
void GraphCreator::Move() {
  float x, y, theta;

  SmallLog("Its a Move section insert x, y and Theta:", 2);
  x     = SmallLog<float>("ENTER x in meters:", 2, true);
  y     = SmallLog<float>("ENTER y in meters:", 2, true);
  theta = SmallLog<float>("ENTER Theta in Degree:", 2, true);
  graph_.Move(x, y, theta * TO_RAD);
}


/*------- PRIVAT SPACE ---------*/


bool GraphCreator::CheckBuffer() const {
  if (!IsBufferEmpty()) {
    return false;
  }
  SmallLog("ERROR buffer is not defined!", 2);
  return true;
}

bool GraphCreator::IsBufferEmpty() const {
  return vertex_buffer_ == nullptr;
}

void GraphCreator::SetBuffer(const Vertex& v) {
  vertex_buffer_ = new Vertex(v);
}

void GraphCreator::ClearBuffer() {
  delete vertex_buffer_;
}

template <typename T>
T GraphCreator::SmallLog(const std::string text, size_t deep_level, bool is_reply) const {
  std::string request = "";
  for (size_t i = 0; i < deep_level; ++i) {
    request += "--";
  }

  std::cout << request + "> " + text << std::endl;
  if (is_reply) {
    T ans;
    std::cout << request + "| ";
    std::cin >> ans;
    return ans;
  }
  return T();
}