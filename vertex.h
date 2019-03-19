#include "moveparams.h"

class Edge;

class Vertex {
 public:
  Vertex() = delete;
  Vertex(std::vector <std::pair<float, float> > new_param_values_, bool is_radian = false);
  Vertex(std::vector <float> new_angel_values_, std::vector <float> new_hardness_values_, bool is_radian = false);
  Vertex(const Vertex& vertex);

  Vertex& operator =(const Vertex& vertex);

  float Dist(const Vertex& vertex) const;

  void SetName(const std::string& name);

  std::string GetName() const;

  int GetAdjacentCount() const;

  std::vector<float> GetRadianValues() const;

  std::vector<float> GetDegreesValues() const;

  std::vector<float> GetHardnessValues() const;

  void AddEdge(const Edge* new_edge);

  void PrintState(std::ostream &out) const;

  const Edge* GetEdge(int ind) const;

  void Reflect();

  void CopyFromSide(JOINT_TYPE side_name);

 private:
  std::string name_;
  std::vector <float> degree_values_;
  std::vector <float> radian_values_;
  std::vector <float> hardness_values_;
  std::vector <const Edge*> adjacent_edges_;
};