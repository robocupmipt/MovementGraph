#include <queue>
#include <algorithm>
#include "edge.h"

class PrimalGraph {
 public:
  PrimalGraph();

  void Initialize();

  void AddVertex(const Vertex& v);

  void AddEdge(const std::string& from, const std::string& to, float time);

  const Vertex* GetVertex(const std::string& name) const;

  const Vertex* GetNearestVertex(const Vertex& v) const;

  bool FindWayToVertexFromVertex(const std::string& start,
                                 const std::string& finish,
                                 std::vector <const Edge*>& way) const;

  bool FindWayThroughVertexes(const std::vector <std::string>& names,
                              std::vector <const Edge*>& way) const;

  bool IsVertexContains(const std::string& name) const;

  void Debug() const;

 private:


  bool IsEdgeContains(const Edge& e) const;

  bool FindWayToVertexFromVertexViaBFS(int start,
                                       int finish,
                                       std::vector <int> & way) const;
 private:
  std::vector <Vertex> vertexes_;
  std::vector <Edge> edges_;
  std::map <std::string, int> vertexes_by_name_;
  mutable std::map <const Vertex*, int> vertex_to_index_;
  std::vector <std::vector <int>> adjacency_list_;
};