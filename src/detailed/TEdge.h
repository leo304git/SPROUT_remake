#ifndef TEDGE_H
#define TEDGE_H

#include "../base/Include.h"

using namespace std;

enum TEdgeType {
    Horizontal, // horizontal edge connecting non-source/target-via nodes
    Feasible2D, // vertical edge connecting 2D feasible nodes
    SourceVia,  // vertical edge connecting source-via nodes
    TargetVia,  // vertical edge connecting target-via nodes
    SRoundVia,  // horizontal edge connecting source-via nodes
    TRoundVia   // horizontal edge connecting target-via nodes
};

class TEdge {
    // public:
    //     TEdge(): _sNodeId(0), _tNodeId(0), _capacity(0), _cost(1), _hasNet(false) {}
    //     ~TEdge() {}

    //     unsigned int sNodeId() const { return _sNodeId; }
    //     unsigned int tNodeId() const { return _tNodeId; }
    //     unsigned int capacity() const { return _capacity; }
    //     unsigned int cost() const { return _cost; }
    //     TEdgeType edgeType() const { return _edgeType; }
    //     unsigned int edgeTypeId() const {return _edgeTypeId; }
    //     unsigned int netId() const { return _netId; }
    //     bool hasNet() const { return _hasNet; }

    //     void setSNodeId(unsigned int sNodeId) { _sNodeId = sNodeId; }
    //     void setTNodeId(unsigned int tNodeId) { _tNodeId = tNodeId; }
    //     void setEdgeType(TEdgeType type) { _edgeType = type; }
    //     void setEdgeTypeId(size_t edgeTypeId) { _edgeTypeId = edgeTypeId; }
    //     void setNetId(size_t netId) { _netId = netId; }
    //     void setNet() { _hasNet = true; }
    // private:
    //     unsigned int _sNodeId;   // source node Id, not nodeTypeId
    //     unsigned int _tNodeId;   // target node Id, not nodeTypeId
    //     unsigned int _capacity;
    //     unsigned int _cost;
    //     unsigned int _edgeTypeId;   // index in the edge vector of _type
    //     TEdgeType _edgeType;
    //     unsigned int _netId;    // initialized for Source/Target Via Edges, to be assigned for other edges
    //     bool _hasNet; // true if flowed (not approved) by net[_netId]
};

#endif