#ifndef TNODE_H
#define TNODE_H

#include "../base/Include.h"
#include "TEdge.h"
#include "../base/Tile.h"
#include "../base/Via.h"

using namespace std;

enum TNodeType {
    SVia,
    TVia,
    F2D,
    Other
};

class TNode {
    // public:
    //     TNode(Tile* tile, ViaCluster* viaCluster): _tile(tile), _viaCluster(viaCluster) {
    //         _nodeType = TNodeType::Other;
    //     }
    //     ~TNode() {}

    //     Tile* tile() const { return _tile; }
    //     ViaCluster* viaCluster() const { return _viaCluster; }
    //     // TEdge* vTEdge(int edgeId) const { return _vTEdge[edgeId]; }
    //     // size_t numTEdges() const { return _vTEdge.size(); }

    //     TEdge* vOutEdge(int oEdgeId) const { return _vOutEdge[oEdgeId]; }
    //     TEdge* vInEdge(int iEdgeId) const { return _vInEdge[iEdgeId]; }
    //     size_t numOutEdges() const { return _vOutEdge.size(); }
    //     size_t numInEdges() const { return _vInEdge.size(); }
    //     TNodeType nodeType() const { return _nodeType;}
    //     unsigned int nodeTypeId() const { return _nodeTypeId; }
    //     unsigned int nodeId() const { return _nodeId; }

    //     // void addTEdge(TEdge* e) { _vTEdge.push_back(e); }
    //     void addOutEdge(TEdge* e) { _vOutEdge.push_back(e); }
    //     void addInEdge(TEdge* e) { _vInEdge.push_back(e); }
    //     void setNodeType(TNodeType t) { _nodeType = t; }
    //     void setNodeTypeId(unsigned int i) { _nodeTypeId = i; }
    //     void setNodeId(unsigned int nodeId) { _nodeId = nodeId; }

    // private:
    //     Tile* _tile;
    //     ViaCluster* _viaCluster;
    //     // vector<TEdge*> _vTEdge;  // the neighboring edges
    //     vector<TEdge*> _vOutEdge; // edges that start from this node
    //     vector<TEdge*> _vInEdge; // edges that go to this node
    //     TNodeType _nodeType;
    //     unsigned int _nodeTypeId;
    //     unsigned int _nodeId;
};

#endif