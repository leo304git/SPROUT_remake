#ifndef PRE_MGR_H
#define PRE_MGR_H

#include "DB.h"
#include "Shape.h"

struct BoundBox {
    double minX;
    double minY;
    double maxX;
    double maxY;
};

class PreMgr {
    public:
        PreMgr(DB& db, SVGPlot& plot) : _db(db), _plot(plot) {
            _vNumTPorts = {1,1,2};
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector<BoundBox> temp;
                _vTBoundBox.push_back(temp);
                vector< vector< DBNode* > > netNode;
                for (size_t tPortId = 0; tPortId < _vNumTPorts[netId]; ++tPortId) {
                    vector<DBNode*> tPortNode;
                    netNode.push_back(tPortNode);
                }
                _vTClusteredNode.push_back(netNode);
            }
        }
        ~PreMgr() {}

        void nodeClustering();
        void plotBoundBox();
        void assignPortPolygon();
    private:
        void kMeansClustering(size_t netId, vector<DBNode*> vNode, int numEpochs, int k);
        Polygon* convexHull(vector<DBNode*> vNode);
        DB& _db;
        SVGPlot& _plot;
        vector< size_t > _vNumTPorts;       // index = [netId]
        vector< BoundBox > _vSBoundBox;    //  bounding box of the source nodes (port), index = [netId]
        vector< vector< BoundBox > > _vTBoundBox;   // bounding box of the target nodes (ports), index = [netId] [tPortId]
        vector< vector< vector< DBNode* > > > _vTClusteredNode; // index = [netId] [tPortId] [nodeId]
};

#endif