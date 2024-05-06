#ifndef PRE_MGR_H
#define PRE_MGR_H

#include "DB.h"
#include "Shape.h"

enum PreGridType {
    EMPTY,
    OBSTACLE,
    FREGION,
    CONTOUR
};

struct PreGrid {
    PreGridType type;
    string name;
    double pointX;
    double pointY;
    int fRegionId;
};

struct BoundBox {
    double minX;
    double minY;
    double maxX;
    double maxY;
};

class PreMgr {
    public:
        PreMgr(DB& db, SVGPlot& plot) : _db(db), _plot(plot) {
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                _vNumTPorts.push_back(_db.vNet(netId)->numTPorts());
            }
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

        // functions for obstacle clustering
        void initPreGrid(double gridWidth);
        void plotPreGrid();
        void fillLineGrid(size_t layId, double x1, double y1, double x2, double y2, string name);
        void fillLineGridXArch(size_t layId, double x1, double y1, double x2, double y2, string name);
        void fillPolygonGrid(size_t layId, Polygon* polygon, string name);

        // functions for port clustering
        void nodeClustering();
        void plotBoundBox();
        void assignPortPolygon();

        // functions for feasible region construction
        void clearPortGrid();
        void spareRailSpace();
        FRegion* constructFRegion(size_t netId, size_t layId);
        
    private:
        void kMeansClustering(size_t netId, vector<DBNode*> vNode, int numEpochs, int k);
        Polygon* convexHull(vector<DBNode*> vNode);
        DB& _db;
        SVGPlot& _plot;
        vector< size_t > _vNumTPorts;       // index = [netId]
        vector< BoundBox > _vSBoundBox;    //  bounding box of the source nodes (port), index = [netId]
        vector< vector< BoundBox > > _vTBoundBox;   // bounding box of the target nodes (ports), index = [netId] [tPortId]
        vector< vector< vector< DBNode* > > > _vTClusteredNode; // index = [netId] [tPortId] [nodeId]
        vector< vector< vector< PreGrid > > > _vPreGrid; // index = [layId] [rowId] [colId]
        size_t _numRows;
        size_t _numCols;
        double _gridWidth;
};

#endif