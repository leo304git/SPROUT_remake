#ifndef ASTAR_ROUTER_H
#define ASTAR_ROUTER_H

#include "../base/Include.h"
#include "DetailedDB.h"
using namespace std;

enum Direction {
    Up, Down, Right, Left,
    UpRight, UpLeft, DownRight, DownLeft
};

class AStarRouter {
    public:
        AStarRouter(vector< vector< Grid* > > vGrid, Grid* sGrid, Grid* tGrid, double gridWidth)
        : _vGrid(vGrid), _sGrid(sGrid), _tGrid(tGrid), _gridWidth(gridWidth) {
            for (size_t xId = 0; xId < numXId(); ++ xId) {
                vector<GNode*> temp;
                for (size_t yId = 0; yId < numYId(); ++ yId) {
                    GNode* node = new GNode(xId, yId, _vGrid[xId][yId]);
                    temp.push_back(node);
                }
                _vGNode.push_back(temp);
            }
            _sNode = new GNode(_sGrid->xId(), _sGrid->yId(), _sGrid);
            _tNode = new GNode(_tGrid->xId(), _tGrid->yId(), _tGrid);
        }
        ~AStarRouter() {}

        bool BFS();
        void backTrace();
        // bool route();
        // void backTrace(int tXId, int tYId);
        // double marginCongestCost(int xId, int yId, Direction dir);
        // double estDistCost(int xId, int yId) {
        //     // return abs(_tPos.first - xId) + abs(_tPos.second - yId);
        //     double longer = max(abs(_tPos.first - xId), abs(_tPos.second - yId));
        //     double shorter = min(abs(_tPos.first - xId), abs(_tPos.second - yId));
        //     return longer + shorter * (sqrt(2.0)-1.0);
        // }
        // double lineDistCost(int xId, int yId) {
        //     double den = abs(xId*(_tPos.second-_sPos.first) + _tPos.first*(_sPos.second-yId) + _sPos.first*(yId-_tPos.second));
        //     double num = sqrt(pow(_tPos.first-_sPos.first,2)+pow(_tPos.second-_sPos.second,2));
        //     return den / num;
        // }

        size_t numXId() const { return _vGrid.size(); }
        size_t numYId() const { return _vGrid[0].size(); }
        Grid* path(size_t pGridId) { return _path[pGridId]; }
        size_t numPath() const { return _path.size(); }
        // vector<Grid*> path() { return _path; }
        // Grid* vPGrid(size_t pGridId) { return _vPGrid[pGridId]; }
        // size_t numPGrids() const { return _vPGrid.size(); }
        // size_t exactWidth() const { return _exactWidth; }
        // size_t exactLength() const { return _exactLength; }

    private:
        bool legal(int xId, int yId) { return (xId>=0 && xId<numXId() && yId>=0 && yId<numYId()); }
        // double pathLength(int threshold);
        // input
        vector< vector< Grid* > > _vGrid;   // index = [xId] [yId]
        Grid* _sGrid;
        Grid* _tGrid;
        GNode* _sNode;
        GNode* _tNode;
        double _gridWidth;
        // output
        vector< Grid* > _path;  // the spine of the path
        vector< Grid* > _vPGrid;    // the grids in the path (considering width)
        // process
        vector< vector< GNode* > > _vGNode;     // index = [xId] [yId]
};

#endif