#ifndef DETAILED_MGR_H
#define DETAILED_MGR_H

#include "../base/Include.h"
#include "../base/DB.h"
#include "DetailedDB.h"
#include "AStarRouter.h"
using namespace std;

class DetailedMgr {
    public:
        DetailedMgr(DB& db, SVGPlot& plot, double gridWidth) : _db(db), _plot(plot), _gridWidth(gridWidth) {
            _numXs = _db.boardWidth() / _gridWidth;
            _numYs = _db.boardHeight() / _gridWidth;
            for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                vector< vector<Grid*> > vLayGrid;
                for (size_t xId = 0; xId < _numXs; ++ xId) {
                    vector<Grid*> vXGrid;
                    for (size_t yId = 0; yId < _numYs; ++ yId) {
                        Grid* grid = new Grid(xId, yId);
                        vXGrid.push_back(grid);
                    }
                    vLayGrid.push_back(vXGrid);
                }
                _vGrid.push_back(vLayGrid);
            }
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< vector< Grid* > > vNetGrid;
                for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                    vector<Grid*> vLayGrid;
                    vNetGrid.push_back(vLayGrid);
                }
                _vNetGrid.push_back(vNetGrid);
            }
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< Grid* > vNetSGrid;
                for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                    size_t xId = _db.vNet(netId)->sourceViaCstr()->centerX() / _gridWidth;
                    size_t yId = _db.vNet(netId)->sourceViaCstr()->centerY() / _gridWidth;
                    Grid* sGrid = new Grid(xId, yId);
                    sGrid->setOccupied(false);
                    vNetSGrid.push_back(sGrid);
                }
                _vSGrid.push_back(vNetSGrid);
            }
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< vector< Grid* > > vNetTGrid;
                for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                    vector< Grid* > vLayTGrid;
                    for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                        size_t xId = _db.vNet(netId)->vTargetViaCstr(tPortId)->centerX() / _gridWidth;
                        size_t yId = _db.vNet(netId)->vTargetViaCstr(tPortId)->centerY() / _gridWidth;
                        Grid* tGrid = new Grid(xId, yId);
                        tGrid->setOccupied(false);
                        vLayTGrid.push_back(tGrid);
                    }
                    vNetTGrid.push_back(vLayTGrid);
                }
                _vTGrid.push_back(vNetTGrid);
            }
        }
        ~DetailedMgr() {}

        void initGridMap();
        void plotGraph();
        void plotGridMap();
        void naiveAStar();
        void eigenTest();
        void fillInnerCircle(size_t layId, size_t netId);
        void fillBoard(size_t layId, size_t netId, size_t xId, size_t yId);
        int indexOfInnerCircles;
        vector< vector< Grid* > > boardOfInnerCircles; 

    private:
        void clearNet(size_t layId, size_t netId);
        bool legal(int xId, int yId) { return (xId>=0 && xId<_vGrid[0].size() && yId>=0 && yId<_vGrid[0][0].size()); }
        DB& _db;
        SVGPlot& _plot;
        double _gridWidth;
        vector< vector< vector< Grid* > > > _vGrid;     // index = [layId] [xId] [yId]; from left xId = 0, from bottom yId = 0
        vector< vector< vector< Grid* > > > _vNetGrid;  // index = [netId] [layId] [gridId]
        vector< vector< Grid* > > _vSGrid;              // index = [netId] [layId]
        vector< vector< vector< Grid* > > > _vTGrid;    // index = [netId] [layId] [tPortId]
        size_t _numXs;
        size_t _numYs;
};

#endif