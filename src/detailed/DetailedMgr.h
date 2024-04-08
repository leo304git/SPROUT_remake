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
                    // size_t xId = _db.vNet(netId)->sourceViaCstr()->centerX() / _gridWidth;
                    // size_t yId = _db.vNet(netId)->sourceViaCstr()->centerY() / _gridWidth;
                    size_t xId = _db.vNet(netId)->sourcePort()->boundPolygon()->ctrX() / _gridWidth;
                    size_t yId = _db.vNet(netId)->sourcePort()->boundPolygon()->ctrY() / _gridWidth;
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
                        // size_t xId = _db.vNet(netId)->vTargetViaCstr(tPortId)->centerX() / _gridWidth;
                        // size_t yId = _db.vNet(netId)->vTargetViaCstr(tPortId)->centerY() / _gridWidth;
                        size_t xId = _db.vNet(netId)->targetPort(tPortId)->boundPolygon()->ctrX() / _gridWidth;
                        size_t yId = _db.vNet(netId)->targetPort(tPortId)->boundPolygon()->ctrY() / _gridWidth;
                        Grid* tGrid = new Grid(xId, yId);
                        tGrid->setOccupied(false);
                        vLayTGrid.push_back(tGrid);
                    }
                    vNetTGrid.push_back(vLayTGrid);
                }
                _vTGrid.push_back(vNetTGrid);
            }
            // _vTPortCurr.reserve(_db.numNets());
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                // _vTPortCurr[netId].reserve(_db.vNet(netId)->numTPorts());
                vector<double> temp(_db.vNet(netId)->numTPorts(), 0.0);
                _vTPortCurr.push_back(temp);
            }
            // _vTPortVolt.reserve(_db.numNets());
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                // _vTPortVolt[netId].reserve(_db.vNet(netId)->numTPorts());
                vector<double> temp(_db.vNet(netId)->numTPorts(), 0.0);
                _vTPortVolt.push_back(temp);
            }
            // _vNetPortGrid
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< vector< pair<int, int> > > temp;
                for (size_t portId = 0; portId <_db.vNet(netId)->numTPorts()+1; ++ portId) {
                    vector< pair<int, int> > tempPort;
                    temp.push_back(tempPort);
                }
                _vNetPortGrid.push_back(temp);
            }
            //vNetPolygon
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< vector< vector< pair<double,double> > > >  _Netpolygon_temp;
                for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                    vector< vector< pair<double,double> > >   _Netpolygon_Ltemp;
                    _Netpolygon_temp.push_back(_Netpolygon_Ltemp);
                }
                _Netpolygon.push_back(_Netpolygon_temp);
            }
            // _Ploted
            for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                vector< vector< bool* > > vNetPloted;
                for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                    vector<bool*> vLayPloted;
                    for(size_t portId = 0; portId < _db.vNet(netId)->numTPorts()+1; ++portId){
                        bool* ploted = new bool(false);
                        vLayPloted.push_back(ploted);
                    }
                    vNetPloted.push_back(vLayPloted);
                }
                _Ploted.push_back(vNetPloted);
            }
        }
        ~DetailedMgr() {}

        void initGridMap();
        void plotGraph();
        void plotGridMap();
        void plotGridMapVoltage();
        void plotGridMapCurrent();
        void naiveAStar();
        bool SingleNetAStar(size_t netId, size_t layId);
        void eigenTest();
        void plotDB();
        void synchronize();
        void addViaGrid();
        void addPortVia();
        void buildSingleNetMtx(size_t netId/* , size_t numLayers*/);
        void buildMtx(/*size_t numLayers*/);
        double getResistance(Grid*, Grid*);
        void check();
        void fillInnerCircle(size_t layId, size_t netId);
        void fillBoard(size_t layId, size_t netId, size_t xId, size_t yId);
        size_t SmartGrow(size_t layId, size_t netId, int k);
        void SmartRefine(size_t layId, size_t netId, int k);
        void SPROUT();
        int indexOfInnerCircles;
        bool thisSearchHaveFinished;
        vector< vector< int > > boardOfInnerCircles; 
        void writeColorMap_v2(const char* path, bool isVoltage);
        vector< pair<double, double> > kMeansClustering(vector< pair<int,int> > vGrid, int numClusters, int numEpochs);
        void RemoveIsolatedGrid();
        void findPointList();
        void OutputTest();
        void OutputFile(ifstream& fin, ofstream& fout);
        void printResult();

    private:
        void clearNet(size_t layId, size_t netId);
        void ResetAllNets();
        bool legal(int xId, int yId) { return (xId>=0 && xId<_vGrid[0].size() && yId>=0 && yId<_vGrid[0][0].size()); }
        DB& _db;
        SVGPlot& _plot;
        double _gridWidth;
        vector< vector< vector< Grid* > > > _vGrid;     // index = [layId] [xId] [yId]; from left xId = 0, from bottom yId = 0
        vector< vector< vector< Grid* > > > _vNetGrid;  // index = [netId] [layId] [gridId]
        vector< vector< Grid* > > _vSGrid;              // index = [netId] [layId]
        vector< vector< vector< Grid* > > > _vTGrid;    // index = [netId] [layId] [tPortId]
        vector< vector< vector< pair<int, int> > > > _vNetPortGrid;        // index = [netId] [portId] [gridId]
        size_t _numXs;
        size_t _numYs;
        vector< vector< double > > _vTPortVolt;     // index = [netId] [netTportId], record the target port voltage during simulation
        vector< vector< double > > _vTPortCurr;     // index = [netId] [netTportId], record the target port current during simulation
        vector< vector< vector< vector< pair<double,double> > > > > _Netpolygon; // index = [netId] [layId] [polygonId] [pointId]
        vector< vector <vector<bool*>>> _Ploted; // index = [netId][layId][PortId]
};

#endif