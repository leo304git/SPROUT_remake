#ifndef DETAILED_DB_H
#define DETAILED_DB_H

#include "../base/Include.h"
using namespace std;

class Grid {
    public:
        Grid(size_t xId, size_t yId) : _xId(xId), _yId(yId) {
            _occupied = false;
            _netId = -1;
            // _congestion = 0;
            // _congestCur = 0;
            // _congestHis = 0;
        }
        ~Grid() {}

        size_t xId() const { return _xId; }
        size_t yId() const { return _yId; }
        bool occupied() const { return _occupied; }
        Grid* vNeighbor(size_t nbrId) { return _vNeighbor[nbrId]; }
        size_t numNeighbors() const { return _vNeighbor.size(); }
        int netId() const { return _netId; }
        double voltage() const { return _voltage; }
        double current() const { return _current; }

        void setOccupied(bool occupied) { _occupied = occupied; }
        void addNeighbor(Grid* grid) { _vNeighbor.push_back(grid); }
        void setNetId(int netId) { _netId = netId; }
        void setVoltage(double voltage) { _voltage = voltage; }
        void setCurrent(double current) { _current = current; }
        // int congestion() const { return _congestion; }
        // int congestCur() const { return _congestCur; }
        // int congestHis() const { return _congestHis; }
        // size_t vNetId(size_t i) const { return _vNetId[i]; }
        // size_t numNets() const { return _vNetId.size(); }
        // void incCongestCur() { _congestCur ++; _congestion ++; }
        // void decCongestCur() { _congestCur --; _congestion --; }
        // void addNet(size_t netId) { _vNetId.push_back(netId); }
        // void removeNet(size_t netId) {
        //     for (vector<size_t>::iterator i = _vNetId.begin(); i != _vNetId.end(); ++ i) {
        //         if (*i == netId) {
        //             _vNetId.erase(i);
        //             return;
        //         }
        //     }
        // }
        // bool hasNet(size_t netId) {
        //     for (size_t i = 0; i<_vNetId.size(); ++ i) {
        //         if (_vNetId[i] == netId) return true;
        //     }
        //     return false;
        // }
    private:
        // int _congestion;    // _congestHis + _congestCur
        // int _congestCur;    // current congestion cost
        // int _congestHis;    // history congestion cost
        // vector<size_t> _vNetId;
        size_t _xId;
        size_t _yId;
        bool _occupied;
        int _netId;
        vector<Grid*> _vNeighbor;
        double _voltage;
        double _current;
};

enum GNodeStatus {
    Init,
    InQueue,
    InPath
};

class GNode {
    public:
        GNode(int xId, int yId, Grid* grid) : _xId(xId), _yId(yId), _grid(grid) {
            _status = GNodeStatus::Init;
            _parent = NULL;
            _cost = numeric_limits<double>::infinity();
            _curDist = numeric_limits<double>::infinity();
            _estDist = numeric_limits<double>::infinity();
            _congest = numeric_limits<double>::infinity();
        }
        ~GNode() {}

        // get function
        int xId() const { return _xId; }
        int yId() const { return _yId; }
        GNodeStatus status() const { return _status; }
        GNode* parent() { return _parent; }
        double cost() const { return _cost; }
        double curDist() const { return _curDist; }
        double estDist() const { return _estDist; }
        double congest() const { return _congest; }
        Grid*  grid()   { return _grid; }

        // set function
        void setStatus(GNodeStatus status) { _status = status; }
        void setParent(GNode* parentNode) { _parent = parentNode; }
        void setCost(double cost) { _cost = cost; }
        void setCurDist(double curDist) { _curDist = curDist; }
        void setEstDist(double estDist) { _estDist = estDist; }
        void setCongest(double congest) { _congest = congest; }

    private:
        int _xId;
        int _yId;
        GNodeStatus _status;
        GNode* _parent;
        double _cost;
        double _curDist;    // the current distance cost (from the source)
        double _estDist;    // the estimated distance cost (to the target)
        double _congest;    // the (accumulated) congestion cost
        Grid* _grid;
};

#endif