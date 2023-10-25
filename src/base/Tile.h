#ifndef TILE_H
#define TILE_H

#include "Include.h"

using namespace std;

class Tile{
    public:
        Tile(unsigned int layId, unsigned int rowId, unsigned int colId): _layId(layId), _rowId(rowId), _colId(colId) {
            _netId = 0;
            _hasNet = false;
            _hasObstacle = false;
            _hasVia = false;
        }
        ~Tile() {}

        // get functions
        unsigned int layId() const { return _layId; }
        unsigned int rowId() const { return _rowId; }
        unsigned int colId() const { return _colId; }
        unsigned int netId() const {
            if (!_hasNet) {
                cerr << "ERROR: Tile(" << _layId << "," << _rowId << "," << _colId << ") has no net!" << endl;
                return 0;
            } else {
                return _netId;
            }
        }
        unsigned int nodeId() const { return _nodeId; }
        bool hasNet() const { return _hasNet; }
        bool hasObstacle() const { return _hasObstacle; }
        bool hasVia() const { return _hasVia; }

        // set functions
        void setObstacle() {_hasObstacle = true; }
        void setNet(unsigned int netId) {
            // if (_hasObstacle) {
            //     cerr << "setNet fails! ERROR: Tile(" << _layId << "," << _rowId << "," << _colId << ") is full!" << endl;
            // }
            _netId = netId; 
            _hasNet = true;
        }
        void setNodeId(size_t nodeId) { _nodeId = nodeId; }
        void setVia() { _hasVia = true; }
    private:
        unsigned int _layId;
        unsigned int _rowId;
        unsigned int _colId;
        unsigned int _netId;    // the id of the net crossing the tile horizontally
        unsigned int _nodeId;
        unsigned int _viaId;
        bool _hasNet;   // true if has net crossing the tile horizontally
        bool _hasObstacle;
        bool _hasVia;
};

#endif