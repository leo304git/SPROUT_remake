#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Include.h"
#include "Shape.h"
#include "Net.h"
using  namespace std;

class Obstacle {
    public:
        Obstacle(vector<Shape*> vShape) : _vShape(vShape) {}
        ~Obstacle() {}
        size_t numShapes() const      { return _vShape.size(); }
        Shape* vShape(size_t shapeId) { return _vShape[shapeId]; }
        void print() {
            cerr << "Obstacle {vShape=" << endl;
            for (size_t shapeId = 0; shapeId < _vShape.size(); ++ shapeId) {
                _vShape[shapeId]->print();
            }
            cerr << "}" << endl;
        }
    private:
        vector<Shape*> _vShape;
};

class FRegion {
    public:
        FRegion() {};
        FRegion(Polygon* polygon) : _polygon(polygon) {};
        ~FRegion() {};

        void addObstacle(Obstacle* obstacle) {
            _vObstacle.push_back(obstacle);
        }
        void addPort(Port* port) {
            _vPort.push_back(port);
        }
        void setPolygon(Polygon* polygon) {
            _polygon = polygon;
        }

        size_t numObstacles() const { return _vObstacle.size(); }
        size_t numPorts() const { return _vPort.size(); }
        Obstacle* vObstacle(size_t obsId) { return _vObstacle[obsId]; }
        Port* vPort(size_t portId) { return _vPort[portId]; }
        Polygon* polygon() { return _polygon; }

        void print() {
            cerr << "FRegion {vObstacle=" << endl;
            for (size_t obsId = 0; obsId < _vObstacle.size(); ++ obsId) {
                _vObstacle[obsId]->print();
            }
            cerr << "vPort=" << endl;
            for (size_t portId = 0; portId < _vPort.size(); ++ portId) {
                _vPort[portId]->print();
            }
            cerr << "}" << endl;
        }

    private:
        vector<Port*> _vPort;
        vector<Obstacle*> _vObstacle;
        Polygon* _polygon;
};

#endif