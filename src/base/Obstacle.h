#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "Include.h"
#include "Shape.h"
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

#endif