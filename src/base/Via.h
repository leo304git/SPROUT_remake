#ifndef VIA_H
#define VIA_H

#include "Include.h"
#include "Shape.h"

using namespace std;

class ViaEdge {
    public:
        ViaEdge(string netName, string upNodeName, string lowNodeName, string padStackName)
        : _netName(netName), _upNodeName(upNodeName), _lowNodeName(lowNodeName), _padStackName(padStackName) {}
        ~ViaEdge() {}
    private:
        // string _name;
        string _netName;
        string _upNodeName;
        string _lowNodeName;
        string _padStackName;
};

class PadStack {
    public:
    private:
        string _name;
        string _shape;
        vector< double > _vRegular;   // if circle: radius; if square: width
        vector< double > _vAnti;      // if circle: radius; if square: width
};

enum ViaType {
    Source,
    Target,
    Added
};

class Via{
    public:
        // Via(unsigned int rowId, unsigned int colId, unsigned int netId, ViaType type, Shape* shape): _rowId(rowId), _colId(colId), _netId(netId), _viaType(type), _shape(shape) {}
        Via(unsigned int netId, ViaType type, Shape* shape): _netId(netId), _viaType(type), _shape(shape) {}
        ~Via() {}
        
        // unsigned int rowId() const { return _rowId; }
        // unsigned int colId() const { return _colId; }
        unsigned int netId() const {return _netId; }
        ViaType viaType() const { return _viaType; }
        Shape* shape() {return _shape;}
        void print() {
            cerr << "Via {netId=" << _netId << ", viaType=" << _viaType << endl;
            cerr << ", shape=";
            _shape->print();
            cerr << "}" << endl;
        }
    private:
        // unsigned int _rowId;
        // unsigned int _colId;
        unsigned int _netId;
        ViaType _viaType;
        Shape* _shape;
        string _padStackName;
        double _padDiameter;
        double _drillDiameter;
        double _antiPadDiameter;
};

class ViaCluster{
    public:
        ViaCluster() {}
        ~ViaCluster() {}

        Via*    vVia(int viaId) const { return _vVia[viaId]; }
        size_t  numVias()       const { return _vVia.size(); }
        size_t  netId()         const { return _vVia[0]->netId(); }
        ViaType viaType()       const { return _vVia[0]->viaType(); }
        // unsigned int nodeId() const { return _nodeId; }
        // double centerRowId() {
        //     double cRowId = 0;
        //     for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
        //         cRowId += _vVia[viaId]->rowId();
        //     }
        //     cRowId /= _vVia.size();
        //     return cRowId;
        // }

        // double centerColId() {
        //     double cColId = 0;
        //     for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
        //         cColId += _vVia[viaId]->colId();
        //     }
        //     cColId /= _vVia.size();
        //     return cColId;
        // }

        Shape* bBox() { return _bBox; }

        double centerX() {
            double x = 0;
            for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
                x += _vVia[viaId]->shape()->ctrX();
            }
            x /= _vVia.size();
            return x;
        }

        double centerY() {
            double y = 0;
            for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
                y += _vVia[viaId]->shape()->ctrY();
            }
            y /= _vVia.size();
            return y;
        }

        void addVia(Via* v) { _vVia.push_back(v); }
        // void setNodeId(unsigned int nodeId) { _nodeId = nodeId; }
        void constructBBox(SVGPlot& _plot) {
            double maxX = _vVia[0]->shape()->maxX();
            double minX = _vVia[0]->shape()->minX();
            double maxY = _vVia[0]->shape()->maxY();
            double minY = _vVia[0]->shape()->minY();
            for (size_t viaId = 0; viaId < _vVia.size(); ++viaId) {
                if (_vVia[viaId]->shape()->maxX() > maxX) {
                    maxX = _vVia[viaId]->shape()->maxX();
                }
                if (_vVia[viaId]->shape()->minX() < minX) {
                    minX = _vVia[viaId]->shape()->minX();
                }
                if (_vVia[viaId]->shape()->maxY() > maxY) {
                    maxY = _vVia[viaId]->shape()->maxY();
                }
                if (_vVia[viaId]->shape()->minY() < minY) {
                    minY = _vVia[viaId]->shape()->minY();
                }
            }
            vector<pair<double, double> > vVtx;
            vVtx.push_back(make_pair(minX, minY));
            vVtx.push_back(make_pair(maxX, minY));
            vVtx.push_back(make_pair(maxX, maxY));
            vVtx.push_back(make_pair(minX, maxY));
            _bBox = new Polygon(vVtx, _plot);
        }

        void print() {
            cerr << "ViaCluster {vVia=" << endl;
            for (size_t viaId = 0; viaId < _vVia.size(); ++ viaId) {
                _vVia[viaId]->print();
            }
            cerr << "}" << endl;
        }
    private:
        vector<Via*> _vVia;
        // unsigned int _nodeId;
        Shape* _bBox;
};

#endif