#ifndef LAYER_H
#define LAYER_H

#include "Include.h"
#include "Obstacle.h"
using namespace std;

class Layer {
    public:
        Layer(string name, size_t layId, double thickness, double permittivity) : _layName(name), _layId(layId), _thickness(thickness), _permittivity(permittivity) {}
        ~Layer() {}
        virtual double thickness()    const { return _thickness; }
        virtual double permittivity() const { return _permittivity; }
        virtual size_t layId()        const { return _layId; }
        virtual string layName()      const { return _layName; }
        virtual void setLayId(size_t layId) { _layId = layId; }
        
    protected:
        double _thickness;
        double _permittivity;
        size_t _layId;
        string _layName;
};

class MediumLayer : public Layer {
    public:
        // MediumLayer(string name, size_t layId, double thickness, double permittivity, double lossTangent)
        // : _layName(name), _layId(layId), _thickness(thickness), _permittivity(permittivity), _lossTangent(lossTangent) {}
        MediumLayer(string name, size_t layId, double thickness, double permittivity, double lossTangent)
        : Layer(name, layId, thickness, permittivity), _lossTangent(lossTangent) {}
        ~MediumLayer() {}
        void print() {
            cerr << "MediumLayer {layId=" << _layId << ", layName=" << _layName << ", thickness=" << _thickness 
                 << ", permittivity=" << _permittivity << ", lossTangent=" << _lossTangent << "}" << endl;
        }
    private:
        // double _thickness;
        // double _permittivity;
        // size_t _layId;
        // string _layName;
        double _lossTangent;
};

class MetalLayer : public Layer {
    public:
        // MetalLayer(string name, size_t layId, double thickness, double conductivity, double permittivity)
        // : _layName(name), _layId(layId), _thickness(thickness), _conductivity(conductivity), _permittivity(permittivity) {}
        MetalLayer(string name, size_t layId, double thickness, double conductivity, double permittivity)
        : Layer(name, layId, thickness, permittivity), _conductivity(conductivity) {}
        ~MetalLayer() {}

        size_t numObstacles() const       { return _vObstacle.size(); }
        Obstacle* vObstacle(size_t obsId) { return _vObstacle[obsId]; }
        double conductivity() const {return _conductivity; }

        void addObstacle(Obstacle* obs) { _vObstacle.push_back(obs); }
        void print() {
            cerr << "MetalLayer {layId=" << _layId << ", layName=" << _layName << ", thickness=" << _thickness 
                 << ", conductivity=" << _conductivity << ", permittivity=" << _permittivity << ", vObstacle=" << endl;
            for (size_t obsId=0; obsId<_vObstacle.size(); ++ obsId) {
                _vObstacle[obsId]->print();
            }
            cerr << "}" << endl;
        }
    private:
        // double            _thickness;
        // double            _permittivity;
        // size_t            _layId;
        // string            _layName;
        double            _conductivity;
        vector<Obstacle*> _vObstacle;
};

#endif