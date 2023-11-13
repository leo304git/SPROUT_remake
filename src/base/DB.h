#ifndef DB_H
#define DB_H

#include "Include.h"
#include "Net.h"
#include "Tile.h"
#include "Via.h"
#include "Layer.h"
#include "Obstacle.h"

class DB {
    public:
        // DB(size_t numNets, size_t numLayers, size_t numRows, size_t numCols): _numNets(numNets), _numLayers(numLayers), _numRows(numRows), _numCols(numCols) {
        //     for (size_t layId = 0; layId < _numLayers; ++layId) {
        //         vector< vector<Tile*> > tempTempTemp;
        //         for (size_t rowId = 0; rowId < _numRows; ++rowId) {
        //             vector<Tile*> tempTemp;
        //             for (size_t colId = 0; colId < _numCols; ++colId) {
        //                 Tile* tile = new Tile(layId, rowId, colId);
        //                 tempTemp.push_back(tile);
        //             }
        //             tempTempTemp.push_back(tempTemp);
        //         }
        //         _vTile.push_back(tempTempTemp);
        //     }

        //     for (size_t netId = 0; netId <_numNets; ++netId) {
        //         vector<ViaCluster*> tempTemp;
        //         _vViaCluster.push_back(tempTemp);
        //     }

        // }
        DB(SVGPlot& plot) : _plot(plot) {}
        ~DB() {}

        // void testInitialize();

        Tile*        vTile(int layId, int rowId, int colId)   { return _vTile[layId][rowId][colId]; }
        Via*         vVia(int viaId)                          { return _vVia[viaId]; }
        ViaCluster*  vViaCluster(size_t viaCstrId)            { return _vViaCluster[viaCstrId]; }
        // ViaCluster* vViaCluster(size_t netId, size_t netViaCstrId) { return _vNet[netId]->v; }
        MediumLayer* vMediumLayer(size_t mediumLayId)         { return _vMediumLayer[mediumLayId]; }
        MetalLayer*  vMetalLayer(size_t metalLayId)           { return _vMetalLayer[metalLayId]; }
        Net*         vNet(size_t netId)                       { return _vNet[netId]; }
        Obstacle*    vObstacle(size_t obsId)                  { return _vObstacle[obsId]; }
        Obstacle*    vObstacle(size_t layId, size_t layObsId) { return _vMetalLayer[layId]->vObstacle(layObsId); }
        // Node*        vNode(string nodeName)                   { return _vNode[_nodeName2Id[nodeName]]; }
        DBNode*      vDBNode(string nodeName)                 { return _vDBNode[_nodeName2Id[nodeName]]; }
        DBNode*      vSNode(size_t netId, size_t sNodeId)     { return vDBNode(_vSNode[netId][sNodeId]); }
        DBNode*      vTNode(size_t netId, size_t tNodeId)     { return vDBNode(_vTNode[netId][tNodeId]); }
        DBNode*      vClusteredNode(size_t netId, size_t portId, size_t nodeId) { return _vClusteredNode[netId][portId][nodeId]; }

        size_t numNets()                  const { return _vNet.size(); }
        size_t numLayers()                const { return _vMetalLayer.size(); } // number of metal layers
        size_t numMediumLayers()          const { return _vMediumLayer.size(); }
        // size_t numRows() const { return _numRows; }
        // size_t numCols() const { return _numCols; }
        size_t numVias()                  const { return _vVia.size(); }
        size_t numViaClusters()           const { return _vViaCluster.size(); }
        // size_t numViaClusters(size_t netId) const { return _vViaCluster[netId].size(); }
        size_t numObstacles()             const { return _vObstacle.size(); }
        size_t numObstacles(size_t layId) const { return _vMetalLayer[layId]->numObstacles(); }
        size_t numSNodes(size_t netId)    const { return _vSNode[netId].size(); }
        size_t numTNodes(size_t netId)    const { return _vTNode[netId].size(); }
        double boardWidth()               const { return _boardWidth; }
        double boardHeight()              const { return _boardHeight; }
        double areaWeight()               const { return _areaWeight; }
        double viaWeight()                const { return _viaWeight; }
        PadStack* VIA16D8A24()                  { return _VIA16D8A24; }
        size_t numClusteredNodes(size_t netId, size_t portId)  const { return _vClusteredNode[netId][portId].size(); }

        // size_t addVia(unsigned int rowId, unsigned int colId, unsigned int netId, ViaType type) {
        //     for (size_t layId = 0; layId < _numLayers; ++layId) {
        //         _vTile[layId][rowId][colId]->setVia();
        //     }
        //     Via* via = new Via(rowId, colId, netId, type);
        //     _vVia.push_back(via);
        //     return _vVia.size()-1;
        // }

        // void addNet(Net* net) { _vNet.push_back(net); }

        void initNet(size_t numNets) {
            for (size_t netId = 0; netId < numNets; ++ netId) {
                Net* net = new Net(numLayers());
                _vNet.push_back(net);
                vector<string> temp;
                _vSNode.push_back(temp);
                _vTNode.push_back(temp);
                vector< vector< DBNode* > > clusterTemp;
                _vClusteredNode.push_back(clusterTemp);
            }
        }

        void setBoundary(double boardWidth, double boardHeight) {
            _boardWidth = boardWidth;
            _boardHeight = boardHeight;
        }

        void addMediumLayer(string name, double thickness, double permittivity, double lossTangent) {
            MediumLayer* layer = new MediumLayer(name, _vMediumLayer.size(), thickness, permittivity, lossTangent);
            _vMediumLayer.push_back(layer);
        }

        void reverseMediumLayers() {
            vector<MediumLayer*> reverse;
            for (int mediumLayId = numMediumLayers()-1; mediumLayId >= 0; -- mediumLayId) {
                _vMediumLayer[mediumLayId]->setLayId(reverse.size());
                reverse.push_back(_vMediumLayer[mediumLayId]);
            }
            _vMediumLayer = reverse;
        }

        void addMetalLayer(string name, double thickness, double conductivity, double permittivity) {
            MetalLayer* layer = new MetalLayer(name, _vMetalLayer.size(), thickness, conductivity, permittivity);
            _vMetalLayer.push_back(layer);
        }

        void reverseMetalLayers() {
            vector<MetalLayer*> reverse;
            for (int metalLayId = numLayers()-1; metalLayId >= 0; -- metalLayId) {
                _vMetalLayer[metalLayId]->setLayId(reverse.size());
                reverse.push_back(_vMetalLayer[metalLayId]);
            }
            _vMetalLayer = reverse;
        }

        void addCircleVia(double x, double y, size_t netId, ViaType type) {
            Shape* circle = new Circle(x, y, 100*0.0254, _plot);
            Via* via = new Via(netId, type, circle);
            _vVia.push_back(via);
        }

        size_t addVia(double x, double y, size_t netId, ViaType type) {
            Via* via = new Via(x, y, _VIA16D8A24, netId, type, _plot);
            size_t viaId = _vVia.size();
            _vVia.push_back(via);
            return viaId;
        }

        ViaCluster* clusterVia(vector<size_t> vViaId) {
            ViaCluster* viaCluster = new ViaCluster;
            for (size_t i = 0; i < vViaId.size(); ++i) {
                viaCluster->addVia(_vVia[vViaId[i]]);
            }
            viaCluster->constructBBox(_plot);
            _vViaCluster.push_back(viaCluster);
            if (viaCluster->viaType() == ViaType::Added) {
                _vNet[viaCluster->netId()]->addAddedViaCstr(viaCluster);
            } 
            // else add viaCluster by port
            return viaCluster;
        }

        void addPort(double voltage, double current, ViaCluster* viaCstr) {
            Port* port = new Port(_vPort.size(), voltage, current, viaCstr);
            _vPort.push_back(port);
            if (viaCstr->viaType() == ViaType::Source) {
                _vNet[viaCstr->netId()] -> addSPort(port);
            } else if (viaCstr->viaType() == ViaType::Target) {
                _vNet[viaCstr->netId()] -> addTPort(port);
            } else {
                cerr << "ERROR: addPort FAILs! Wrong viaType!" << endl;
            }
        }

        void addSPort(size_t netId, double voltage, double current) {
            Port* port = new Port(_vPort.size(), -1, voltage, current);
            _vPort.push_back(port);
            _vNet[netId]->addSPort(port);
        }

        void addTPort(size_t netId, double voltage, double current) {
            Port* port = new Port(_vPort.size(), _vNet[netId]->numTPorts(), voltage, current);
            _vPort.push_back(port);
            _vNet[netId]->addTPort(port);
        }

        void addNode(string nodeName, double x, double y, size_t layId) {
            Node* node = new Node(x, y, _plot);
            // node->setLayId(layId);
            DBNode* dbNode = new DBNode(nodeName, node, layId);
            _nodeName2Id[nodeName] = _vDBNode.size();
            _vDBNode.push_back(dbNode);
        }

        void addViaEdge(string netName, string upNodeName, string lowNodeName, string padStackName) {
            ViaEdge* viaEdge = new ViaEdge(netName, upNodeName, lowNodeName, padStackName);
            _vViaEdge.push_back(viaEdge);
            _vDBNode[_nodeName2Id[upNodeName]]->setLowViaEdge(viaEdge);
            _vDBNode[_nodeName2Id[lowNodeName]]->setUpViaEdge(viaEdge);
        }

        void addSNode(size_t netId, string sNodeName) {
            _vSNode[netId].push_back(sNodeName);
        }

        void addTNode(size_t netId, string tNodeName) {
            _vTNode[netId].push_back(tNodeName);
        }

        void addObstacle (size_t layId, vector<Shape*> vShape) {
            Obstacle* obs = new Obstacle(vShape);
            _vObstacle.push_back(obs);
            _vMetalLayer[layId]->addObstacle(obs);
        }

        void addRectObstacle(size_t layId, double xLeft, double xRight, double yDown, double yUp) {
            assert((xLeft < xRight) && (yDown < yUp));
            vector< pair<double, double> > vVtx;
            vVtx.push_back(make_pair(xLeft, yDown));
            vVtx.push_back(make_pair(xRight, yDown));
            vVtx.push_back(make_pair(xRight, yUp));
            vVtx.push_back(make_pair(xLeft, yUp));
            Polygon* rect = new Polygon(vVtx, _plot);
            vector<Shape*> vShape;
            vShape.push_back(rect);
            addObstacle(layId, vShape);
        }

        void setFlowWeight(double areaWeight, double viaWeight) {
            _areaWeight = areaWeight;
            _viaWeight = viaWeight;
        }

        // void addObstacle(size_t layId, size_t rowId, size_t colId) {
        //     _vTile[layId][rowId][colId]->setObstacle();
        // }

        // void addSVGPlot(SVGPlot& plot) { _plot = SVGPlot&(plot); }

        void setVIA16D8A24() {
            vector<double> vRegular(numLayers(), 8*0.0254);
            vector<double> vAnti(numLayers(), 12*0.0254);
            _VIA16D8A24 = new PadStack("VIA16D8A24", "Circle", 4*0.0254, vRegular, vAnti);
        }

        void addVClusteredNode(size_t netId, vector< DBNode* > vNode) {
            _vClusteredNode[netId].push_back(vNode);
        }

        void print() {
            cerr << "DB {boardWidth=" << _boardWidth << ", boardHeight=" << _boardHeight << endl;
            cerr << "vObstacle=" << endl;
            for (size_t obsId = 0; obsId <  _vObstacle.size(); ++ obsId) {
                _vObstacle[obsId]->print();
            }
            cerr << "vMediumLayer=" << endl;
            for (size_t mediumLayId = 0; mediumLayId < _vMediumLayer.size(); ++ mediumLayId) {
                _vMediumLayer[mediumLayId]->print();
            }
            cerr << "vMetalLayer=" << endl;
            for (size_t metalLayId = 0; metalLayId < _vMetalLayer.size(); ++ metalLayId) {
                _vMetalLayer[metalLayId]->print();
            }
            cerr << "vVia=" << endl;
            for (size_t viaId = 0; viaId < _vVia.size(); ++ viaId) {
                _vVia[viaId]->print();
            }
            cerr << "vViaCluster=" << endl;
            for (size_t viaCstrId = 0; viaCstrId < _vViaCluster.size(); ++ viaCstrId) {
                _vViaCluster[viaCstrId]->print();
            }
            cerr << "vPort=" << endl;
            for (size_t portId = 0; portId < _vPort.size(); ++ portId) {
                _vPort[portId]->print();
            }
            cerr << "vNet=" << endl;
            for (size_t netId = 0; netId < _vNet.size(); ++ netId) {
                _vNet[netId]->print();
            }
            cerr << "}" << endl;

        }
        
    private:
        vector<Net*>         _vNet;
        vector<Via*>         _vVia;
        vector<ViaCluster*>  _vViaCluster;
        // vector< vector<ViaCluster*> > _vViaCluster;  // index = [netId] [viaClusterId]
        vector<MediumLayer*> _vMediumLayer;
        vector<MetalLayer*>  _vMetalLayer;
        vector<Obstacle*>    _vObstacle;
        // vector< vector<Obstacle*> > _vObstacle;     // index = [layId] [obsId]
        vector<Port*>        _vPort;
        // vector<Node*>        _vNode;
        vector<DBNode*>      _vDBNode;
        vector<ViaEdge*>     _vViaEdge;
        vector< vector< vector< Tile* > > > _vTile;     // index = [layId][rowId][colId], layId of the bottom layer is 0
        double               _boardWidth;
        double               _boardHeight;
        SVGPlot&             _plot;
        double _areaWeight;
        double _viaWeight;
        // size_t _numRows;
        // size_t _numCols;
        map<string, int>    _nodeName2Id;
        // map<string, int>    _layName2Id;
        vector< vector< string > > _vSNode; // index = [netId] [sNodeId]
        vector< vector< string > > _vTNode; // index = [netId] [tNodeId]
        PadStack* _VIA16D8A24;
        vector< vector< vector< DBNode* > > > _vClusteredNode; // index = [netId] [portId] [nodeId]

};

#endif