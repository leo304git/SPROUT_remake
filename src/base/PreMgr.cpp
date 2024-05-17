#include "PreMgr.h"

void PreMgr::initPreGrid(double gridWidth) {
    _gridWidth = gridWidth;
    _numRows = ceil(_db.boardHeight() / gridWidth);
    _numCols = ceil(_db.boardWidth() / gridWidth);
    PreGrid pGrid = {PreGridType::EMPTY, "empty", 0.0, 0.0, -1};
    vector< PreGrid > vColGrid(_numCols, pGrid);
    vector< vector< PreGrid > > vRowGrid(_numRows, vColGrid);
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        _vPreGrid.push_back(vRowGrid);
    }
}

void PreMgr::plotPreGrid() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t rowId = 0; rowId < _numRows; ++ rowId) {
            for (size_t colId = 0; colId < _numCols; ++ colId) {
                if (_vPreGrid[layId][rowId][colId].type == PreGridType::OBSTACLE) {
                    double x1 = colId * _gridWidth;
                    double y1 = rowId * _gridWidth;
                    double x2 = (colId + 1) * _gridWidth;
                    double y2 = (rowId + 1) * _gridWidth;
                    vector< pair<double, double> > vVtx;
                    vVtx.push_back(make_pair(x1, y1));
                    vVtx.push_back(make_pair(x2, y1));
                    vVtx.push_back(make_pair(x2, y2));
                    vVtx.push_back(make_pair(x1, y2));
                    Polygon* p = new Polygon(vVtx, _plot);
                    p->plot(SVGPlotColor::gray, layId);
                }
                else if (_vPreGrid[layId][rowId][colId].type == PreGridType::CONTOUR) {
                    double x1 = colId * _gridWidth;
                    double y1 = rowId * _gridWidth;
                    double x2 = (colId + 1) * _gridWidth;
                    double y2 = (rowId + 1) * _gridWidth;
                    vector< pair<double, double> > vVtx;
                    vVtx.push_back(make_pair(x1, y1));
                    vVtx.push_back(make_pair(x2, y1));
                    vVtx.push_back(make_pair(x2, y2));
                    vVtx.push_back(make_pair(x1, y2));
                    Polygon* p = new Polygon(vVtx, _plot);
                    p->plot(SVGPlotColor::red, layId);
                }
                else if (_vPreGrid[layId][rowId][colId].type == PreGridType::FREGION) {
                    double x1 = colId * _gridWidth;
                    double y1 = rowId * _gridWidth;
                    double x2 = (colId + 1) * _gridWidth;
                    double y2 = (rowId + 1) * _gridWidth;
                    vector< pair<double, double> > vVtx;
                    vVtx.push_back(make_pair(x1, y1));
                    vVtx.push_back(make_pair(x2, y1));
                    vVtx.push_back(make_pair(x2, y2));
                    vVtx.push_back(make_pair(x1, y2));
                    Polygon* p = new Polygon(vVtx, _plot);
                    p->plot(SVGPlotColor::gold, layId);
                }
            }
        }
    }

}

void PreMgr::fillLineGrid(size_t layId, double x1, double y1, double x2, double y2, string name) {
    int x1Idx = floor(x1 / _gridWidth);
    int y1Idx = floor(y1 / _gridWidth);
    int x2Idx = floor(x2 / _gridWidth);
    int y2Idx = floor(y2 / _gridWidth);
    if (x1Idx == _numCols) x1Idx--;
    if (y1Idx == _numRows) y1Idx--;
    if (x2Idx == _numCols) x2Idx--;
    if (y2Idx == _numRows) y2Idx--;
    assert(x1Idx >= 0 && x1Idx < _numCols);
    assert(y1Idx >= 0 && y1Idx < _numRows);
    assert(x2Idx >= 0 && x2Idx < _numCols);
    assert(y2Idx >= 0 && y2Idx < _numRows);
    int xIdx = x1Idx;
    int yIdx = y1Idx;
    int dx = abs(x2Idx - x1Idx);
    int dy = abs(y2Idx - y1Idx);
    int sx = (x1Idx <= x2Idx) ? ((x1Idx == x2Idx) ? 0 : 1) : -1;
    int sy = (y1Idx <= y2Idx) ? ((y1Idx == y2Idx) ? 0 : 1) : -1;
    if (x1Idx < x2Idx) {
        sx = 1;
    } else if (x1Idx > x2Idx) {
        sx = -1;
    } else {
        sx = 0;
    }
    if (y1Idx < y2Idx) {
        sy = 1;
    } else if (y1Idx > y2Idx) {
        sy = -1;
    } else {
        sy = 0;
    }
    bool interchange = false;
    if (dy > dx) {
        int temp = dx;
        dx = dy;
        dy = temp;
        interchange = true;
    }

    int D = 2 * dy - dx;
    int DE = 2 * dy;
    int DNE = 2 * (dy - dx);

    // if (sx <= 0 && sy <= 0)
    _vPreGrid[layId][yIdx][xIdx].type = PreGridType::OBSTACLE;

    while (xIdx < x2Idx || yIdx < y2Idx) {
        if (D <= 0) {
            if (interchange) {
                assert(sy != 0);
                yIdx += sy;
            } else {
                assert(sx != 0);
                xIdx += sx;
            }
            D += DE;
        } else {
            assert(sx != 0);
            assert(sy != 0);
            yIdx += sy;
            xIdx += sx;
            D += DNE;
        }
        // if (sx <= 0 && sy <= 0)
        _vPreGrid[layId][yIdx][xIdx].type = PreGridType::OBSTACLE;
    }
        
}

void PreMgr::fillLineGridXArch(size_t layId, double x1, double y1, double x2, double y2, string name) {
    int x1Idx = floor(x1 / _gridWidth);
    int y1Idx = floor(y1 / _gridWidth);
    int x2Idx = floor(x2 / _gridWidth);
    int y2Idx = floor(y2 / _gridWidth);
    if (x1Idx == _numCols) x1Idx--;
    if (y1Idx == _numRows) y1Idx--;
    if (x2Idx == _numCols) x2Idx--;
    if (y2Idx == _numRows) y2Idx--;
    assert(x1Idx >= 0 && x1Idx < _numCols);
    assert(y1Idx >= 0 && y1Idx < _numRows);
    assert(x2Idx >= 0 && x2Idx < _numCols);
    assert(y2Idx >= 0 && y2Idx < _numRows);
    if (x1Idx == x2Idx) {
        if (y1Idx < y2Idx) {
            for (int yIdx = y1Idx; yIdx <= y2Idx; ++ yIdx) {
                _vPreGrid[layId][yIdx][x1Idx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][yIdx][x1Idx].name = name;
            }
        } else if (y1Idx > y2Idx) {
            for (int yIdx = y2Idx; yIdx <= y1Idx; ++ yIdx) {
                _vPreGrid[layId][yIdx][x1Idx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][yIdx][x1Idx].name = name;
            }
        } else {
            _vPreGrid[layId][y1Idx][x1Idx].type = PreGridType::OBSTACLE;
            _vPreGrid[layId][y1Idx][x1Idx].name = name;
        }
    } else if (x1Idx < x2Idx) {
        if (y1Idx < y2Idx) {
            int yIdx = y1Idx;
            for (int xIdx = x1Idx; xIdx <= x2Idx && yIdx <= y2Idx; ++ xIdx) {
                _vPreGrid[layId][yIdx][xIdx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][yIdx][xIdx].name = name;
                yIdx += 1;
            }
        } else if (y1Idx > y2Idx) {
            int yIdx = y1Idx;
            for (int xIdx = x1Idx; xIdx <= x2Idx && yIdx >= y2Idx; ++ xIdx) {
                _vPreGrid[layId][yIdx][xIdx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][yIdx][xIdx].name = name;
                yIdx -= 1;
            }
        } else {
            for (int xIdx = x1Idx; xIdx <= x2Idx; ++ xIdx) {
                _vPreGrid[layId][y1Idx][xIdx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][y1Idx][xIdx].name = name;
            }
        }
    } else {
        if (y1Idx < y2Idx) {
            int yIdx = y2Idx;
            for (int xIdx = x2Idx; xIdx <= x1Idx && yIdx >= y1Idx; ++ xIdx) {
                _vPreGrid[layId][yIdx][xIdx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][yIdx][xIdx].name = name;
                yIdx -= 1;
            }
        } else if (y1Idx > y2Idx) {
            int yIdx = y2Idx;
            for (int xIdx = x2Idx; xIdx <= x1Idx && yIdx <= y1Idx; ++ xIdx) {
                _vPreGrid[layId][yIdx][xIdx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][yIdx][xIdx].name = name;
                yIdx += 1;
            }
        } else {
            for (int xIdx = x2Idx; xIdx <= x1Idx; ++ xIdx) {
                _vPreGrid[layId][y1Idx][xIdx].type = PreGridType::OBSTACLE;
                _vPreGrid[layId][y1Idx][xIdx].name = name;
            }
        }
    }
}

void PreMgr::fillPolygonGrid(size_t layId, Polygon* polygon, string name) {
    auto occupy = [&] (int xId, int yId, Shape* shape) -> bool {
        if (shape->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
        if (shape->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
        if (shape->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
        if (shape->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        for (size_t vtxIdx = 0; vtxIdx < shape->numBPolyVtcs(); ++ vtxIdx) {
            if (shape->bPolygonX(vtxIdx) >= xId*_gridWidth &&
                shape->bPolygonX(vtxIdx) <= (xId+1)*_gridWidth &&
                shape->bPolygonY(vtxIdx) >= yId*_gridWidth &&
                shape->bPolygonY(vtxIdx) <= (yId+1)*_gridWidth) {
                    return true;
                }
        }
        return false;
    };

    int xMinIdx = floor(polygon->minX() / _gridWidth);
    int xMaxIdx = floor(polygon->maxX() / _gridWidth);
    int yMinIdx = floor(polygon->minY() / _gridWidth);
    int yMaxIdx = floor(polygon->maxY() / _gridWidth);
    if (xMaxIdx == _numCols) xMaxIdx --;
    if (yMaxIdx == _numRows) yMaxIdx --;
    assert(xMinIdx >= 0 && xMinIdx < _numCols);
    assert(yMinIdx >= 0 && yMinIdx < _numRows);
    assert(xMaxIdx >= 0 && xMaxIdx < _numCols);
    assert(yMaxIdx >= 0 && yMaxIdx < _numRows);

    for(int xIdx = xMinIdx; xIdx <= xMaxIdx; ++ xIdx) {
        for(int yIdx = yMinIdx; yIdx <= yMaxIdx; ++ yIdx) {
            if (occupy(xIdx, yIdx, polygon)) {
                _vPreGrid[layId][yIdx][xIdx].name = name;
                if (name.find("-") == string::npos) {
                    _vPreGrid[layId][yIdx][xIdx].type = PreGridType::OBSTACLE;
                } else {
                    _vPreGrid[layId][yIdx][xIdx].type = PreGridType::EMPTY;
                }
            }
        }
    }
}

void PreMgr::initialize() {
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        _vNumTPorts.push_back(_db.vNet(netId)->numTPorts());
    }
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        vector<BoundBox> temp;
        _vTBoundBox.push_back(temp);
        vector< vector< DBNode* > > netNode;
        for (size_t tPortId = 0; tPortId < _vNumTPorts[netId]; ++tPortId) {
            vector<DBNode*> tPortNode;
            netNode.push_back(tPortNode);
        }
        _vTClusteredNode.push_back(netNode);
    }
}

void PreMgr::nodeClustering() {
    for (size_t netId =0; netId < _db.numNets(); ++ netId) {
        BoundBox sb = {_db.vSNode(netId, 0)->node()->ctrX(), _db.vSNode(netId, 0)->node()->ctrY(),
                      _db.vSNode(netId, 0)->node()->ctrX(), _db.vSNode(netId, 0)->node()->ctrY()};
        for (size_t sNodeId = 0; sNodeId < _db.numSNodes(netId); ++ sNodeId) {
            if (_db.vSNode(netId, sNodeId)->node()->ctrX() < sb.minX) {
                sb.minX = _db.vSNode(netId, sNodeId)->node()->ctrX();
            }
            if (_db.vSNode(netId, sNodeId)->node()->ctrY() < sb.minY) {
                sb.minY = _db.vSNode(netId, sNodeId)->node()->ctrY();
            }
            if (_db.vSNode(netId, sNodeId)->node()->ctrX() > sb.maxX) {
                sb.maxX = _db.vSNode(netId, sNodeId)->node()->ctrX();
            }
            if (_db.vSNode(netId, sNodeId)->node()->ctrY() > sb.maxY) {
                sb.maxY = _db.vSNode(netId, sNodeId)->node()->ctrY();
            }
        }
        sb.minX -= 2;
        sb.minY -= 2;
        sb.maxX += 2;
        sb.maxY += 2;
        _vSBoundBox.push_back(sb);
    }

    for (size_t netId =0; netId < _db.numNets(); ++ netId) {
        if (_vNumTPorts[netId] > 1) {
            vector<DBNode*> vTNode;
            for (size_t tNodeId = 0; tNodeId < _db.numTNodes(netId); ++ tNodeId) {
                vTNode.push_back(_db.vTNode(netId, tNodeId));
            }
            // cerr << "kMeans..." << endl;
            kMeansClustering(netId, vTNode, 10, _vNumTPorts[netId]);
            for (size_t tPortId = 0; tPortId < _vNumTPorts[netId]; ++ tPortId) {
                assert(_vTClusteredNode[netId][tPortId].size() > 0);
            }
        }
        else {
            for (size_t tNodeId = 0; tNodeId < _db.numTNodes(netId); ++ tNodeId) {
                _vTClusteredNode[netId][0].push_back(_db.vTNode(netId, tNodeId));
            }
        }
    }

    for (size_t netId =0; netId < _db.numNets(); ++ netId) {
        for (size_t tPortId = 0; tPortId < _vNumTPorts[netId]; ++ tPortId) {
            BoundBox tb = {_vTClusteredNode[netId][tPortId][0]->node()->ctrX(), _vTClusteredNode[netId][tPortId][0]->node()->ctrY(),
                           _vTClusteredNode[netId][tPortId][0]->node()->ctrX(), _vTClusteredNode[netId][tPortId][0]->node()->ctrY()};
            for (size_t tNodeId = 0; tNodeId < _vTClusteredNode[netId][tPortId].size(); ++ tNodeId) {
                if (_vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrX() < tb.minX) {
                    tb.minX = _vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrX();
                }
                if (_vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrY() < tb.minY) {
                    tb.minY = _vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrY();
                }
                if (_vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrX() > tb.maxX) {
                    tb.maxX = _vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrX();
                }
                if (_vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrY() > tb.maxY) {
                    tb.maxY = _vTClusteredNode[netId][tPortId][tNodeId]->node()->ctrY();
                }
            }
            tb.minX -= 2;
            tb.minY -= 2;
            tb.maxX += 2;
            tb.maxY += 2;
            _vTBoundBox[netId].push_back(tb);
        }
    }

    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        vector<DBNode*> vSNode;
        for (size_t sNodeId = 0; sNodeId < _db.numSNodes(netId); ++ sNodeId) {
            vSNode.push_back(_db.vSNode(netId, sNodeId));
        }
        _db.addVClusteredNode(netId, vSNode);
        for (size_t tPortId = 0; tPortId < _vNumTPorts[netId]; ++ tPortId) {
            _db.addVClusteredNode(netId, _vTClusteredNode[netId][tPortId]);
        }
    }
}

void PreMgr::plotBoundBox() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t netId =0; netId < _db.numNets(); ++ netId) {
            // vector< pair<double, double> > vVtx;
            // vVtx.push_back(make_pair(_vSBoundBox[netId].minX, _vSBoundBox[netId].minY));
            // vVtx.push_back(make_pair(_vSBoundBox[netId].maxX, _vSBoundBox[netId].minY));
            // vVtx.push_back(make_pair(_vSBoundBox[netId].maxX, _vSBoundBox[netId].maxY));
            // vVtx.push_back(make_pair(_vSBoundBox[netId].minX, _vSBoundBox[netId].maxY));
            // Polygon* p = new Polygon(vVtx, _plot);
            // p->plot(netId, 0);
            _db.vNet(netId)->sourcePort()->boundPolygon()->plot(netId, layId);
            for (size_t tPortId = 0; tPortId < _vNumTPorts[netId]; ++ tPortId) {
                // vector< pair<double, double> > vVtx;
                // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].minX, _vTBoundBox[netId][tPortId].minY));
                // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].maxX, _vTBoundBox[netId][tPortId].minY));
                // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].maxX, _vTBoundBox[netId][tPortId].maxY));
                // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].minX, _vTBoundBox[netId][tPortId].maxY));
                // Polygon* p = new Polygon(vVtx, _plot);
                // p->plot(netId, 0);
                _db.vNet(netId)->targetPort(tPortId)->boundPolygon()->plot(netId, layId);
            }
        }
    }
    
}

void PreMgr::assignPortPolygon() {
    for (size_t netId =0; netId < _db.numNets(); ++ netId) {
        vector< pair<double, double> > vVtx;
        vVtx.push_back(make_pair(_vSBoundBox[netId].minX, _vSBoundBox[netId].minY));
        vVtx.push_back(make_pair(_vSBoundBox[netId].maxX, _vSBoundBox[netId].minY));
        vVtx.push_back(make_pair(_vSBoundBox[netId].maxX, _vSBoundBox[netId].maxY));
        vVtx.push_back(make_pair(_vSBoundBox[netId].minX, _vSBoundBox[netId].maxY));
        Polygon* p = new Polygon(vVtx, _plot);
        _db.vNet(netId)->sourcePort()->setBoundPolygon(p);
        for (size_t tPortId = 0; tPortId < _vNumTPorts[netId]; ++ tPortId) {
            // vector< pair<double, double> > vVtx;
            // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].minX, _vTBoundBox[netId][tPortId].minY));
            // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].maxX, _vTBoundBox[netId][tPortId].minY));
            // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].maxX, _vTBoundBox[netId][tPortId].maxY));
            // vVtx.push_back(make_pair(_vTBoundBox[netId][tPortId].minX, _vTBoundBox[netId][tPortId].maxY));
            // Polygon* p = new Polygon(vVtx, _plot);
            // _db.vNet(netId)->targetPort(tPortId)->setBoundPolygon(p);
            Polygon* p = convexHull(_vTClusteredNode[netId][tPortId]);
            _db.vNet(netId)->targetPort(tPortId)->setBoundPolygon(p);
        }
    }
}

void PreMgr::clearPortGrid() {
    auto occupy = [&] (int xId, int yId, Shape* shape) -> bool {
        if (shape->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
        if (shape->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
        if (shape->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
        if (shape->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        for (size_t vtxIdx = 0; vtxIdx < shape->numBPolyVtcs(); ++ vtxIdx) {
            if (shape->bPolygonX(vtxIdx) >= xId*_gridWidth &&
                shape->bPolygonX(vtxIdx) <= (xId+1)*_gridWidth &&
                shape->bPolygonY(vtxIdx) >= yId*_gridWidth &&
                shape->bPolygonY(vtxIdx) <= (yId+1)*_gridWidth) {
                    return true;
                }
        }
        return false;
    };
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            for (size_t portId = 0; portId < _db.vNet(netId)->numTPorts()+1; ++ portId) {
                Polygon* polygon;
                if (portId == 0) {
                    // _db.vNet(netId)->sourcePort()->boundPolygon()->clearGrid(layId);
                    polygon = _db.vNet(netId)->sourcePort()->boundPolygon();
                } else {
                    // _db.vNet(netId)->targetPort(portId-1)->boundPolygon()->clearGrid(layId);
                    polygon = _db.vNet(netId)->targetPort(portId-1)->boundPolygon();
                }
                int xMinIdx = floor(polygon->minX() / _gridWidth);
                int xMaxIdx = floor(polygon->maxX() / _gridWidth);
                int yMinIdx = floor(polygon->minY() / _gridWidth);
                int yMaxIdx = floor(polygon->maxY() / _gridWidth);
                if (xMaxIdx == _numCols) xMaxIdx --;
                if (yMaxIdx == _numRows) yMaxIdx --;
                assert(xMinIdx >= 0 && xMinIdx < _numCols);
                assert(yMinIdx >= 0 && yMinIdx < _numRows);
                assert(xMaxIdx >= 0 && xMaxIdx < _numCols);
                assert(yMaxIdx >= 0 && yMaxIdx < _numRows);

                for(int xIdx = xMinIdx; xIdx <= xMaxIdx; ++ xIdx) {
                    for(int yIdx = yMinIdx; yIdx <= yMaxIdx; ++ yIdx) {
                        if (occupy(xIdx, yIdx, polygon)) {
                            _vPreGrid[layId][yIdx][xIdx].type = PreGridType::EMPTY;
                        }
                    }
                }
            }
        }
    }
}

void PreMgr::spareRailSpace() {
    auto legal = [&](int yIdx, int xIdx) -> bool {
        return xIdx >= 0 && xIdx < _numCols && yIdx >= 0 && yIdx < _numRows;
    };

    cerr << "spareRailSpace..." << endl;

    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        // size_t layId = 1;
        cerr << "layId = " << layId << endl;
        int fRegionId = -1;
        // FRegion* fRegion;
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            // size_t netId = 0;
            for (size_t portId = 0; portId < _db.vNet(netId)->numTPorts()+1 ; ++ portId) {
                // size_t portId = 0;
                int x0Idx;
                int y0Idx;
                if (portId == 0) {
                    x0Idx = floor(_db.vSNode(netId, 0)->node()->ctrX() / _gridWidth);
                    y0Idx = floor(_db.vSNode(netId, 0)->node()->ctrY() / _gridWidth);
                } else {
                    x0Idx = floor(_vTClusteredNode[netId][portId-1][0]->node()->ctrX() / _gridWidth); 
                    y0Idx = floor(_vTClusteredNode[netId][portId-1][0]->node()->ctrY() / _gridWidth);
                }
            
                // if (_vPreGrid[layId][y0Idx][x0Idx].type == PreGridType::OBSTACLE) {
                //     continue;
                // }
                assert(_vPreGrid[layId][y0Idx][x0Idx].type != PreGridType::OBSTACLE);
                if (_vPreGrid[layId][y0Idx][x0Idx].type == PreGridType::EMPTY) {
                    fRegionId++;
                    // BFS
                    queue< pair<int, int> > q;
                    _vPreGrid[layId][y0Idx][x0Idx].type = PreGridType::FREGION;
                    _vPreGrid[layId][y0Idx][x0Idx].fRegionId = fRegionId;
                    q.push(make_pair(y0Idx, x0Idx));
                    int sRowIdx = -1;
                    int sColIdx = -1;
                    bool sFound = false;
                    vector< pair<int, int> > vContour;

                    while (!q.empty()) {
                        pair<int, int> p = q.front();
                        q.pop();
                        int yIdx = p.first;
                        int xIdx = p.second;
                        // cerr << "yIdx = " << yIdx << ", xIdx = " << xIdx << endl;
                        assert(xIdx >= 0 && xIdx < _numCols && yIdx >= 0 && yIdx < _numRows);

                        // explore upward
                        if (legal(yIdx-1, xIdx) && _vPreGrid[layId][yIdx-1][xIdx].type == PreGridType::EMPTY) {
                            _vPreGrid[layId][yIdx-1][xIdx].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx-1][xIdx].fRegionId = fRegionId;
                            q.push(make_pair(yIdx-1, xIdx));
                        } else if (legal(yIdx-1, xIdx) && _vPreGrid[layId][yIdx-1][xIdx].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx-1][xIdx].type = PreGridType::CONTOUR;
                            _vPreGrid[layId][yIdx-1][xIdx].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx-1, xIdx));
                            if (!sFound) {
                                sRowIdx = yIdx-1;
                                sColIdx = xIdx;
                                sFound = true;
                            }
                        } else if (legal(yIdx-1, xIdx) && _vPreGrid[layId][yIdx-1][xIdx].type == PreGridType::CONTOUR) {
                            _vPreGrid[layId][yIdx-1][xIdx].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx-1, xIdx));
                            if (!sFound) {
                                sRowIdx = yIdx-1;
                                sColIdx = xIdx;
                                sFound = true;
                            }
                        } else if (!legal(yIdx-1, xIdx)) {
                            _vPreGrid[layId][yIdx][xIdx].type = PreGridType::CONTOUR;
                            vContour.push_back(make_pair(yIdx, xIdx));
                        }

                        // explore downward
                        if (legal(yIdx+1, xIdx) && _vPreGrid[layId][yIdx+1][xIdx].type == PreGridType::EMPTY) {
                            _vPreGrid[layId][yIdx+1][xIdx].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx+1][xIdx].fRegionId = fRegionId;
                            q.push(make_pair(yIdx+1, xIdx));
                        } else if (legal(yIdx+1, xIdx) && _vPreGrid[layId][yIdx+1][xIdx].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx+1][xIdx].type = PreGridType::CONTOUR;
                            _vPreGrid[layId][yIdx+1][xIdx].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx+1, xIdx));
                            if (!sFound) {
                                sRowIdx = yIdx+1;
                                sColIdx = xIdx;
                                sFound = true;
                            }
                        } else if (legal(yIdx+1, xIdx) && _vPreGrid[layId][yIdx+1][xIdx].type == PreGridType::CONTOUR) {
                            _vPreGrid[layId][yIdx+1][xIdx].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx+1, xIdx));
                            if (!sFound) {
                                sRowIdx = yIdx+1;
                                sColIdx = xIdx;
                                sFound = true;
                            }
                        } else if (!legal(yIdx+1, xIdx)) {
                            _vPreGrid[layId][yIdx][xIdx].type = PreGridType::CONTOUR;
                            vContour.push_back(make_pair(yIdx, xIdx));
                        }

                        // explore leftward
                        if (legal(yIdx, xIdx-1) && _vPreGrid[layId][yIdx][xIdx-1].type == PreGridType::EMPTY) {
                            _vPreGrid[layId][yIdx][xIdx-1].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx][xIdx-1].fRegionId = fRegionId;
                            q.push(make_pair(yIdx, xIdx-1));
                        } else if (legal(yIdx, xIdx-1) && _vPreGrid[layId][yIdx][xIdx-1].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx][xIdx-1].type = PreGridType::CONTOUR;
                            _vPreGrid[layId][yIdx][xIdx-1].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx, xIdx-1));
                            if (!sFound) {
                                sRowIdx = yIdx;
                                sColIdx = xIdx-1;
                                sFound = true;
                            }
                        } else if (legal(yIdx, xIdx-1) && _vPreGrid[layId][yIdx][xIdx-1].type == PreGridType::CONTOUR) {
                            _vPreGrid[layId][yIdx][xIdx-1].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx, xIdx-1));
                            if (!sFound) {
                                sRowIdx = yIdx;
                                sColIdx = xIdx-1;
                                sFound = true;
                            }
                        } else if (!legal(yIdx, xIdx-1)) {
                            _vPreGrid[layId][yIdx][xIdx].type = PreGridType::CONTOUR;
                            vContour.push_back(make_pair(yIdx, xIdx));
                        }

                        // explore rightward
                        if (legal(yIdx, xIdx+1) && _vPreGrid[layId][yIdx][xIdx+1].type == PreGridType::EMPTY) {
                            _vPreGrid[layId][yIdx][xIdx+1].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx][xIdx+1].fRegionId = fRegionId;
                            q.push(make_pair(yIdx, xIdx+1));
                        } else if (legal(yIdx, xIdx+1) && _vPreGrid[layId][yIdx][xIdx+1].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx][xIdx+1].type = PreGridType::CONTOUR;
                            _vPreGrid[layId][yIdx][xIdx+1].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx, xIdx+1));
                            if (!sFound) {
                                sRowIdx = yIdx;
                                sColIdx = xIdx+1;
                                sFound = true;
                            }
                        } else if (legal(yIdx, xIdx+1) && _vPreGrid[layId][yIdx][xIdx+1].type == PreGridType::CONTOUR) {
                            _vPreGrid[layId][yIdx][xIdx+1].fRegionId = fRegionId;
                            vContour.push_back(make_pair(yIdx, xIdx+1));
                            if (!sFound) {
                                sRowIdx = yIdx;
                                sColIdx = xIdx+1;
                                sFound = true;
                            }
                        } else if (!legal(yIdx, xIdx+1)) {
                            _vPreGrid[layId][yIdx][xIdx].type = PreGridType::CONTOUR;
                            vContour.push_back(make_pair(yIdx, xIdx));
                        }
                    }
                    cerr << "sRowIdx = " << sRowIdx << ", sColIdx = " << sColIdx << endl;
                    // double x1 = sColIdx * _gridWidth;
                    // double y1 = sRowIdx * _gridWidth;
                    // double x2 = (sColIdx + 1) * _gridWidth;
                    // double y2 = (sRowIdx + 1) * _gridWidth;
                    // vector< pair<double, double> > vVtx;
                    // vVtx.push_back(make_pair(x1, y1));
                    // vVtx.push_back(make_pair(x2, y1));
                    // vVtx.push_back(make_pair(x2, y2));
                    // vVtx.push_back(make_pair(x1, y2));
                    // Polygon* p = new Polygon(vVtx, _plot);
                    // p->plot(SVGPlotColor::black, layId);

                    FRegion* fRegion = constructFRegion(netId, layId, portId, fRegionId, vContour);
                    _db.vMetalLayer(layId)->addFRegion(fRegion);
                    _db.setPortinFRegion(layId, fRegionId, netId, portId);
                } else {
                    assert(_vPreGrid[layId][y0Idx][x0Idx].type == PreGridType::FREGION);
                    _db.setPortinFRegion(layId, _vPreGrid[layId][y0Idx][x0Idx].fRegionId, netId, portId);
                }
                
            }
        }
    }
}

FRegion* PreMgr::constructFRegion(size_t netId, size_t layId, size_t portId, size_t fRegionId, const vector< pair<int, int> >& vContour) {
    for (size_t contourId = 0; contourId < vContour.size(); ++ contourId) {
        int rowIdx = vContour[contourId].first;
        int colIdx = vContour[contourId].second;
        vector< pair<double, double> > vVtx;
        vector< vector< bool > > visited(_numRows, vector<bool>(_numCols, false));
        bool connected = constructFRegionDFS(layId, fRegionId, rowIdx, colIdx, rowIdx, colIdx, vVtx, visited);
        if (connected) {
            Polygon* polygon = new Polygon(vVtx, _plot);
            double portX, portY;
            if (portId == 0) {
                portX = _db.vSNode(netId, 0)->node()->ctrX();
                portY = _db.vSNode(netId, 0)->node()->ctrY();
            } else {
                portX = _vTClusteredNode[netId][portId-1][0]->node()->ctrX(); 
                portY = _vTClusteredNode[netId][portId-1][0]->node()->ctrY();
            }
            if (polygon->enclose(portX, portY)) {
                // polygon->plot(SVGPlotColor::gold, layId);
                FRegion* fRegion = new FRegion();
                fRegion->setPolygon(polygon);
                return fRegion;
            }
            
        }
    }
}

void PreMgr::plotFRegion() {
    cerr << "//////////////////" << endl;
    cerr << "// Plot FRegion //" << endl;
    cerr << "//////////////////" << endl;
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        cerr << "layId = " << layId << endl;
        for (size_t fRegionId = 0; fRegionId < _db.vMetalLayer(layId)->numFRegions(); ++ fRegionId) {
            cerr << "fRegionId = " << fRegionId << endl;
            FRegion* fRegion = _db.vMetalLayer(layId)->vFRegion(fRegionId);
            fRegion->polygon()->plot(SVGPlotColor::gold, layId);
            _plot.drawText(fRegion->polygon()->ctrX(), fRegion->polygon()->ctrY(), to_string(fRegionId), SVGPlotColor::black, layId, 24);
            fRegion->print();
        }
    }
    cerr << endl;
}

void PreMgr::kMeansClustering(size_t netId, vector<DBNode*> vNode, int numEpochs, int k) {
    struct Point {
        double x, y;     // coordinates
        int cluster;     // no default cluster
        double minDist;  // default infinite dist to nearest cluster
        DBNode* node;
    };
    auto distance = [] (Point p1, Point p2) -> double {
        return pow((p1.x-p2.x), 2) + pow((p1.y-p2.y), 2);
    }; 

    vector<Point> points;
    for (size_t nodeId = 0; nodeId < vNode.size(); ++ nodeId) {
        DBNode* node = vNode[nodeId];
        Point p = {node->node()->ctrX(), node->node()->ctrY(), -1, numeric_limits<double>::max(), node};
        points.push_back(p);
    }

    vector<bool> isCentroid(points.size(), false);
    vector<Point> centroids;
    srand(time(0));  // need to set the random seed
    for (int i = 0; i < k; ++i) {
        // centroids.push_back(points.at(rand() % points.size()));
        int pointId = rand() % points.size();
        while(isCentroid[pointId]) {
            pointId = rand() % points.size();
        }
        centroids.push_back(points.at(pointId));
        isCentroid[pointId] = true;
        // cerr << "centroid: (" << centroids[i].x << ", " << centroids[i].y << ")" << endl;
    }

    // vector<int> nPoints(k,0);
    // vector<double> sumX(k,0.0);
    // vector<double> sumY(k,0.0);
    int* nPoints = new int[k];
    double* sumX = new double[k];
    double* sumY = new double[k];
    for (size_t epoch = 0; epoch < numEpochs; ++ epoch) {
        for (size_t centId = 0; centId < centroids.size(); ++centId) {
            // quick hack to get cluster index
            // Point c = centroids[centId];
            int clusterId = centId;

            for (size_t pointId = 0; pointId < points.size(); ++ pointId) {
                // Point p = points[pointId];
                double dist = distance(centroids[centId],points[pointId]);
                if (dist < points[pointId].minDist) {
                    points[pointId].minDist = dist;
                    points[pointId].cluster = clusterId;
                    // cerr << "p.cluster = " << points[pointId].cluster << endl;
                }
                // *it = p;
            }
        }

        
        // sumX.clear();
        
        // sumY.clear();
        // // Initialise with zeroes
        // for (int j = 0; j < k; ++j) {
        //     nPoints.push_back(0);
        //     sumX.push_back(0.0);
        //     sumY.push_back(0.0);
        // }
        for (size_t centId=0; centId<centroids.size(); ++centId) {
            nPoints[centId] = 0;
            sumX[centId] = 0.0;
            sumY[centId] = 0.0;
        }
        // Iterate over points to append data to centroids
        // for (vector<Point>::iterator it = points.begin(); it != points.end(); ++it) {
            // int clusterId = it->cluster;
        for (size_t pointId = 0; pointId < points.size(); ++ pointId) {
            int clusterId = points[pointId].cluster;
            nPoints[clusterId] += 1;
            sumX[clusterId] += points[pointId].x;
            sumY[clusterId] += points[pointId].y;
            // cerr << "sumX" << clusterId << ": " << sumX[clusterId] << endl;
            // cerr << "sumY" << clusterId << ": " << sumY[clusterId] << endl;
            // cerr << "nPoints" << clusterId << ": " << nPoints[clusterId] << endl;
            points[pointId].minDist = numeric_limits<double>::max();  // reset distance
        }

        // Compute the new centroids
        // cerr << "Compute the new centroids" << endl;
        for (size_t centId = 0; centId < centroids.size(); ++ centId) {
            int clusterId = centId;
            centroids[centId].x = sumX[clusterId] / nPoints[clusterId];
            centroids[centId].y = sumY[clusterId] / nPoints[clusterId];
            // cerr << "sumX" << clusterId << ": " << sumX[clusterId] << endl;
            // cerr << "sumY" << clusterId << ": " << sumY[clusterId] << endl;
            // cerr << "nPoints" << clusterId << ": " << nPoints[clusterId] << endl;
            // cerr << "centroid: (" << centroids[centId].x << ", " << centroids[centId].y << ")" << endl;
        }
    }

    vector<size_t> vClusterId;  // index = [tPortId]
    assert(k=2);
    double sX = 0.5 * (_vSBoundBox[netId].maxX + _vSBoundBox[netId].minX);
    double sY = 0.5 * (_vSBoundBox[netId].maxY + _vSBoundBox[netId].minY);
    double dist0 = pow(centroids[0].x - sX, 2) + pow(centroids[0].y - sY, 2);
    double dist1 = pow(centroids[1].x - sX, 2) + pow(centroids[1].y - sY, 2);
    if (dist0 < dist1) {
        vClusterId.push_back(0);
        vClusterId.push_back(1);
    } else {
        vClusterId.push_back(1);
        vClusterId.push_back(0);
    }

    for (size_t pointId = 0; pointId < points.size(); ++ pointId) {
        Point p = points[pointId];
        for (size_t tPortId = 0; tPortId < k; ++ tPortId) {
            size_t clusterId = vClusterId[tPortId];
            if (p.cluster == clusterId) {
                // cerr << "tPortId = " << tPortId << endl;
                // cerr << "   node = " << p.node->name() << endl;
                _vTClusteredNode[netId][tPortId].push_back(p.node);
            }
        }
    }
    
}

Polygon* PreMgr::convexHull(vector<DBNode*> vNode) {
    struct Point 
    { 
        double x, y; 
    }; 

    auto orientation = [] (Point p, Point q, Point r) -> int {
        double val = (q.y - p.y) * (r.x - q.x) - 
              (q.x - p.x) * (r.y - q.y); 
  
        if (val == 0) return 0;  // collinear 
        return (val > 0)? 1: 2; // clock or counterclock wise 
    };

    vector<Point> points;
    for (size_t nodeId = 0; nodeId < vNode.size(); ++ nodeId) {
        Point p = {vNode[nodeId]->node()->ctrX(), vNode[nodeId]->node()->ctrY()};
        points.push_back(p);
    }

    size_t n = points.size();
    // There must be at least 3 points 
    // if (n < 3) return; 
    assert(points.size() >= 3);
  
    // Initialize Result 
    vector<Point> hull; 
  
    // Find the leftmost point 
    int l = 0; 
    for (int i = 1; i < n; i++) 
        if (points[i].x < points[l].x) 
            l = i; 
  
    // Start from leftmost point, keep moving counterclockwise 
    // until reach the start point again.  This loop runs O(h) 
    // times where h is number of points in result or output. 
    int p = l, q; 
    do
    { 
        // Add current point to result 
        hull.push_back(points[p]); 
  
        // Search for a point 'q' such that orientation(p, q, 
        // x) is counterclockwise for all points 'x'. The idea 
        // is to keep track of last visited most counterclock- 
        // wise point in q. If any point 'i' is more counterclock- 
        // wise than q, then update q. 
        q = (p+1)%n; 
        for (int i = 0; i < n; i++) 
        { 
           // If i is more counterclockwise than current q, then 
           // update q 
           if (orientation(points[p], points[i], points[q]) == 2) 
               q = i; 
        } 
  
        // Now q is the most counterclockwise with respect to p 
        // Set p as q for next iteration, so that q is added to 
        // result 'hull' 
        p = q; 
  
    } while (p != l);  // While we don't come to first point 
  
    // Print Result 
    // for (int i = 0; i < hull.size(); i++) 
    //     cout << "(" << hull[i].x << ", "
    //           << hull[i].y << ")\n"; 
    vector< pair<double, double> > vVtx;
    for (int i = 0; i < hull.size(); i++) {
        vVtx.push_back(make_pair(hull[i].x, hull[i].y));
    }
    Polygon* boundPolygon = new Polygon(vVtx, _plot);
    return boundPolygon;
}

bool PreMgr::constructFRegionDFS(const size_t& layId, const size_t& fRegionId, const int& rowIdx, const int& colIdx, const int& sRowIdx, const int& sColIdx, vector< pair<double, double> >& vVtx, vector< vector< bool > >& visited) {
    auto legal = [&](int yIdx, int xIdx) -> bool {
        return xIdx >= 0 && xIdx < _numCols && yIdx >= 0 && yIdx < _numRows;
    };

    // cerr << "rowIdx = " << rowIdx << ", colIdx = " << colIdx << endl;
    visited[rowIdx][colIdx] = true;

    // explore downward
    if (legal(rowIdx-1, colIdx) && 
        _vPreGrid[layId][rowIdx-1][colIdx].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx-1][colIdx].fRegionId == fRegionId && 
        visited[rowIdx-1][colIdx] == false) {
        // cerr << "downward" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx-1, colIdx, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx+0.5)*_gridWidth, (rowIdx-0.5)*_gridWidth));
            return true;
        }
    } 
    
    // explore leftward
    if (legal(rowIdx, colIdx-1) && 
        _vPreGrid[layId][rowIdx][colIdx-1].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx][colIdx-1].fRegionId == fRegionId && 
        visited[rowIdx][colIdx-1] == false) {
        // cerr << "leftward" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx, colIdx-1, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx-0.5)*_gridWidth, (rowIdx+0.5)*_gridWidth));
            return true;
        }
    } 
    
    // explore upward
    if (legal(rowIdx+1, colIdx) && 
        _vPreGrid[layId][rowIdx+1][colIdx].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx+1][colIdx].fRegionId == fRegionId && 
        visited[rowIdx+1][colIdx] == false) {
        // cerr << "upward" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx+1, colIdx, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx+0.5)*_gridWidth, (rowIdx+1.5)*_gridWidth));
            return true;
        }
    } 
    
    // explore rightward
    if (legal(rowIdx, colIdx+1) && 
        _vPreGrid[layId][rowIdx][colIdx+1].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx][colIdx+1].fRegionId == fRegionId && 
        visited[rowIdx][colIdx+1] == false) {
        // cerr << "rightward" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx, colIdx+1, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx+1.5)*_gridWidth, (rowIdx+0.5)*_gridWidth));
            return true;
        }
    } 
    
    // explore downleft
    if (legal(rowIdx-1, colIdx-1) && 
        _vPreGrid[layId][rowIdx-1][colIdx-1].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx-1][colIdx-1].fRegionId == fRegionId && 
        visited[rowIdx-1][colIdx-1] == false) {
        // cerr << "downleft" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx-1, colIdx-1, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx-0.5)*_gridWidth, (rowIdx-0.5)*_gridWidth));
            return true;
        }
    } 
    
    // explore downright
    if (legal(rowIdx-1, colIdx+1) && 
        _vPreGrid[layId][rowIdx-1][colIdx+1].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx-1][colIdx+1].fRegionId == fRegionId && 
        visited[rowIdx-1][colIdx+1] == false) {
        // cerr << "downright" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx-1, colIdx+1, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx+1.5)*_gridWidth, (rowIdx-0.5)*_gridWidth));
            return true;
        }
    } 

    // explore upright
    if (legal(rowIdx+1, colIdx+1) && 
        _vPreGrid[layId][rowIdx+1][colIdx+1].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx+1][colIdx+1].fRegionId == fRegionId && 
        visited[rowIdx+1][colIdx+1] == false) {
        // cerr << "upright" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx+1, colIdx+1, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx+1.5)*_gridWidth, (rowIdx+1.5)*_gridWidth));
            return true;
        }
    } 

    // explore upleft
    if (legal(rowIdx+1, colIdx-1) && 
        _vPreGrid[layId][rowIdx+1][colIdx-1].type == PreGridType::CONTOUR && 
        _vPreGrid[layId][rowIdx+1][colIdx-1].fRegionId == fRegionId && 
        visited[rowIdx+1][colIdx-1] == false) {
        // cerr << "upleft" << endl;
        if (constructFRegionDFS(layId, fRegionId, rowIdx+1, colIdx-1, sRowIdx, sColIdx, vVtx, visited)) {
            vVtx.push_back(make_pair((colIdx-0.5)*_gridWidth, (rowIdx+1.5)*_gridWidth));
            return true;
        }
    } 

    if (rowIdx >= sRowIdx-1 && rowIdx <= sRowIdx+1 && colIdx >= sColIdx-1 && colIdx <= sColIdx+1) {
        vVtx.push_back(make_pair((sColIdx+0.5)*_gridWidth, (sRowIdx+0.5)*_gridWidth));
        return true;
    }
    return false;
}

// bool PreMgr::detectLoop(const size_t& layId, const size_t& fRegionId, const int& rowIdx, const int& colIdx, const int& loopIdx, vector<vector< pair<double, double> > >& vVtx, vector< vector< char > >& state) {
//     auto legal = [&](int yIdx, int xIdx) -> bool {
//         return xIdx >= 0 && xIdx < _numCols && yIdx >= 0 && yIdx < _numRows;
//     };

//     state[rowIdx][colIdx] = 1;
//     if (loopIdx >= vVtx.size()) {
//         vVtx.push_back(vector< pair<double, double> >());
//     }
//     bool cyclic = false;
//     // explore downward
//     if (legal(rowIdx-1, colIdx) && 
//         _vPreGrid[layId][rowIdx-1][colIdx].type == PreGridType::CONTOUR && 
//         _vPreGrid[layId][rowIdx-1][colIdx].fRegionId == fRegionId) {
//         cerr << "downward" << endl;
//         if (state[rowIdx-1][colIdx] == 0) {
//             if (detectLoop(layId, fRegionId, rowIdx-1, colIdx, loopIdx, vVtx, state)) {
//                 vVtx[loopIdx].push_back(make_pair((colIdx+0.5)*_gridWidth, (rowIdx-0.5)*_gridWidth));
//                 cyclic = true;
//             }
//         } else if (state[rowIdx-1][colIdx] == 1) {
            
//         }
        
//     }
// }