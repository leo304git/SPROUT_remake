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
    // fill the contour line
    for (size_t vtxId = 0; vtxId < polygon->numVtcs(); ++ vtxId) {
        double x1 = polygon->vtxX(vtxId);
        double y1 = polygon->vtxY(vtxId);
        double x2 = polygon->vtxX((vtxId + 1) % polygon->numVtcs());
        double y2 = polygon->vtxY((vtxId + 1) % polygon->numVtcs());
        fillLineGridXArch(layId, x1, y1, x2, y2, name);
    }

/*
    // fill the inside of the polygon
    int minColIdx = floor(polygon->minX() / _gridWidth);
    int minRowIdx = floor(polygon->minY() / _gridWidth);
    int maxColIdx = floor(polygon->maxX() / _gridWidth);
    int maxRowIdx = floor(polygon->maxY() / _gridWidth);
    if (minColIdx == _numCols) minColIdx--;
    if (minRowIdx == _numRows) minRowIdx--;
    if (maxColIdx == _numCols) maxColIdx--;
    if (maxRowIdx == _numRows) maxRowIdx--;
    // enum State {OUTSIDE, INSIDE, ONBOUNDARY};
    // State state = OUTSIDE;
    // for (size_t rowIdx = minRowIdx; rowIdx <= maxRowIdx; ++ rowIdx) {
    //     bool inside = false;
    //     bool onBoundary = false;
    //     for (size_t colIdx = minColIdx; colIdx <= maxColIdx; ++ colIdx) {
    //         if (_vPreGrid[layId][rowIdx][colIdx].name == name) {
    //             if (!onBoundary) {
    //                 inside = !inside;
    //             }
    //             onBoundary = true;
    //             // if (colIdx >= 1) {
    //             //     if (_vPreGrid[layId][rowIdx][colIdx-1].name != name) {
    //             //         inside = true;
    //             //     } else if (inside) {
    //             //         inside = false;
    //             //     }
    //             // } else {
    //             //     inside = !inside;
    //             // }
    //         } else {
    //             if (inside) {
    //                 _vPreGrid[layId][rowIdx][colIdx].isOccupied = true;
    //                 _vPreGrid[layId][rowIdx][colIdx].name = name;
    //             }
    //             onBoundary = false;
    //         }
    //     }
    // }

    for (size_t rowIdx = minRowIdx; rowIdx <= maxRowIdx; ++ rowIdx) {
        bool onBoundary = false;
        int numIntersect = 0;
        for (size_t colIdx = minColIdx; colIdx <= maxColIdx; ++ colIdx) {
            if (_vPreGrid[layId][rowIdx][colIdx].name == name) {
                if (!onBoundary) {
                    numIntersect++;
                }
                onBoundary = true;
            } else {
                onBoundary = false;
            }
        }
        if (numIntersect > 1) {
            onBoundary = false;
            int numCurIntersect = 0;
            bool inside = false;
            for (size_t colIdx = minColIdx; colIdx <= maxColIdx; ++ colIdx) {
                if (_vPreGrid[layId][rowIdx][colIdx].name == name) {
                    if (!onBoundary) {
                        numCurIntersect++;
                        inside = !inside;
                        if (numCurIntersect == numIntersect) {
                            inside = false;
                        }
                    }
                    onBoundary = true;
                    // if (colIdx >= 1) {
                    //     if (_vPreGrid[layId][rowIdx][colIdx-1].name != name) {
                    //         inside = true;
                    //     } else if (inside) {
                    //         inside = false;
                    //     }
                    // } else {
                    //     inside = !inside;
                    // }
                } else {
                    if (inside) {
                        _vPreGrid[layId][rowIdx][colIdx].isOccupied = true;
                        _vPreGrid[layId][rowIdx][colIdx].name = name;
                    }
                    onBoundary = false;
                }
            }
        }
    
    }
*/
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
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            for (size_t portId = 0; portId < _db.vNet(netId)->numTPorts()+1; ++ portId) {
                if (portId == 0) {
                    // _db.vNet(netId)->sourcePort()->boundPolygon()->clearGrid(layId);
                } else {
                    // _db.vNet(netId)->targetPort(portId-1)->boundPolygon()->clearGrid(layId);
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

    size_t fRegionId = 0;
    FRegion* fRegion;
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t portId = 0; portId < _db.vNet(netId)->numTPorts()+1 ; ++ portId) {
            int x0Idx;
            int y0Idx;
            if (portId == 0) {
                x0Idx = floor(_db.vSNode(netId, 0)->node()->ctrX() / _gridWidth);
                y0Idx = floor(_db.vSNode(netId, 0)->node()->ctrY() / _gridWidth);
            } else {
                x0Idx = floor(_vTClusteredNode[netId][portId-1][0]->node()->ctrX() / _gridWidth); 
                y0Idx = floor(_vTClusteredNode[netId][portId-1][0]->node()->ctrY() / _gridWidth);
            }
            for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
                cerr << "layId = " << layId << endl;
                if (_vPreGrid[layId][y0Idx][x0Idx].type == PreGridType::OBSTACLE) {
                    continue;
                }
                assert(_vPreGrid[layId][y0Idx][x0Idx].type != PreGridType::OBSTACLE);
                if (_vPreGrid[layId][y0Idx][x0Idx].type == PreGridType::EMPTY) {
                    fRegionId++;
                    // BFS
                    queue< pair<int, int> > q;
                    // _vPreGrid[layId][y0Idx][x0Idx].name = "RailSpace" + to_string(fRegionId);
                    q.push(make_pair(y0Idx, x0Idx));
                    while (!q.empty()) {
                        pair<int, int> p = q.front();
                        q.pop();
                        int yIdx = p.first;
                        int xIdx = p.second;
                        // cerr << "yIdx = " << yIdx << ", xIdx = " << xIdx << endl;
                        assert(xIdx >= 0 && xIdx < _numCols && yIdx >= 0 && yIdx < _numRows);
                        // _vPreGrid[layId][yIdx][xIdx].name = "RailSpace";
                        if (legal(yIdx-1, xIdx) && _vPreGrid[layId][yIdx-1][xIdx].type == PreGridType::EMPTY) {
                            // _vPreGrid[layId][yIdx-1][xIdx].name = "RailSpace";
                            _vPreGrid[layId][yIdx-1][xIdx].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx-1][xIdx].fRegionId = fRegionId;
                            q.push(make_pair(yIdx-1, xIdx));
                        } else if (legal(yIdx-1, xIdx) && _vPreGrid[layId][yIdx-1][xIdx].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx-1][xIdx].type = PreGridType::CONTOUR;
                        }
                        if (legal(yIdx+1, xIdx) && _vPreGrid[layId][yIdx+1][xIdx].type == PreGridType::EMPTY) {
                            // _vPreGrid[layId][yIdx+1][xIdx].name = "RailSpace";
                            _vPreGrid[layId][yIdx+1][xIdx].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx+1][xIdx].fRegionId = fRegionId;
                            q.push(make_pair(yIdx+1, xIdx));
                        } else if (legal(yIdx-1, xIdx) && _vPreGrid[layId][yIdx-1][xIdx].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx+1][xIdx].type = PreGridType::CONTOUR;
                        }
                        if (legal(yIdx, xIdx-1) && _vPreGrid[layId][yIdx][xIdx-1].type == PreGridType::EMPTY) {
                            // _vPreGrid[layId][yIdx][xIdx-1].name = "RailSpace";
                            _vPreGrid[layId][yIdx][xIdx-1].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx][xIdx-1].fRegionId = fRegionId;
                            q.push(make_pair(yIdx, xIdx-1));
                        } else if (legal(yIdx, xIdx-1) && _vPreGrid[layId][yIdx][xIdx-1].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx][xIdx-1].type = PreGridType::CONTOUR;
                        }
                        if (legal(yIdx, xIdx+1) && !_vPreGrid[layId][yIdx][xIdx+1].type == PreGridType::EMPTY) {
                            // _vPreGrid[layId][yIdx][xIdx+1].name = "RailSpace";
                            _vPreGrid[layId][yIdx][xIdx+1].type = PreGridType::FREGION;
                            _vPreGrid[layId][yIdx][xIdx+1].fRegionId = fRegionId;
                            q.push(make_pair(yIdx, xIdx+1));
                        } else if (legal(yIdx, xIdx+1) && _vPreGrid[layId][yIdx][xIdx+1].type == PreGridType::OBSTACLE) {
                            _vPreGrid[layId][yIdx][xIdx+1].type = PreGridType::CONTOUR;
                        }
                    }
                    fRegion = constructFRegion(netId, layId);
                    _db.vMetalLayer(layId)->addFRegion(fRegion);
                } else {
                    assert(_vPreGrid[layId][y0Idx][x0Idx].type == PreGridType::FREGION);
                    fRegion = _db.vMetalLayer(layId)->vFRegion(_vPreGrid[layId][y0Idx][x0Idx].fRegionId);
                }
                _db.setPortinFRegion(fRegion, netId, portId);
            }
        }
    }
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