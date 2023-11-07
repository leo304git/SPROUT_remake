#include "DetailedMgr.h"
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/SparseCholesky>

void DetailedMgr::plotDB() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t viaId = 0; viaId < _db.numVias(); ++ viaId) {
            Via* via = _db.vVia(viaId);
            via->shape()->plot(via->netId(), layId);
            // _plot.drawCircle(via->shape()->ctrX(), via->shape()->ctrY(), via->shape()->radius(), via->netId());
        }
        for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
            Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
            for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                obs->vShape(shapeId)->plot(SVGPlotColor::gray, layId);
            }
        }
    }
    
}

void DetailedMgr::initGridMap() {

    auto occupy = [&] (size_t xId, size_t yId, Shape* shape) -> bool {
        if (shape->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
        if (shape->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
        if (shape->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
        if (shape->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
        return false;
    };

    auto fullyOccupy = [&] (size_t xId, size_t yId, Shape* shape) -> bool {
        if (!shape->enclose(xId*_gridWidth, yId*_gridWidth)) return false;
        if (!shape->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return false;
        if (!shape->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return false;
        if (!shape->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return false;
        return true;
    };

    // set obstacle & viaCluster grids
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                // set obstacle grids
                for (size_t obsId = 0; obsId < _db.numObstacles(layId); ++ obsId) {
                    if (occupy(xId, yId, _db.vObstacle(layId, obsId)->vShape(0))) {
                        grid->setOccupied(true);
                    }
                }
                // set viaCluster grids
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    // source
                    if (occupy(xId, yId, _db.vNet(netId)->sourceViaCstr()->bBox())) {
                        if (fullyOccupy(xId, yId, _db.vNet(netId)->sourceViaCstr()->bBox())) {  // inner grids
                            grid->setOccupied(true);
                            grid->setNetId(netId);
                        } else {    // outer rim grids
                            grid->setOccupied(false);
                            grid->setNetId(netId);
                            grid->addNeighbor(_vSGrid[netId][layId]);
                            _vSGrid[netId][layId]->addNeighbor(grid);
                        }
                    }
                    // target
                    for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                        if (occupy(xId, yId, _db.vNet(netId)->vTargetViaCstr(tPortId)->bBox())) {
                            if (fullyOccupy(xId, yId, _db.vNet(netId)->vTargetViaCstr(tPortId)->bBox())) {  // inner grids
                                grid->setOccupied(true);
                                grid->setNetId(netId);
                            } else {    // outer rim grids
                                grid->setOccupied(false);
                                grid->setNetId(netId);
                                grid->addNeighbor(_vTGrid[netId][layId][tPortId]);
                                _vTGrid[netId][layId][tPortId]->addNeighbor(grid);
                            }
                        }
                    }
                }
            }
        }
    }

    // set neighbors
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                if (legal(xId+1, yId)) {
                    Grid* rGrid = _vGrid[layId][xId+1][yId];
                    if (!rGrid->occupied()) {
                        grid->addNeighbor(rGrid);
                    }
                }
                if (legal(xId-1, yId)) {
                    Grid* lGrid = _vGrid[layId][xId-1][yId];
                    if (!lGrid->occupied()) {
                        grid->addNeighbor(lGrid);
                    }
                }
                if (legal(xId, yId+1)) {
                    Grid* uGrid = _vGrid[layId][xId][yId+1];
                    if (!uGrid->occupied()) {
                        grid->addNeighbor(uGrid);
                    }
                }
                if (legal(xId, yId-1)) {
                    Grid* dGrid = _vGrid[layId][xId][yId-1];
                    if (!dGrid->occupied()) {
                        grid->addNeighbor(dGrid);
                    }
                }
            }
        }
    }

    // set occupy of the outer rims
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            Grid* grid = _vSGrid[netId][layId];
            for (size_t nbrId = 0; nbrId < grid->numNeighbors(); ++ nbrId) {
                grid->vNeighbor(nbrId)->setOccupied(true);
            }
            for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                Grid* tGrid = _vTGrid[netId][layId][tPortId];
                for (size_t nbrId = 0; nbrId < tGrid->numNeighbors(); ++ nbrId) {
                    tGrid->vNeighbor(nbrId)->setOccupied(true);
                }
            }
        }
    }
    // auto occupy = [&] (size_t layId, size_t xId, size_t yId, size_t netId) -> bool {
    //     for (size_t segId = 0; segId < _db.vNet(netId)->numSegments(layId); ++ segId) {
    //         Trace* trace = _db.vNet(netId)->vSegment(layId, segId)->trace();
    //         if (trace->enclose(xId*_gridWidth, yId*_gridWidth)) return true;
    //         if (trace->enclose((xId+1)*_gridWidth, yId*_gridWidth)) return true;
    //         if (trace->enclose(xId*_gridWidth, (yId+1)*_gridWidth)) return true;
    //         if (trace->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) return true;
    //     }
    //     return false;
    // };

    // for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //     for (size_t xId = 0; xId < _numXs; ++ xId) {
    //         for (size_t yId = 0; yId < _numYs; ++ yId) {
    //             Grid* grid = _vGrid[layId][xId][yId];
    //             for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
    //                 if (occupy(layId, xId, yId, netId)) {
    //                     grid->addNet(netId);
    //                     grid->incCongestCur();
    //                     _vNetGrid[netId][layId].push_back(grid);
    //                 }
    //             }
    //         }
    //     }
    // }

    // for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
    //     Net* net = _db.vNet(netId);
    //     for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //         for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
    //             Segment* segment = net->vSegment(layId, segId);
    //             Circle* sCircle = new Circle(segment->sX(), segment->sY(), segment->width()/2, _plot);
    //             Circle* tCircle = new Circle(segment->tX(), segment->tY(), segment->width()/2, _plot);
    //             int sXId = floor(segment->sX() / _gridWidth);
    //             int sYId = floor(segment->sY() / _gridWidth);
    //             int tXId = floor(segment->tX() / _gridWidth);
    //             int tYId = floor(segment->tY() / _gridWidth);
    //             int boxWidth = ceil(segment->width() / _gridWidth);
    //             for (int xId = sXId - ceil(boxWidth); xId <= sXId + ceil(boxWidth); ++ xId) {
    //                 for (int yId = sYId - ceil(boxWidth); yId <= sYId + ceil(boxWidth); ++ yId) {
    //                     if (legal(xId, yId)) {
    //                         Grid* grid = _vGrid[layId][xId][yId];
    //                         if (!grid->hasNet(netId)) {
    //                             if (sCircle->Circle::enclose(xId*_gridWidth, yId*_gridWidth) ||
    //                                 sCircle->Circle::enclose((xId+1)*_gridWidth, yId*_gridWidth) ||
    //                                 sCircle->Circle::enclose(xId*_gridWidth, (yId+1)*_gridWidth) ||
    //                                 sCircle->Circle::enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //             for (int xId = tXId - ceil(boxWidth/2.0); xId <= tXId + ceil(boxWidth/2.0); ++ xId) {
    //                 for (int yId = tYId - ceil(boxWidth/2.0); yId <= tYId + ceil(boxWidth/2.0); ++ yId) {
    //                     if (legal(xId, yId)) {
    //                         Grid* grid = _vGrid[layId][xId][yId];
    //                         if (!grid->hasNet(netId)) {
    //                             if (tCircle->enclose(xId*_gridWidth, yId*_gridWidth) ||
    //                                 tCircle->enclose((xId+1)*_gridWidth, yId*_gridWidth) ||
    //                                 tCircle->enclose(xId*_gridWidth, (yId+1)*_gridWidth) ||
    //                                 tCircle->enclose((xId+1)*_gridWidth, (yId+1)*_gridWidth)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }    
    // }
    
    // for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
    //     Net* net = _db.vNet(netId);
    //     for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //         for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
    //             Segment* segment = net->vSegment(layId, segId);
    //             int sXId = floor(segment->sX() / _gridWidth);
    //             int sYId = floor(segment->sY() / _gridWidth);
    //             int tXId = floor(segment->tX() / _gridWidth);
    //             int tYId = floor(segment->tY() / _gridWidth);
    //             int boxWidth = ceil(segment->width() / _gridWidth);
    //             for (int xId = sXId - floor(boxWidth/2.0); xId <= sXId + floor(boxWidth/2.0); ++ xId) {
    //                 if (xId >= 0 && xId < _vGrid[0].size()) {
    //                     for (int yId = sYId - floor(boxWidth/2.0); yId <= sYId + floor(boxWidth/2.0); ++ yId) {
    //                         if (yId >= 0 && yId < _vGrid[0][0].size()) {
    //                             Grid* grid = _vGrid[layId][xId][yId];
    //                             if (!grid->hasNet(netId)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //             for (int xId = tXId - floor(boxWidth/2.0); xId <= tXId + floor(boxWidth/2.0); ++ xId) {
    //                 if (xId >= 0 && xId < _vGrid[0].size()) {
    //                     for (int yId = tYId - floor(boxWidth/2.0); yId <= tYId + floor(boxWidth/2.0); ++ yId) {
    //                         if (yId >= 0 && yId < _vGrid[0][0].size()) {
    //                             Grid* grid = _vGrid[layId][xId][yId];
    //                             if (!grid->hasNet(netId)) {
    //                                 grid->addNet(netId);
    //                                 grid->incCongestCur();
    //                                 _vNetGrid[netId][layId].push_back(grid);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }
}

void DetailedMgr::plotGraph() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                vector< pair<double, double> > vVtx;
                vVtx.push_back(make_pair(xId*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, (yId+1)*_gridWidth));
                vVtx.push_back(make_pair(xId*_gridWidth, (yId+1)*_gridWidth));
                Polygon* p = new Polygon(vVtx, _plot);
                if (grid->occupied()) {
                    p->plot(SVGPlotColor::gray, layId);
                } else {
                    p->plot(SVGPlotColor::white, layId);
                }
            }    
        }
    }
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            Grid* sGrid = _vSGrid[netId][layId];
            for (size_t nbrId = 0; nbrId < sGrid->numNeighbors(); ++ nbrId) {
                Grid* grid = sGrid->vNeighbor(nbrId);
                size_t xId = grid->xId();
                size_t yId = grid->yId();
                vector< pair<double, double> > vVtx;
                vVtx.push_back(make_pair(xId*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, (yId+1)*_gridWidth));
                vVtx.push_back(make_pair(xId*_gridWidth, (yId+1)*_gridWidth));
                Polygon* p = new Polygon(vVtx, _plot);
                p->plot(netId, layId);
            }
            for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                Grid* tGrid = _vTGrid[netId][layId][tPortId];
                for (size_t nbrId = 0; nbrId < tGrid->numNeighbors(); ++ nbrId) {
                    Grid* grid = tGrid->vNeighbor(nbrId);
                    size_t xId = grid->xId();
                    size_t yId = grid->yId();
                    vector< pair<double, double> > vVtx;
                    vVtx.push_back(make_pair(xId*_gridWidth, yId*_gridWidth));
                    vVtx.push_back(make_pair((xId+1)*_gridWidth, yId*_gridWidth));
                    vVtx.push_back(make_pair((xId+1)*_gridWidth, (yId+1)*_gridWidth));
                    vVtx.push_back(make_pair(xId*_gridWidth, (yId+1)*_gridWidth));
                    Polygon* p = new Polygon(vVtx, _plot);
                    p->plot(netId, layId);
                }
            }
        }
    }
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                Circle* c = new Circle((xId+0.5)*_gridWidth, (yId+0.5)*_gridWidth, 0.25*_gridWidth, _plot);
                if (!grid->occupied()) {
                    c->plot(SVGPlotColor::gold, layId);
                    for (size_t nbrId = 0; nbrId < grid->numNeighbors(); ++ nbrId) {
                        // if (!grid->vNeighbor(nbrId)->occupied()) {
                            double x1 = (xId+0.5)*_gridWidth;
                            double y1 = (yId+0.5)*_gridWidth;
                            double x2 = (grid->vNeighbor(nbrId)->xId()+0.5)*_gridWidth;
                            double y2 = (grid->vNeighbor(nbrId)->yId()+0.5)*_gridWidth;
                            _plot.drawLine(x1, y1, x2, y2, SVGPlotColor::blue, layId, 1.0);
                        // }
                    }
                }
            }
        }
    }
}

void DetailedMgr::plotGridMap() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                vector< pair<double, double> > vVtx;
                vVtx.push_back(make_pair(xId*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, yId*_gridWidth));
                vVtx.push_back(make_pair((xId+1)*_gridWidth, (yId+1)*_gridWidth));
                vVtx.push_back(make_pair(xId*_gridWidth, (yId+1)*_gridWidth));
                Polygon* p = new Polygon(vVtx, _plot);
                if (grid->netId() == -1) {
                    if (grid->occupied()) {
                        p->plot(SVGPlotColor::gray, layId);
                    } else {
                        p->plot(SVGPlotColor::white, layId);
                    }
                } else {
                    p->plot(grid->netId(), layId);
                }
            }
        }
    }
    // int area = 0;
    // int overlapArea = 0;
    // for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //     for (size_t xId = 0; xId < _numXs; ++ xId) {
    //         for (size_t yId = 0; yId < _numYs; ++ yId) {
    //             Grid* grid = _vGrid[layId][xId][yId];
    //             if (grid->congestCur() >= 1) area++;
    //             overlapArea += grid->congestCur();
    //         }
    //     }
    // }
    // overlapArea -= area;
    // cerr << "area = " << area << endl;
    // cerr << "overlapArea = " << overlapArea << endl;
}

void DetailedMgr::plotGridMapVoltage() {
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        // cerr << "netId = " << netId << endl;
        double ubVolt = _db.vNet(netId)->sourcePort()->voltage();
        // double lbVolt = _db.vNet(netId)->targetPort(0)->voltage();
        // for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
        //     if (_db.vNet(netId)->targetPort(tPortId)->voltage() < lbVolt) {
        //         lbVolt = _db.vNet(netId)->targetPort(tPortId)->voltage();
        //     }
        // }
        double lbVolt = ubVolt * 0;
        _plot.setColorValueRange(lbVolt, ubVolt);
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            // cerr << " layId = " << layId << endl;
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid = _vNetGrid[netId][layId][gridId];
                // cerr << "   voltage = " << grid->voltage() << endl;
                Square* square = new Square(grid->xId()*_gridWidth, grid->yId()*_gridWidth, _gridWidth, _plot);
                square->plotValue(grid->voltage(), layId);
            }
        }
    }
}

void DetailedMgr::plotGridMapCurrent() {
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
        // cerr << "netId = " << netId << endl;
        // double ubCurr = _db.vNet(netId)->sourcePort()->voltage() * via_condutance;
        double ubCurr = 120;
        // double lbVolt = _db.vNet(netId)->targetPort(0)->voltage();
        // for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
        //     if (_db.vNet(netId)->targetPort(tPortId)->voltage() < lbVolt) {
        //         lbVolt = _db.vNet(netId)->targetPort(tPortId)->voltage();
        //     }
        // }
        double lbCurr = 0;
        _plot.setColorValueRange(lbCurr, ubCurr);
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            // cerr << " layId = " << layId << endl;
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid = _vNetGrid[netId][layId][gridId];
                // cerr << "   voltage = " << grid->voltage() << endl;
                Square* square = new Square(grid->xId()*_gridWidth, grid->yId()*_gridWidth, _gridWidth, _plot);
                square->plotValue(grid->current(), layId);
            }
        }
    }
}

void DetailedMgr::naiveAStar() {
    // cerr << "naiveAStar..." << endl;
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        // cerr << "layId = " << layId;
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            // cerr << " netId = " << netId << endl;
    // size_t layId = 0;
    // size_t netId = 0;
            Net* net = _db.vNet(netId);
            // clearNet(layId, netId);
            Grid* grid = _vSGrid[netId][layId];

            // unoccupied the outer rim grids
            for (size_t nbrId = 0; nbrId < grid->numNeighbors(); ++ nbrId) {
                grid->vNeighbor(nbrId)->setOccupied(false);
            }
            for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                Grid* tGrid = _vTGrid[netId][layId][tPortId];
                for (size_t nbrId = 0; nbrId < tGrid->numNeighbors(); ++ nbrId) {
                    tGrid->vNeighbor(nbrId)->setOccupied(false);
                }
            }
            for (size_t tPortId = 0; tPortId < net->numTPorts(); ++ tPortId) {
                AStarRouter sRouter(_vGrid[layId], _vSGrid[netId][layId], _vTGrid[netId][layId][tPortId], _gridWidth);
                if (sRouter.BFS()) {
                    for (size_t pGridId = 0; pGridId < sRouter.numPath(); ++ pGridId) {
                        Grid* grid = sRouter.path(pGridId);
                        if (grid->netId() == -1) {  // avoid pushing the same grid to _vNetGrid again
                            grid->setNetId(netId);
                            // grid->setOccupied(true);
                            if (grid != _vSGrid[netId][layId] && grid != _vTGrid[netId][layId][tPortId]) {
                                _vNetGrid[netId][layId].push_back(grid);
                            }
                        }
                    }
                } else {
                    cerr << "BFS fail: lay" << layId << " net" << netId << " s->t" << tPortId << endl;
                }
                
                for (size_t tPortId1 = tPortId+1; tPortId1 < net->numTPorts(); ++ tPortId1) {
                    AStarRouter tRouter(_vGrid[layId], _vTGrid[netId][layId][tPortId], _vTGrid[netId][layId][tPortId1], _gridWidth);
                    if (tRouter.BFS()) {
                        for (size_t pGridId = 0; pGridId < tRouter.numPath(); ++ pGridId) {
                            Grid* grid = tRouter.path(pGridId);
                            if (grid->netId() == -1) {
                                grid->setNetId(netId);
                                // grid->setOccupied(true);
                                if (grid != _vTGrid[netId][layId][tPortId] && grid != _vTGrid[netId][layId][tPortId1]) {
                                    _vNetGrid[netId][layId].push_back(grid);
                                }
                            }
                        }
                    } else {
                        cerr << "BFS fail: lay" << layId << " net" << netId << " t" << tPortId << "->t" << tPortId1 << endl;
                    }
                    
                }
            }

            // occupied the net grids
            for (size_t netGridId = 0; netGridId < _vNetGrid[netId][layId].size(); ++ netGridId) {
                _vNetGrid[netId][layId][netGridId]->setOccupied(true);
            }
            // occupied the outer rim grids
            for (size_t nbrId = 0; nbrId < grid->numNeighbors(); ++ nbrId) {
                grid->vNeighbor(nbrId)->setOccupied(true);
            }
            for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                Grid* tGrid = _vTGrid[netId][layId][tPortId];
                for (size_t nbrId = 0; nbrId < tGrid->numNeighbors(); ++ nbrId) {
                    tGrid->vNeighbor(nbrId)->setOccupied(true);
                }
            }
            
            // for (size_t segId = 0; segId < net->numSegments(layId); ++ segId) {
            //     Segment* segment = net->vSegment(layId, segId);
            //     int sXId = floor(segment->sX() / _gridWidth);
            //     int sYId = floor(segment->sY() / _gridWidth);
            //     int tXId = floor(segment->tX() / _gridWidth);
            //     int tYId = floor(segment->tY() / _gridWidth);
            //     AStarRouter router(_vGrid[layId], make_pair(sXId, sYId), make_pair(tXId, tYId), _gridWidth, segment->length(), segment->width(), 0.9, _db.numNets() * 10.0, 0.2);
            //     router.route();
            //     segment->setWidth(router.exactWidth() * _gridWidth);
            //     segment->setLength(router.exactLength() * _gridWidth);
            //     for (size_t pGridId = 0; pGridId < router.numPGrids(); ++ pGridId) {
            //         Grid* grid = router.vPGrid(pGridId);
            //         if (!grid->hasNet(netId)) {
            //             _vNetGrid[netId][layId].push_back(grid);
            //             grid->addNet(netId);
            //         }
            //     }
            //     // _vNetGrid[netId][layId].insert(_vNetGrid[netId][layId].end(), router.path().begin(), router.path().end());
            // }
            // for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
            //     _vNetGrid[netId][layId][gridId]->incCongestCur();
            // }
        }
    }
    // int area = 0;
    // int overlapArea = 0;
    // for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //     for (size_t xId = 0; xId < _numXs; ++ xId) {
    //         for (size_t yId = 0; yId < _numYs; ++ yId) {
    //             Grid* grid = _vGrid[layId][xId][yId];
    //             if (grid->congestCur() >= 1) area++;
    //             overlapArea += grid->congestCur();
    //         }
    //     }
    // }
    // overlapArea -= area;
    // cerr << "area = " << area << endl;
    // cerr << "overlapArea = " << overlapArea << endl;
}

void DetailedMgr::clearNet(size_t layId, size_t netId) {
    // for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
    //     _vNetGrid[netId][layId][gridId]->removeNet(netId);
    //     _vNetGrid[netId][layId][gridId]->decCongestCur();
    // }
    // _vNetGrid[netId][layId].clear();
}

void DetailedMgr::eigenTest() {
    int N = 3;
    Eigen::SparseMatrix<double, Eigen::RowMajor> A(N, N);
    Eigen::VectorXd b(N);
    Eigen::VectorXd x(N);
    vector< Eigen::Triplet<double> > vTplA;
    vTplA.reserve(9);
    vTplA.push_back(Eigen::Triplet<double>(0,0,2));
    vTplA.push_back(Eigen::Triplet<double>(0,1,-1));
    vTplA.push_back(Eigen::Triplet<double>(1,0,-1));
    vTplA.push_back(Eigen::Triplet<double>(1,1,2));
    vTplA.push_back(Eigen::Triplet<double>(1,2,-1));
    vTplA.push_back(Eigen::Triplet<double>(2,1,-1));
    vTplA.push_back(Eigen::Triplet<double>(2,2,2));
    vTplA.push_back(Eigen::Triplet<double>(0,0,-1));
    A.setFromTriplets(vTplA.begin(), vTplA.end());
    b(0) = 1;
    // b(1) = 0;
    // b(2) = 0;

    // Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IdentityPreconditioner> solver;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
    solver.setMaxIterations(100);
    solver.setTolerance(1e-6);
    solver.compute(A);
    x = solver.solveWithGuess(b, x);

    cerr << "A = \n" << Eigen::MatrixXd(A) << endl;
    cerr << "b = \n" << b << endl;
    cerr << "x = " << x << endl;

    double x0 = x[0];
    cerr << "x0 = " << x0 << endl;
    // A.coeffRef(0,0) = 2;
    // A.coeffRef(0,1) = -1;
    // A.coeffRef(0,2) = 0;
    // A.coeffRef(1,0) = -1;
    // A.coeffRef(1,1) = 2;
    // A.coeffRef(1,2) = -1;
    // A.coeffRef(2,0) = 0;
    // A.coeffRef(2,1) = -1;
    // A.coeffRef(2,2) = 2;
    // A.makeCompressed();
    // b.coeffRef(0) = 1;
    // b.coeffRef(1) = 0;
    // b.coeffRef(2) = 0;

    // ConjugateGradient<SparseMatrix<double>, Eigen::Upper> solver;
    // x = solver.compute(A).solve(b);
}

void DetailedMgr::synchronize() {
    auto hasGrid = [] (vector<Grid*> grids, Grid* grid) -> bool {
        for (size_t gridId = 0; gridId < grids.size(); ++ gridId) {
            if (grids[gridId] == grid) {
                return true;
            }
        }
        return false;
    };
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                if (grid->netId() >= 0) {
                    assert(grid->occupied());
                    if (! hasGrid(_vNetGrid[grid->netId()][layId], grid)) {
                        _vNetGrid[grid->netId()][layId].push_back(grid);
                    }
                }
            }
        }
    }
}

void DetailedMgr::addViaGrid() {
    auto gridEnclose = [&] (Grid* grid, double x, double y) -> bool {
        double gridLX = grid->xId() * _gridWidth;
        double gridUX = (grid->xId()+1) * _gridWidth;
        double gridLY = grid->yId() * _gridWidth;
        double gridUY = (grid->yId()+1) * _gridWidth;
        // to avoid a via enclosed by multiple grids, set ">=" but "<" only
        return ((x >= gridLX) && (x < gridUX) && (y >= gridLY) && (y < gridUY));
    };
    // cerr << "addViaGrid..." << endl;
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId < _numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    if (grid->netId() != netId) {
                        for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
                            double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
                            double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
                            // cerr << "sVia = (" << sX << ", " << sY << ")" << endl;
                            if (gridEnclose(grid, sX, sY)) {
                                _vNetGrid[netId][layId].push_back(grid);
                                grid->setNetId(netId);
                                // cerr << "net" << netId << "sGrid = (" << xId << ", " << yId << ")" << endl;
                            }
                        }
                        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                            for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
                                double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
                                double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
                                if (gridEnclose(grid, tX, tY)) {
                                    _vNetGrid[netId][layId].push_back(grid);
                                    grid->setNetId(netId);
                                    // cerr << "net" << netId << "tGrid" << tPortId << " = (" << xId << ", " << yId << ")" << endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void DetailedMgr::buildMtx() {
    // https://i.imgur.com/rIwlXJQ.png
    // return an impedance matrix for each net
    // number of nodes: \sum_{layId=0}^{_vNetGrid[netID].size()} _vNetGrid[netID][layId].size()

    auto gridEnclose = [&] (Grid* grid, double x, double y) -> bool {
        double gridLX = grid->xId() * _gridWidth;
        double gridUX = (grid->xId()+1) * _gridWidth;
        double gridLY = grid->yId() * _gridWidth;
        double gridUY = (grid->yId()+1) * _gridWidth;
        // to avoid a via enclosed by multiple grids, set ">=" but "<" only
        return ((x >= gridLX) && (x < gridUX) && (y >= gridLY) && (y < gridUY));
    };

    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        printf("netID: %d\n", netId);
        
        size_t numNode = 0;
        map< tuple<size_t, size_t, size_t>, size_t > getID; // i = getID[layID, xId, yId] = ith node
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                getID[make_tuple(layId, _vNetGrid[netId][layId][gridId]->xId(), _vNetGrid[netId][layId][gridId]->yId())] = numNode;
                numNode++;
            }
        }
        printf("numNode: %d\n", numNode);

        // initialize matrix and vector
        Eigen::SparseMatrix<double, Eigen::RowMajor> Y(numNode, numNode);
        Eigen::VectorXd I(numNode);
        Eigen::VectorXd V(numNode);
        vector< Eigen::Triplet<double> > vTplY;
        vTplY.reserve(6 * numNode);
        for (size_t i = 0; i < numNode; ++ i) {
            I[i] = 0;
            V[i] = 0;
        }

        // // initialize
        // vector< vector<double > > mtx;
        // // numNode += (_db.vNet(netId)->numTPorts() + 1) * _db.numLayers();    // add the source/target via nodes on each layer
        // for(int i=0; i<numNode; i++) {
        //     mtx.push_back(vector<double>());
        //     for(int j=0; j<numNode; j++)
        //         mtx[i].push_back(0.0);
        // }
        assert(_vNetGrid[netId].size() == _db.numLayers());
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                assert(grid_i->netId() == netId);
                // cerr << "grid = (" << grid_i->xId() << " " << grid_i->yId() << ")" << endl;
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
            //    printf("x: %-4d, y: %-4d, lay: %-4d, ID: %-4d\n", i->xId(), i->yId(), layId, getID[make_tuple(layId, i->xId(), i->yId())]);
            
                double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
                // double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
                double via_condutance_up, via_condutance_down;
                if (layId > 0) {
                    via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
                }
                if (layId < _db.numLayers() - 1) {
                    via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
                }
                double small_conductance = 1.0;

                // check left
                if(grid_i->xId() > 0 && _vGrid[layId][grid_i->xId()-1][grid_i->yId()]->netId() == netId) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())], -g2g_condutance));
                }

                // check right
                if(grid_i->xId() < _numXs-1 && _vGrid[layId][grid_i->xId()+1][grid_i->yId()]->netId() == netId) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())], -g2g_condutance));
                }
                
                // check down
                if(grid_i->yId() > 0 && _vGrid[layId][grid_i->xId()][grid_i->yId()-1]->netId() == netId) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)], -g2g_condutance));
                }
                
                // check up
                if(grid_i->yId() < _numYs-1 && _vGrid[layId][grid_i->xId()][grid_i->yId()+1]->netId() == netId) {
                    // mtx[node_id][node_id] += g2g_condutance;
                    // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)]] -= g2g_condutance;
                    vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
                    vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)], -g2g_condutance));
                }

                // // check top layer
                // if(layId > 0 && _vGrid[layId-1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
                //     mtx[node_id][node_id] += via_condutance;
                //     mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                // }

                // // check bottom layer
                // if(layId < _db.numLayers()-1 && _vGrid[layId+1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
                //     mtx[node_id][node_id] += via_condutance;
                //     mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                // }

                for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
                    double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
                    double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
                    if (gridEnclose(grid_i, sX, sY)) {
                        // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
                        if (layId > 0) {
                            // mtx[node_id][node_id] += via_condutance;
                            // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                            vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                        } else {
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                            I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * via_condutance_up;
                        }
                        if (layId < _db.numLayers()-1) {
                            // mtx[node_id][node_id] += via_condutance;
                            // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                            vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
                        }
                    }
                }
                for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                    for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
                        double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
                        double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
                        if (gridEnclose(grid_i, tX, tY)) {
                            // cerr << "Enclose: net" << netId << " layer" << layId << " target" << tPortId << ", grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
                            if (layId > 0) {
                                // mtx[node_id][node_id] += via_condutance;
                                // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                                vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                            } else {
                                double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_up + 1.0/loadConductance)));
                            }
                            if (layId < _db.numLayers()-1) {
                                // mtx[node_id][node_id] += via_condutance;
                                // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                                vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
                            } 
                        }
                    }
                }
            }
        }

        Y.setFromTriplets(vTplY.begin(), vTplY.end());
        // Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IdentityPreconditioner> solver;
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
        // Eigen::SimplicialCholeskyLDLT<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
        // solver.setMaxIterations(100000);
        // solver.setTolerance(1e-15);
        solver.compute(Y);
        // V = solver.solveWithGuess(I, V);
        V = solver.solve(I);
        assert(solver.info() == Eigen::Success);

        // set voltage of each grid
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
                grid_i->setVoltage(V[node_id]);
                assert(grid_i->voltage() <= _db.vNet(netId)->sourcePort()->voltage());
            }
        }

        // set current of each grid
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                size_t xId = grid_i->xId();
                size_t yId = grid_i->yId();
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
                double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
                // double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
                double via_condutance_up, via_condutance_down;
                if (layId > 0) {
                    via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
                }
                if (layId < _db.numLayers() - 1) {
                    via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
                }
                if (gridId == 0) {
                    cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
                    cerr << ", via_conductance_down = " << via_condutance_down << endl;
                }
                double current = 0;
                size_t nbrId;
                if (legal(xId+1, yId)) {
                    if (_vGrid[layId][xId+1][yId]->netId() == netId) {
                        current += abs(grid_i->voltage() - _vGrid[layId][xId+1][yId]->voltage()) * g2g_condutance;
                    }
                }
                if (legal(xId-1, yId)) {
                    if (_vGrid[layId][xId-1][yId]->netId() == netId) {
                        current += abs(grid_i->voltage() - _vGrid[layId][xId-1][yId]->voltage()) * g2g_condutance;
                    }
                }
                if (legal(xId, yId+1)) {
                    if (_vGrid[layId][xId][yId+1]->netId() == netId) {
                        current += abs(grid_i->voltage() - _vGrid[layId][xId][yId+1]->voltage()) * g2g_condutance;
                    }
                }
                if (legal(xId, yId-1)) {
                    if (_vGrid[layId][xId][yId-1]->netId() == netId) {
                        current += abs(grid_i->voltage() - _vGrid[layId][xId][yId-1]->voltage()) * g2g_condutance;
                    }
                }
                // via current
                for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
                    double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
                    double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
                    if (gridEnclose(grid_i, sX, sY)) {
                        // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
                        if (layId > 0) {
                            current += abs(grid_i->voltage() - _vGrid[layId-1][xId][yId]->voltage()) * via_condutance_down;
                        } else {
                            current += abs(grid_i->voltage() - _db.vNet(netId)->sourcePort()->voltage()) * via_condutance_up;
                        }
                        if (layId < _db.numLayers()-1) {
                            current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
                        }
                    }
                }
                for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                    double tPortCurr = 0;
                    double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage()* _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
                    for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
                        double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
                        double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
                        if (gridEnclose(grid_i, tX, tY)) {
                            if (layId > 0) {
                                current += abs(grid_i->voltage() - _vGrid[layId-1][xId][yId]->voltage()) * via_condutance_down;
                            } else {
                                current += abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance);
                                _vTPortCurr[netId][tPortId] += abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance);
                                cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage();
                                cerr << ", current = " << abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance) << endl;
                            }
                            if (layId < _db.numLayers()-1) {
                                current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
                            } 
                        }
                    }
                    // _vTPortCurr[netId].push_back(tPortCurr);
                    // _vTPortVolt[netId].push_back(tPortCurr / loadConductance);
                    // if (_vTPortVolt[netId][tPortId] > _db.vNet(netId)->sourcePort()->voltage()) {
                    //     cerr << "ERROR: tPort voltage > sPort voltage !!!" << endl;
                    //     _vTPortVolt[netId][tPortId] = 0;
                    // }
                    // cerr << "tPortCurr = " << tPortCurr << ", tPortVolt = " << _vTPortVolt[netId][tPortId] << endl;
                }
                grid_i->setCurrent(current * 0.5);
                // cerr << "gridCurrent = " << current * 0.5 << endl;
                // if (grid_i->xId() == 25) {
                //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
                //     cerr << ", Voltage = " << grid_i->voltage() << endl;
                // }
                // if (grid_i->xId() == 26) {
                //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
                //     cerr << ", Voltage = " << grid_i->voltage() << endl;
                // }
                // if (grid_i->xId() == 27) {
                //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
                //     cerr << ", Voltage = " << grid_i->voltage() << endl;
                // }
                // if (grid_i->xId() == 28) {
                //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
                //     cerr << ", Voltage = " << grid_i->voltage() << endl;
                // }
                // cerr << "gridCurrent = " << current * 0.5 << endl;
            }
        }
        // for(int i=0; i<20; i++) {
        //     for(int j=0; j<20; j++)
        //         printf("%4.1f ", mtx[i][j]);
        //     printf("\n");
        // }
    }
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            double loadResistance = _db.vNet(netId)->targetPort(tPortId)->voltage() / _db.vNet(netId)->targetPort(tPortId)->current();
            cerr << "net" << netId << " tPort" << tPortId << ": current = " << _vTPortCurr[netId][tPortId];
            cerr << ", voltage = " << _vTPortCurr[netId][tPortId] * loadResistance << endl;
        }
    }
}

double DetailedMgr::getResistance(Grid* g1, Grid* g2) {
    return 0.0;
}

void DetailedMgr::check() {
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
                assert(_vNetGrid[netId][layId][gridId] != _vSGrid[netId][layId]);
                for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
                    assert(_vNetGrid[netId][layId][gridId] != _vTGrid[netId][layId][tPortId]);
                }
                for (size_t gridId1 = gridId+1; gridId1 < _vNetGrid[netId][layId].size(); ++ gridId1) {
                    assert(_vNetGrid[netId][layId][gridId] != _vNetGrid[netId][layId][gridId1]);
                }
            }
        }
    }
}