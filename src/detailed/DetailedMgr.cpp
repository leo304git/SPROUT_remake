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
                    // Shape* sBBox = _db.vNet(netId)->sourceViaCstr()->bBox();
                    Shape* sBBox = _db.vNet(netId)->sourcePort()->boundPolygon();
                    if (occupy(xId, yId, sBBox)) {
                        grid->setPort();
                        //only set port with one layer
                        if(layId == 0){
                            _vNetPortGrid[netId][0].push_back(make_pair(xId,yId));
                        }
                        if (fullyOccupy(xId, yId, sBBox)) {  // inner grids
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
                        // Shape* tBBox = _db.vNet(netId)->vTargetViaCstr(tPortId)->bBox();
                        Shape* tBBox = _db.vNet(netId)->targetPort(tPortId)->boundPolygon();
                        // if (netId == 2) {
                        //     vector< pair<double, double> > vVtx;
                        //     vVtx.push_back(make_pair(tBBox->minX(), tBBox->minY()));
                        //     vVtx.push_back(make_pair(tBBox->maxX(), tBBox->minY()));
                        //     vVtx.push_back(make_pair(tBBox->maxX(), tBBox->maxY()));
                        //     vVtx.push_back(make_pair(tBBox->minX(), tBBox->maxY()));
                        //     Polygon* p = new Polygon(vVtx, _plot);
                        //     tBBox = p;
                        // } 
                        if (occupy(xId, yId, tBBox)) {
                            grid->setPort();
                            if(layId == 0){
                                _vNetPortGrid[netId][tPortId+1].push_back(make_pair(xId,yId));
                            }
                            if (fullyOccupy(xId, yId, tBBox)) {  // inner grids
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
                            _plot.drawLine(x1, y1, x2, y2, SVGPlotColor::blue, layId, 0.5);
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
        // double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
        // cerr << "netId = " << netId << endl;
        // double ubCurr = _db.vNet(netId)->sourcePort()->voltage() * via_condutance;
        double ubCurr = 5;
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
    size_t layId = 0;
    // for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
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
    // }
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

bool DetailedMgr::SingleNetAStar(size_t netId, size_t layId) {

    bool CanRoute = false;
    
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
            CanRoute = true;
            //set ploted///////
            delete _Ploted[netId][layId][0];//because port 0 is source port
            _Ploted[netId][layId][0] = _Ploted[netId][layId][tPortId+1];
            ///////////////////
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
            //CanRoute = false;
            cerr << "BFS fail: lay" << layId << " net" << netId << " s->t" << tPortId << endl;
        }
        
        for (size_t tPortId1 = tPortId+1; tPortId1 < net->numTPorts(); ++ tPortId1) {
            AStarRouter tRouter(_vGrid[layId], _vTGrid[netId][layId][tPortId], _vTGrid[netId][layId][tPortId1], _gridWidth);
            if (tRouter.BFS()) {
                CanRoute = true;
                //set ploted///////
                delete _Ploted[netId][layId][tPortId1+1];
                _Ploted[netId][layId][tPortId1+1] = _Ploted[netId][layId][tPortId+1];
                ///////////////////
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
            } 
            else {
                //CanRoute = false;
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

    return CanRoute;  
}

void DetailedMgr::clearNet(size_t layId, size_t netId) {
    // for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
    //     _vNetGrid[netId][layId][gridId]->removeNet(netId);
    //     _vNetGrid[netId][layId][gridId]->decCongestCur();
    // }
    // _vNetGrid[netId][layId].clear();
}

void DetailedMgr::ResetAllNets(){
    for(size_t netId = 0; netId < _vNetGrid.size();++ netId ){
        for(size_t layId = 0; layId <  _vNetGrid[netId].size() ; ++ layId){
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
                _vNetGrid[netId][layId][gridId]->setNetId(-1);
                _vNetGrid[netId][layId][gridId]->setOccupied(0);
            }
            _vNetGrid[netId][layId].clear();
        }
    }
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
    cerr << "addViaGrid..." << endl;
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

vector< pair<double, double> > DetailedMgr::kMeansClustering(vector< pair<int,int> > vGrid, int numClusters, int numEpochs) {
    assert(numClusters <= vGrid.size());
    // if (numClusters == vGrid.size()) {
    //     return vGrid;
    // }
    struct Point {
        double x, y;     // coordinates
        int cluster;     // no default cluster
        double minDist;  // default infinite dist to nearest cluster
    };
    auto distance = [] (Point p1, Point p2) -> double {
        return pow((p1.x-p2.x), 2) + pow((p1.y-p2.y), 2);
    }; 

    vector<Point> points;
    for (size_t gridId = 0; gridId < vGrid.size(); ++ gridId) {
        Point p = {(vGrid[gridId].first+0.5)*_gridWidth, (vGrid[gridId].second+0.5)*_gridWidth, -1, numeric_limits<double>::max()};
        points.push_back(p);
    }

    vector<bool> isCentroid(points.size(), false);
    vector<Point> centroids;
    srand(time(0));  // need to set the random seed
    for (int i = 0; i < numClusters; ++i) {
        int pointId = rand() % points.size();
        while(isCentroid[pointId]) {
            pointId = rand() % points.size();
        }
        centroids.push_back(points.at(pointId));
        isCentroid[pointId] = true;
        // cerr << "centroid: (" << centroids[i].x << ", " << centroids[i].y << ")" << endl;
    }
    // for (int i = 0; i < numClusters; ++i) {
    //     centroids.push_back(points.at(i));
    // }

    // vector<int> nPoints(k,0);
    // vector<double> sumX(k,0.0);
    // vector<double> sumY(k,0.0);
    int* nPoints = new int[numClusters];
    double* sumX = new double[numClusters];
    double* sumY = new double[numClusters];
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

    // for (size_t centId = 0; centId < centroids.size(); ++ centId) {
    //     int centX = floor(centroids[centId].x);
    //     int centY = floor(centroids[centId].y);
    //     _db.addVia(centroids[centId].x, centroids[centId].y, )
    // }

    // for (size_t pointId = 0; pointId < points.size(); ++ pointId) {
    //     Point p = points[pointId];
    //     for (size_t tPortId = 0; tPortId < k; ++ tPortId) {
    //         if (p.cluster == tPortId) {
    //             // cerr << "tPortId = " << tPortId << endl;
    //             // cerr << "   node = " << p.node->name() << endl;
    //             _vTClusteredNode[netId][tPortId].push_back(p.node);
    //         }
    //     }
    // }

    vector< pair<double, double> > centPos;
    for (size_t centId = 0; centId < centroids.size(); ++ centId) {
        centPos.push_back(make_pair(centroids[centId].x, centroids[centId].y));
    }
    return centPos;
}

void DetailedMgr::addPortVia() {
    cerr << "Adding Vias to Each Port..." << endl;
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        Port* sPort = _db.vNet(netId)->sourcePort();
        int numSVias = _vNetPortGrid[netId][0].size()*0.25;
        assert(numSVias >= 1);
        cerr << "net" << netId << " sPort: numVias = " << numSVias << "; numGrids = " << _vNetPortGrid[netId][0].size() << endl;
        //cerr << "viaArea = " << sPort->viaArea() << ", metalArea = " <<  _db.VIA16D8A24()->metalArea() << endl;
        cerr << "capacity = " << _vNetPortGrid[netId][0].size() * _db.VIA16D8A24()->metalArea() << endl;
        vector< pair<double, double> > centPos = kMeansClustering(_vNetPortGrid[netId][0], numSVias, 100);
        // for(size_t gridId = 0; gridId < _vNetPortGrid[netId][0].size(); ++gridId){
        //     double x = (_vNetPortGrid[netId][0][gridId].first + 0.5)*_gridWidth;
        //     double y = (_vNetPortGrid[netId][0][gridId].second + 0.5)*_gridWidth;
        //     centPos.push_back(make_pair(x,y));
        // }
        assert(centPos.size() == numSVias);
        vector<size_t> vViaId(centPos.size(), 0);
        for (size_t viaId = 0; viaId < centPos.size(); ++ viaId) {
            vViaId[viaId] = _db.addVia(centPos[viaId].first, centPos[viaId].second, netId, ViaType::Source);
        }
        sPort->setViaCluster(_db.clusterVia(vViaId));
    
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            Port* tPort = _db.vNet(netId)->targetPort(tPortId);
            
            int numTVias = _vNetPortGrid[netId][tPortId+1].size()*0.25;
            assert(numTVias >= 1);
            cerr << "net" << netId << " tPort" << tPortId << ": numVias = " << numTVias << "; numGrids = " << _vNetPortGrid[netId][tPortId+1].size() << endl;
            vector< pair<double, double> > centPosT = kMeansClustering(_vNetPortGrid[netId][tPortId+1], numTVias, 100);

            // for(size_t gridId = 0; gridId < _vNetPortGrid[netId][tPortId+1].size(); ++gridId){
            //     double x = (_vNetPortGrid[netId][tPortId+1][gridId].first + 0.5)*_gridWidth;
            //     double y = (_vNetPortGrid[netId][tPortId+1][gridId].second + 0.5)*_gridWidth;
            //     centPosT.push_back(make_pair(x,y));
            // }

            assert(centPosT.size() == numTVias);
            vector<size_t> vViaIdT(centPosT.size(), 0);
            for (size_t viaIdT = 0; viaIdT < centPosT.size(); ++ viaIdT) {
                vViaIdT[viaIdT] = _db.addVia(centPosT[viaIdT].first, centPosT[viaIdT].second, netId, ViaType::Target);
            }
            tPort->setViaCluster(_db.clusterVia(vViaIdT));

        }
    }
}

// void DetailedMgr::buildMtx(size_t numLayers) {
//     // https://i.imgur.com/rIwlXJQ.png
//     // return an impedance matrix for each net
//     // number of nodes: \sum_{layId=0}^{_vNetGrid[netID].size()} _vNetGrid[netID][layId].size()

//     auto gridEnclose = [&] (Grid* grid, double x, double y) -> bool {
//         double gridLX = grid->xId() * _gridWidth;
//         double gridUX = (grid->xId()+1) * _gridWidth;
//         double gridLY = grid->yId() * _gridWidth;
//         double gridUY = (grid->yId()+1) * _gridWidth;
//         // to avoid a via enclosed by multiple grids, set ">=" but "<" only
//         return ((x >= gridLX) && (x < gridUX) && (y >= gridLY) && (y < gridUY));
//     };

//     for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
//         printf("netID: %d\n", netId);
        
//         size_t numNode = 0;
//         map< tuple<size_t, size_t, size_t>, size_t > getID; // i = getID[layID, xId, yId] = ith node
//         // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//         for (size_t layId = 0; layId < numLayers; ++layId) {
//             for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//                 getID[make_tuple(layId, _vNetGrid[netId][layId][gridId]->xId(), _vNetGrid[netId][layId][gridId]->yId())] = numNode;
//                 numNode++;
//             }
//         }
//         printf("numNode: %d\n", numNode);

//         // initialize matrix and vector
//         Eigen::SparseMatrix<double, Eigen::RowMajor> Y(numNode, numNode);
//         Eigen::VectorXd I(numNode);
//         Eigen::VectorXd V(numNode);
//         vector< Eigen::Triplet<double> > vTplY;
//         vTplY.reserve(6 * numNode);
//         for (size_t i = 0; i < numNode; ++ i) {
//             I[i] = 0;
//             V[i] = 0;
//         }

//         // // initialize
//         // vector< vector<double > > mtx;
//         // // numNode += (_db.vNet(netId)->numTPorts() + 1) * _db.numLayers();    // add the source/target via nodes on each layer
//         // for(int i=0; i<numNode; i++) {
//         //     mtx.push_back(vector<double>());
//         //     for(int j=0; j<numNode; j++)
//         //         mtx[i].push_back(0.0);
//         // }
//         // assert(_vNetGrid[netId].size() == _db.numLayers());
//         // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//         for (size_t layId = 0; layId < numLayers; ++layId) {
//             for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//                 Grid* grid_i = _vNetGrid[netId][layId][gridId];
//                 assert(grid_i->netId() == netId);
//                 // cerr << "grid = (" << grid_i->xId() << " " << grid_i->yId() << ")" << endl;
//                 size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
//             //    printf("x: %-4d, y: %-4d, lay: %-4d, ID: %-4d\n", i->xId(), i->yId(), layId, getID[make_tuple(layId, i->xId(), i->yId())]);
            
//                 double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
//                 // double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
//                 double via_condutance_up, via_condutance_down;
//                 if (layId > 0) {
//                     via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
//                 }
//                 // if (layId < _db.numLayers() - 1) {
//                 if (layId < numLayers-1) {
//                     via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
//                 }
//                 double ball_conductance = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(0)->thickness()+ _db.vMediumLayer(1)->thickness()+0.5* _db.vMetalLayer(1)->thickness()));
//                 double small_conductance = 1.0;

//                 // check left
//                 if(grid_i->xId() > 0 && _vGrid[layId][grid_i->xId()-1][grid_i->yId()]->netId() == netId) {
//                     // mtx[node_id][node_id] += g2g_condutance;
//                     // mtx[node_id][getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())]] -= g2g_condutance;
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())], -g2g_condutance));
//                 }

//                 // check right
//                 if(grid_i->xId() < _numXs-1 && _vGrid[layId][grid_i->xId()+1][grid_i->yId()]->netId() == netId) {
//                     // mtx[node_id][node_id] += g2g_condutance;
//                     // mtx[node_id][getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())]] -= g2g_condutance;
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())], -g2g_condutance));
//                 }
                
//                 // check down
//                 if(grid_i->yId() > 0 && _vGrid[layId][grid_i->xId()][grid_i->yId()-1]->netId() == netId) {
//                     // mtx[node_id][node_id] += g2g_condutance;
//                     // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)]] -= g2g_condutance;
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)], -g2g_condutance));
//                 }
                
//                 // check up
//                 if(grid_i->yId() < _numYs-1 && _vGrid[layId][grid_i->xId()][grid_i->yId()+1]->netId() == netId) {
//                     // mtx[node_id][node_id] += g2g_condutance;
//                     // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)]] -= g2g_condutance;
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                     vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)], -g2g_condutance));
//                 }

//                 // // check top layer
//                 // if(layId > 0 && _vGrid[layId-1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
//                 //     mtx[node_id][node_id] += via_condutance;
//                 //     mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                 // }

//                 // // check bottom layer
//                 // if(layId < _db.numLayers()-1 && _vGrid[layId+1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
//                 //     mtx[node_id][node_id] += via_condutance;
//                 //     mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                 // }
//                 if (_db.vNet(netId)->sourceViaCstr() != NULL)
//                 for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
//                     double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
//                     double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
//                     if (gridEnclose(grid_i, sX, sY)) {
//                         // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
//                         if (layId > 0) {
//                             // mtx[node_id][node_id] += via_condutance;
//                             // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
//                         } 
//                         // else {
//                         //     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
//                         //     I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * via_condutance_up;
//                         // }
//                         // if (layId < _db.numLayers()-1) {
//                         if (layId < numLayers-1) {
//                             // mtx[node_id][node_id] += via_condutance;
//                             // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
//                         }
//                     }
//                 }
//                 if (layId == 0) {
//                     for (size_t sNodeId = 0; sNodeId < _db.numClusteredNodes(netId, 0); ++ sNodeId) {
//                         double sX = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrX();
//                         double sY = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrY();
//                         if (gridEnclose(grid_i, sX, sY)) {
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, ball_conductance));
//                             I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * ball_conductance;
//                             //cerr << "layer" << layId << " node" << node_id;
//                             //cerr << ": sVolt = " << _db.vNet(netId)->sourcePort()->voltage();
//                             //cerr << ", ball_conductance = " << ball_conductance;
//                             //cerr << ", I" << node_id << " = " << I(node_id) << endl;
//                         }
//                     }
//                 }
//                 for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//                     if (_db.vNet(netId)->vTargetViaCstr(tPortId) != NULL)
//                     for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
//                         double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
//                         double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
//                         if (gridEnclose(grid_i, tX, tY)) {
//                             // cerr << "Enclose: net" << netId << " layer" << layId << " target" << tPortId << ", grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
//                             if (layId > 0) {
//                                 // mtx[node_id][node_id] += via_condutance;
//                                 // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                                 vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
//                                 vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
//                             } 
//                             // else {
//                             //     double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
//                             //     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_up + 1.0/loadConductance)));
//                             // }
//                             // if (layId < _db.numLayers()-1) {
//                             if (layId < numLayers-1) {
//                                 // mtx[node_id][node_id] += via_condutance;
//                                 // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                                 vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
//                                 vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
//                             } 
//                         }
//                     }
//                     if (layId == 0) {
//                         for (size_t tNodeId = 0; tNodeId < _db.numClusteredNodes(netId, tPortId+1); ++ tNodeId) {
//                             double tX = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrX();
//                             double tY = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrY();
//                             if (gridEnclose(grid_i, tX, tY)) {
//                                 double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.numClusteredNodes(netId, tPortId+1));
//                                 vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/ball_conductance + 1.0/loadConductance)));
//                                 //cerr << "layer" << layId << " node" << node_id;
//                                 //cerr << ": tPort" << tPortId ;
//                                 //cerr << ", total conductance = " << 1.0/(1.0/ball_conductance + 1.0/loadConductance) << endl;
//                                 // << ": via_conductance = " << via_condutance_up << ", loadConductance = " << loadConductance;
//                             }
//                         }
//                     }
//                 }
//             }
//         }

//         Y.setFromTriplets(vTplY.begin(), vTplY.end());
//         // Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IdentityPreconditioner> solver;
//         Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
//         // Eigen::SimplicialCholeskyLDLT<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
//         // solver.setMaxIterations(100000);
//         // solver.setTolerance(1e-15);
//         solver.compute(Y);
//         // V = solver.solveWithGuess(I, V);
//         V = solver.solve(I);
//         assert(solver.info() == Eigen::Success);

//         // set voltage of each grid
//         // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//         for (size_t layId = 0; layId < numLayers; ++layId) {
//             for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//                 Grid* grid_i = _vNetGrid[netId][layId][gridId];
//                 size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
//                 grid_i->setVoltage(V[node_id]);
//                 assert(grid_i->voltage() <= _db.vNet(netId)->sourcePort()->voltage());
//             }
//         }

//         // set current of each grid
//         for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//             _vTPortCurr[netId][tPortId] = 0.0;
//         }
//         // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//         for (size_t layId = 0; layId < numLayers; ++layId) {
//             for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//                 Grid* grid_i = _vNetGrid[netId][layId][gridId];
//                 size_t xId = grid_i->xId();
//                 size_t yId = grid_i->yId();
//                 size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
//                 double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
//                 // double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
//                 double via_condutance_up, via_condutance_down;
//                 if (layId > 0) {
//                     via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
//                 }
//                 // if (layId < _db.numLayers() - 1) {
//                 if (layId < numLayers-1) {
//                     via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
//                 }
//                 double ball_conductance = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(0)->thickness()+ _db.vMediumLayer(1)->thickness()+0.5* _db.vMetalLayer(1)->thickness()));
//                 if (gridId == 0) {
//                     //cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
//                     //cerr << ", via_conductance_down = " << via_condutance_down << endl;
//                 }
//                 double current = 0;
//                 size_t nbrId;
//                 if (legal(xId+1, yId)) {
//                     if (_vGrid[layId][xId+1][yId]->netId() == netId) {
//                         current += abs(grid_i->voltage() - _vGrid[layId][xId+1][yId]->voltage()) * g2g_condutance;
//                     }
//                 }
//                 if (legal(xId-1, yId)) {
//                     if (_vGrid[layId][xId-1][yId]->netId() == netId) {
//                         current += abs(grid_i->voltage() - _vGrid[layId][xId-1][yId]->voltage()) * g2g_condutance;
//                     }
//                 }
//                 if (legal(xId, yId+1)) {
//                     if (_vGrid[layId][xId][yId+1]->netId() == netId) {
//                         current += abs(grid_i->voltage() - _vGrid[layId][xId][yId+1]->voltage()) * g2g_condutance;
//                     }
//                 }
//                 if (legal(xId, yId-1)) {
//                     if (_vGrid[layId][xId][yId-1]->netId() == netId) {
//                         current += abs(grid_i->voltage() - _vGrid[layId][xId][yId-1]->voltage()) * g2g_condutance;
//                     }
//                 }
//                 // via current
//                 if (_db.vNet(netId)->sourceViaCstr() != NULL)
//                 for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
//                     double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
//                     double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
//                     if (gridEnclose(grid_i, sX, sY)) {
//                         // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
//                         if (layId > 0) {
//                             current += abs(grid_i->voltage() - _vGrid[layId-1][xId][yId]->voltage()) * via_condutance_down;
//                         } 
//                         // else {
//                         //     current += abs(grid_i->voltage() - _db.vNet(netId)->sourcePort()->voltage()) * via_condutance_up;
//                         // }
//                         // if (layId < _db.numLayers()-1) {
//                         if (layId < numLayers-1) {
//                             current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
//                         }
//                     }
//                 }
//                 if (layId == 0) {
//                     for (size_t sNodeId = 0; sNodeId < _db.numClusteredNodes(netId, 0); ++ sNodeId) {
//                         double sX = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrX();
//                         double sY = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrY();
//                         if (gridEnclose(grid_i, sX, sY)) {
//                             // vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, ball_conductance));
//                             // I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * ball_conductance;
//                             current += abs(grid_i->voltage() - _db.vNet(netId)->sourcePort()->voltage()) * ball_conductance;
//                         }
//                     }
//                 }
//                 for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//                     // double tPortCurr = 0;
//                     // double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage()* _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
//                     if (_db.vNet(netId)->vTargetViaCstr(tPortId) != NULL)
//                     for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
//                         double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
//                         double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
//                         if (gridEnclose(grid_i, tX, tY)) {
//                             if (layId > 0) {
//                                 current += abs(grid_i->voltage() - _vGrid[layId-1][xId][yId]->voltage()) * via_condutance_down;
//                             } 
//                             // else {
//                             //     current += abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance);
//                             //     _vTPortCurr[netId][tPortId] += abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance);
//                                 // cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage();
//                                 // cerr << ", current = " << abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance) << endl;
//                             // }
//                             // if (layId < _db.numLayers()-1) {
//                             if (layId < numLayers-1) {
//                                 current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
//                             } 
//                         }
//                     }
//                     if (layId == 0) {
//                         for (size_t tNodeId = 0; tNodeId < _db.numClusteredNodes(netId, tPortId+1); ++ tNodeId) {
//                             double tX = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrX();
//                             double tY = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrY();
//                             if (gridEnclose(grid_i, tX, tY)) {
//                                 double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.numClusteredNodes(netId, tPortId+1));
//                                 // vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/ball_conductance + 1.0/loadConductance)));
//                                 current += abs(grid_i->voltage()) /(1.0/ball_conductance + 1.0/loadConductance);
//                                 _vTPortCurr[netId][tPortId] += abs(grid_i->voltage()) /(1.0/ball_conductance + 1.0/loadConductance);
//                                 //cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage();
//                                 //cerr << ", current = " << abs(grid_i->voltage()) /(1.0/ball_conductance + 1.0/loadConductance) << endl;
//                             }
//                         }
//                     }
//                     // _vTPortCurr[netId].push_back(tPortCurr);
//                     // _vTPortVolt[netId].push_back(tPortCurr / loadConductance);
//                     // if (_vTPortVolt[netId][tPortId] > _db.vNet(netId)->sourcePort()->voltage()) {
//                     //     cerr << "ERROR: tPort voltage > sPort voltage !!!" << endl;
//                     //     _vTPortVolt[netId][tPortId] = 0;
//                     // }
//                     // cerr << "tPortCurr = " << tPortCurr << ", tPortVolt = " << _vTPortVolt[netId][tPortId] << endl;
//                 }
//                 grid_i->setCurrent(current * 0.5);
//                 // cerr << "gridCurrent = " << current * 0.5 << endl;
//                 // if (grid_i->xId() == 25) {
//                 //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//                 //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//                 // }
//                 // if (grid_i->xId() == 26) {
//                 //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//                 //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//                 // }
//                 // if (grid_i->xId() == 27) {
//                 //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//                 //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//                 // }
//                 // if (grid_i->xId() == 28) {
//                 //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//                 //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//                 // }
//                 // cerr << "gridCurrent = " << current * 0.5 << endl;
//             }
//         }
//         // for(int i=0; i<20; i++) {
//         //     for(int j=0; j<20; j++)
//         //         printf("%4.1f ", mtx[i][j]);
//         //     printf("\n");
//         // }
//     }
//     for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
//         for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//             double loadResistance = _db.vNet(netId)->targetPort(tPortId)->voltage() / _db.vNet(netId)->targetPort(tPortId)->current();
//             _vTPortVolt[netId][tPortId] = _vTPortCurr[netId][tPortId] * loadResistance;
//             cerr << "net" << netId << " tPort" << tPortId << ": current = " << _vTPortCurr[netId][tPortId];
//             cerr << ", voltage = " << _vTPortVolt[netId][tPortId] << endl;
//         }
//     }
// }

// void DetailedMgr::buildSingleNetMtx(size_t netId , size_t numLayers) {
//     // https://i.imgur.com/rIwlXJQ.png
//     // return an impedance matrix for each net
//     // number of nodes: \sum_{layId=0}^{_vNetGrid[netID].size()} _vNetGrid[netID][layId].size()

//     auto gridEnclose = [&] (Grid* grid, double x, double y) -> bool {
//         double gridLX = grid->xId() * _gridWidth;
//         double gridUX = (grid->xId()+1) * _gridWidth;
//         double gridLY = grid->yId() * _gridWidth;
//         double gridUY = (grid->yId()+1) * _gridWidth;
//         // to avoid a via enclosed by multiple grids, set ">=" but "<" only
//         return ((x >= gridLX) && (x < gridUX) && (y >= gridLY) && (y < gridUY));
//     };

    
//     printf("netID: %d\n", netId);
    
//     size_t numNode = 0;
//     map< tuple<size_t, size_t, size_t>, size_t > getID; // i = getID[layID, xId, yId] = ith node
//     // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//     for (size_t layId = 0; layId < numLayers; ++layId) {
//         for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//             getID[make_tuple(layId, _vNetGrid[netId][layId][gridId]->xId(), _vNetGrid[netId][layId][gridId]->yId())] = numNode;
//             numNode++;
//         }
//     }
//     printf("numNode: %d\n", numNode);

//     // initialize matrix and vector
//     Eigen::SparseMatrix<double, Eigen::RowMajor> Y(numNode, numNode);
//     Eigen::VectorXd I(numNode);
//     Eigen::VectorXd V(numNode);
//     vector< Eigen::Triplet<double> > vTplY;
//     vTplY.reserve(6 * numNode);
//     for (size_t i = 0; i < numNode; ++ i) {
//         I[i] = 0;
//         V[i] = 0;
//     }

//     // // initialize
//     // vector< vector<double > > mtx;
//     // // numNode += (_db.vNet(netId)->numTPorts() + 1) * _db.numLayers();    // add the source/target via nodes on each layer
//     // for(int i=0; i<numNode; i++) {
//     //     mtx.push_back(vector<double>());
//     //     for(int j=0; j<numNode; j++)
//     //         mtx[i].push_back(0.0);
//     // }
//     // assert(_vNetGrid[netId].size() == _db.numLayers());
//     // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//     for (size_t layId = 0; layId < numLayers; ++layId) {
//         for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//             Grid* grid_i = _vNetGrid[netId][layId][gridId];
//             assert(grid_i->netId() == netId);
//             // cerr << "grid = (" << grid_i->xId() << " " << grid_i->yId() << ")" << endl;
//             size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
//         //    printf("x: %-4d, y: %-4d, lay: %-4d, ID: %-4d\n", i->xId(), i->yId(), layId, getID[make_tuple(layId, i->xId(), i->yId())]);
        
//             double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
//             // double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
//             double via_condutance_up, via_condutance_down;
//             if (layId > 0) {
//                 via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
//             }
//             // if (layId < _db.numLayers() - 1) {
//             if (layId < numLayers-1) {
//                 via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
//             }
//             double ball_conductance = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(0)->thickness()+ _db.vMediumLayer(1)->thickness()+0.5* _db.vMetalLayer(1)->thickness()));
//             double small_conductance = 1.0;

//             // check left
//             if(grid_i->xId() > 0 && _vGrid[layId][grid_i->xId()-1][grid_i->yId()]->netId() == netId) {
//                 // mtx[node_id][node_id] += g2g_condutance;
//                 // mtx[node_id][getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())]] -= g2g_condutance;
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()-1, grid_i->yId())], -g2g_condutance));
//             }

//             // check right
//             if(grid_i->xId() < _numXs-1 && _vGrid[layId][grid_i->xId()+1][grid_i->yId()]->netId() == netId) {
//                 // mtx[node_id][node_id] += g2g_condutance;
//                 // mtx[node_id][getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())]] -= g2g_condutance;
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId()+1, grid_i->yId())], -g2g_condutance));
//             }
            
//             // check down
//             if(grid_i->yId() > 0 && _vGrid[layId][grid_i->xId()][grid_i->yId()-1]->netId() == netId) {
//                 // mtx[node_id][node_id] += g2g_condutance;
//                 // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)]] -= g2g_condutance;
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()-1)], -g2g_condutance));
//             }
            
//             // check up
//             if(grid_i->yId() < _numYs-1 && _vGrid[layId][grid_i->xId()][grid_i->yId()+1]->netId() == netId) {
//                 // mtx[node_id][node_id] += g2g_condutance;
//                 // mtx[node_id][getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)]] -= g2g_condutance;
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, g2g_condutance));
//                 vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId, grid_i->xId(), grid_i->yId()+1)], -g2g_condutance));
//             }

//             // // check top layer
//             // if(layId > 0 && _vGrid[layId-1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
//             //     mtx[node_id][node_id] += via_condutance;
//             //     mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//             // }

//             // // check bottom layer
//             // if(layId < _db.numLayers()-1 && _vGrid[layId+1][grid_i->xId()][grid_i->yId()]->hasNet(netId)) {
//             //     mtx[node_id][node_id] += via_condutance;
//             //     mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//             // }
//             if (_db.vNet(netId)->sourceViaCstr() != NULL)
//             for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
//                 double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
//                 double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
//                 if (gridEnclose(grid_i, sX, sY)) {
//                     // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
//                     if (layId > 0) {
//                         // mtx[node_id][node_id] += via_condutance;
//                         // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                         vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
//                         vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
//                     } 
//                     // else {
//                     //     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
//                     //     I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * via_condutance_up;
//                     // }
//                     // if (layId < _db.numLayers()-1) {
//                     if (layId < numLayers-1) {
//                         // mtx[node_id][node_id] += via_condutance;
//                         // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                         vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
//                         vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
//                     }
//                 }
//             }
//             if (layId == 0) {
//                 for (size_t sNodeId = 0; sNodeId < _db.numClusteredNodes(netId, 0); ++ sNodeId) {
//                     double sX = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrX();
//                     double sY = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrY();
//                     if (gridEnclose(grid_i, sX, sY)) {
//                         vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, ball_conductance));
//                         I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * ball_conductance;
//                         //cerr << "layer" << layId << " node" << node_id;
//                         //cerr << ": sVolt = " << _db.vNet(netId)->sourcePort()->voltage();
//                         //cerr << ", ball_conductance = " << ball_conductance;
//                         //cerr << ", I" << node_id << " = " << I(node_id) << endl;
//                     }
//                 }
//             }
//             for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//                 if (_db.vNet(netId)->vTargetViaCstr(tPortId) != NULL)
//                 for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
//                     double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
//                     double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
//                     if (gridEnclose(grid_i, tX, tY)) {
//                         // cerr << "Enclose: net" << netId << " layer" << layId << " target" << tPortId << ", grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
//                         if (layId > 0) {
//                             // mtx[node_id][node_id] += via_condutance;
//                             // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
//                         } 
//                         // else {
//                         //     double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
//                         //     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_up + 1.0/loadConductance)));
//                         // }
//                         // if (layId < _db.numLayers()-1) {
//                         if (layId < numLayers-1) {
//                             // mtx[node_id][node_id] += via_condutance;
//                             // mtx[node_id][getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId+1, grid_i->xId(), grid_i->yId())], -via_condutance_up));
//                         } 
//                     }
//                 }
//                 if (layId == 0) {
//                     for (size_t tNodeId = 0; tNodeId < _db.numClusteredNodes(netId, tPortId+1); ++ tNodeId) {
//                         double tX = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrX();
//                         double tY = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrY();
//                         if (gridEnclose(grid_i, tX, tY)) {
//                             double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.numClusteredNodes(netId, tPortId+1));
//                             vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/ball_conductance + 1.0/loadConductance)));
//                             //cerr << "layer" << layId << " node" << node_id;
//                             //cerr << ": tPort" << tPortId ;
//                             //cerr << ", total conductance = " << 1.0/(1.0/ball_conductance + 1.0/loadConductance) << endl;
//                             // << ": via_conductance = " << via_condutance_up << ", loadConductance = " << loadConductance;
//                         }
//                     }
//                 }
//             }
//         }
//     }

//     Y.setFromTriplets(vTplY.begin(), vTplY.end());
//     // Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IdentityPreconditioner> solver;
//     Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
//     // Eigen::SimplicialCholeskyLDLT<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
//     // solver.setMaxIterations(100000);
//     // solver.setTolerance(1e-15);
//     solver.compute(Y);
//     // V = solver.solveWithGuess(I, V);
//     V = solver.solve(I);
//     assert(solver.info() == Eigen::Success);

//     // set voltage of each grid
//     // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//     for (size_t layId = 0; layId < numLayers; ++layId) {
//         for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//             Grid* grid_i = _vNetGrid[netId][layId][gridId];
//             size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
//             grid_i->setVoltage(V[node_id]);
//             assert(grid_i->voltage() <= _db.vNet(netId)->sourcePort()->voltage());
//         }
//     }

//     // set current of each grid
//     for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//         _vTPortCurr[netId][tPortId] = 0.0;
//     }
//     // for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
//     for (size_t layId = 0; layId < numLayers; ++layId) {
//         for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
//             Grid* grid_i = _vNetGrid[netId][layId][gridId];
//             size_t xId = grid_i->xId();
//             size_t yId = grid_i->yId();
//             size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
//             double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
//             // double via_condutance = (_db.vMetalLayer(0)->conductivity() * _db.vVia(0)->shape()->area() * 1E-6) / (_db.vMediumLayer(0)->thickness() * 1E-3);
//             double via_condutance_up, via_condutance_down;
//             if (layId > 0) {
//                 via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
//             }
//             // if (layId < _db.numLayers() - 1) {
//             if (layId < numLayers-1) {
//                 via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
//             }
//             double ball_conductance = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(0)->thickness()+ _db.vMediumLayer(1)->thickness()+0.5* _db.vMetalLayer(1)->thickness()));
//             if (gridId == 0) {
//                 //cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
//                 //cerr << ", via_conductance_down = " << via_condutance_down << endl;
//             }
//             double current = 0;
//             size_t nbrId;
//             if (legal(xId+1, yId)) {
//                 if (_vGrid[layId][xId+1][yId]->netId() == netId) {
//                     current += abs(grid_i->voltage() - _vGrid[layId][xId+1][yId]->voltage()) * g2g_condutance;
//                 }
//             }
//             if (legal(xId-1, yId)) {
//                 if (_vGrid[layId][xId-1][yId]->netId() == netId) {
//                     current += abs(grid_i->voltage() - _vGrid[layId][xId-1][yId]->voltage()) * g2g_condutance;
//                 }
//             }
//             if (legal(xId, yId+1)) {
//                 if (_vGrid[layId][xId][yId+1]->netId() == netId) {
//                     current += abs(grid_i->voltage() - _vGrid[layId][xId][yId+1]->voltage()) * g2g_condutance;
//                 }
//             }
//             if (legal(xId, yId-1)) {
//                 if (_vGrid[layId][xId][yId-1]->netId() == netId) {
//                     current += abs(grid_i->voltage() - _vGrid[layId][xId][yId-1]->voltage()) * g2g_condutance;
//                 }
//             }
//             // via current
//             if (_db.vNet(netId)->sourceViaCstr() != NULL)
//             for (size_t sViaId = 0; sViaId < _db.vNet(netId)->sourceViaCstr()->numVias(); ++ sViaId) {
//                 double sX = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->x();
//                 double sY = _db.vNet(netId)->sourceViaCstr()->vVia(sViaId)->y();
//                 if (gridEnclose(grid_i, sX, sY)) {
//                     // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
//                     if (layId > 0) {
//                         current += abs(grid_i->voltage() - _vGrid[layId-1][xId][yId]->voltage()) * via_condutance_down;
//                     } 
//                     // else {
//                     //     current += abs(grid_i->voltage() - _db.vNet(netId)->sourcePort()->voltage()) * via_condutance_up;
//                     // }
//                     // if (layId < _db.numLayers()-1) {
//                     if (layId < numLayers-1) {
//                         current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
//                     }
//                 }
//             }
//             if (layId == 0) {
//                 for (size_t sNodeId = 0; sNodeId < _db.numClusteredNodes(netId, 0); ++ sNodeId) {
//                     double sX = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrX();
//                     double sY = _db.vClusteredNode(netId, 0, sNodeId)->node()->ctrY();
//                     if (gridEnclose(grid_i, sX, sY)) {
//                         // vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, ball_conductance));
//                         // I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * ball_conductance;
//                         current += abs(grid_i->voltage() - _db.vNet(netId)->sourcePort()->voltage()) * ball_conductance;
//                     }
//                 }
//             }
//             for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//                 // double tPortCurr = 0;
//                 // double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage()* _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
//                 if (_db.vNet(netId)->vTargetViaCstr(tPortId) != NULL)
//                 for (size_t tViaId = 0; tViaId < _db.vNet(netId)->vTargetViaCstr(tPortId)->numVias(); ++ tViaId) {
//                     double tX = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->x();
//                     double tY = _db.vNet(netId)->vTargetViaCstr(tPortId)->vVia(tViaId)->y();
//                     if (gridEnclose(grid_i, tX, tY)) {
//                         if (layId > 0) {
//                             current += abs(grid_i->voltage() - _vGrid[layId-1][xId][yId]->voltage()) * via_condutance_down;
//                         } 
//                         // else {
//                         //     current += abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance);
//                         //     _vTPortCurr[netId][tPortId] += abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance);
//                             // cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage();
//                             // cerr << ", current = " << abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance) << endl;
//                         // }
//                         // if (layId < _db.numLayers()-1) {
//                         if (layId < numLayers-1) {
//                             current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
//                         } 
//                     }
//                 }
//                 if (layId == 0) {
//                     for (size_t tNodeId = 0; tNodeId < _db.numClusteredNodes(netId, tPortId+1); ++ tNodeId) {
//                         double tX = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrX();
//                         double tY = _db.vClusteredNode(netId, tPortId+1, tNodeId)->node()->ctrY();
//                         if (gridEnclose(grid_i, tX, tY)) {
//                             double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.numClusteredNodes(netId, tPortId+1));
//                             // vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/ball_conductance + 1.0/loadConductance)));
//                             current += abs(grid_i->voltage()) /(1.0/ball_conductance + 1.0/loadConductance);
//                             _vTPortCurr[netId][tPortId] += abs(grid_i->voltage()) /(1.0/ball_conductance + 1.0/loadConductance);
//                             //cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage();
//                             //cerr << ", current = " << abs(grid_i->voltage()) /(1.0/ball_conductance + 1.0/loadConductance) << endl;
//                         }
//                     }
//                 }
//                 // _vTPortCurr[netId].push_back(tPortCurr);
//                 // _vTPortVolt[netId].push_back(tPortCurr / loadConductance);
//                 // if (_vTPortVolt[netId][tPortId] > _db.vNet(netId)->sourcePort()->voltage()) {
//                 //     cerr << "ERROR: tPort voltage > sPort voltage !!!" << endl;
//                 //     _vTPortVolt[netId][tPortId] = 0;
//                 // }
//                 // cerr << "tPortCurr = " << tPortCurr << ", tPortVolt = " << _vTPortVolt[netId][tPortId] << endl;
//             }
//             grid_i->setCurrent(current * 0.5);
//             // cerr << "gridCurrent = " << current * 0.5 << endl;
//             // if (grid_i->xId() == 25) {
//             //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//             //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//             // }
//             // if (grid_i->xId() == 26) {
//             //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//             //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//             // }
//             // if (grid_i->xId() == 27) {
//             //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//             //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//             // }
//             // if (grid_i->xId() == 28) {
//             //     cerr << "grid(" << layId << ", " << grid_i->xId() << ", " << grid_i->yId() << "): Current = " << current * 0.5;
//             //     cerr << ", Voltage = " << grid_i->voltage() << endl;
//             // }
//             // cerr << "gridCurrent = " << current * 0.5 << endl;
//         }
//     }
//     // for(int i=0; i<20; i++) {
//     //     for(int j=0; j<20; j++)
//     //         printf("%4.1f ", mtx[i][j]);
//     //     printf("\n");
//     // }
    
    
//     for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
//         double loadResistance = _db.vNet(netId)->targetPort(tPortId)->voltage() / _db.vNet(netId)->targetPort(tPortId)->current();
//         _vTPortVolt[netId][tPortId] = _vTPortCurr[netId][tPortId] * loadResistance;
//         cerr << "net" << netId << " tPort" << tPortId << ": current = " << _vTPortCurr[netId][tPortId];
//         cerr << ", voltage = " << _vTPortVolt[netId][tPortId] << endl;
//     }
    
// }

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
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
                assert(_vNetGrid[netId][layId][gridId]->netId() == netId);
                for (size_t gridId1 = gridId+1; gridId1 < _vNetGrid[netId][layId].size(); ++ gridId1) {
                    assert(_vNetGrid[netId][layId][gridId] != _vNetGrid[netId][layId][gridId1]);
                }
            }
        }
    }
    for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
        for (size_t xId = 0; xId <_numXs; ++ xId) {
            for (size_t yId = 0; yId < _numYs; ++ yId) {
                Grid* grid = _vGrid[layId][xId][yId];
                for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
                    if (grid->netId() == netId) {
                        bool inVNetGrid = false;
                        for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); ++ gridId) {
                            if (_vNetGrid[netId][layId][gridId] == grid) {
                                inVNetGrid = true;
                            }
                        }
                        assert(inVNetGrid);
                    }
                }
            }
        }
    }
}

void DetailedMgr::fillBoard(size_t layId, size_t netId, size_t xId, size_t yId) {



    // printf("Filling Board for the point of (%d, %d) of net %d in layer %d \n", xId, yId, netId, layId );
    // cout << "Index of the Inner Circle is now  " << indexOfInnerCircles << endl;
    // cout << "Net ID is  " << netId << endl; 
    Grid* grid = _vGrid[layId][xId][yId];
    // cout << "Grid Net ID is  " << grid -> netId() << endl;
    //It has already be set a number
    if(boardOfInnerCircles[xId][yId] != -1){
        return;
    }

    //It is obstalce
    if (grid->netId() == -1 && grid->occupied()) {
        return;
    }

    //This is a net of the searching net
    if (grid->netId() == netId){
        return;
    }

    //基本的判斷完了，現在開始查找其他人
    if (!grid->occupied() || (grid-> netId() != netId && grid ->netId() >= 0)) {
        //找的時候應該只要找他那格是-1，而且沒有被Occupied，或是NetId錯，我只填其他條Net或是空格

        boardOfInnerCircles[xId][yId] = indexOfInnerCircles;

        if(xId > 0)   fillBoard(layId, netId, xId-1, yId);
        if(xId < _numXs - 1)  fillBoard(layId, netId, xId+1, yId);
        if(yId > 0) fillBoard(layId, netId, xId, yId-1);
        if(yId < _numYs - 1) fillBoard(layId, netId, xId, yId+1);  

    }
    
}

//對某一個Layer的
void DetailedMgr::fillInnerCircle(size_t layId, size_t netId) {
    cout << "///////////////////////////////////" << endl;
    cout << "//Function: fillInnerCircle Start//" << endl;
    cout << "///////////////////////////////////" << endl;

    //We check the condition of each net in each layer, one by one.
    indexOfInnerCircles = 0;
    bool thisSearchHaveFinished = false;

    // _vNetGrid[netId][layId][netGridId]->setOccupied(true);
    cout << "The X number of is " << _numXs << endl;
    cout << "The Y number of is " << _numYs << endl;
    //可以用GridOccupied
    //grid->netId() == -1，這格是Obstacle或是空的
    //grid->occupied() == true 代表他是Obstacle
    //grid->occupied() == false 代表他是空的
    //grid->netId() == 特定數值，則他會是被這個netId 的 net佔領。
    boardOfInnerCircles.resize(_numXs);
    cout << "Dealing with layer  " << layId << endl;
    //Reset the temp board as the ones in this layer
    for (size_t xId = 0; xId < _numXs; ++ xId) {
        boardOfInnerCircles[xId].resize(_numYs);
        for (size_t yId = 0; yId < _numYs; ++ yId) {
            // -1 代表初始狀況
            // 0 代表最外圈
            // 1以上的整數代表開始進入內圈
            boardOfInnerCircles[xId][yId] = -1;
        }
    }

    for (size_t xId = 0; xId < _numXs; ++ xId) {
        for (size_t yId = 0; yId < _numYs; ++ yId) {
            Grid* grid = _vGrid[layId][xId][yId];
        
            if (grid->netId() == -1 && !grid->occupied() && boardOfInnerCircles[xId][yId] == -1){
                thisSearchHaveFinished = false;
                fillBoard(layId, netId, xId, yId);

                // cout << "I really got here" << endl;
                ++ indexOfInnerCircles;
                
            }
            else if (grid-> netId() != netId && grid ->netId() >= 0 && boardOfInnerCircles[xId][yId] == -1){
                thisSearchHaveFinished = false;
                fillBoard(layId, netId, xId, yId);
                cout << "I really got here" << endl;
                ++ indexOfInnerCircles;
                
            }
        }
    }

    for (size_t xId = 0; xId < _numXs; ++ xId) {
        for (size_t yId = 0; yId < _numYs; ++ yId) {
            Grid* grid = _vGrid[layId][xId][yId];
            if(boardOfInnerCircles[xId][yId] > 0 && !grid->occupied() ){
                // cout <<" " << boardOfInnerCircles[xId][yId] << " ";
                // cout << boardOfInnerCircles[xId][yId] << endl;
                grid->setNetId(netId);
                grid->setOccupied(1);
                _vNetGrid[netId][layId].push_back(grid);
            }
            // cout << endl;
            // printf("Point of (%d, %d), board value is %d \n", xId, yId, boardOfInnerCircles[xId][yId]);

        }
    }
    
    cout << endl;
    indexOfInnerCircles ++;
    cout << indexOfInnerCircles << endl;

    cout << "///////////////////////////////////" << endl;
    cout << "//Function: fillInnerCircle Ends///" << endl;
    cout << "///////////////////////////////////" << endl;
}

//function for current sortint
bool compareByCurrent(const std::pair<double, int>& a, const std::pair<double, int>& b) {
    return a.first < b.first;
}



size_t DetailedMgr::SmartGrow(size_t layId, size_t netId, int k){
    //cout << "###########Smart GROW###########" << endl;
    //cout << "size of NetGrid after smartgrow : " << _vNetGrid[netId][layId].size() << endl;  


    vector<int> Candidate; // store new node's gridID
    size_t InitialSize = _vNetGrid[netId][layId].size();

    for(size_t gridId = 0; gridId < InitialSize ; gridId++){
        Grid* grid = _vNetGrid[netId][layId][gridId];
        size_t xId = grid->xId();
        size_t yId = grid->yId();

        if (legal(xId+1, yId)) {
            Grid* rGrid = _vGrid[layId][xId+1][yId];
            if (!rGrid->occupied()) {
                int GridID = _vNetGrid[netId][layId].size();
                _vNetGrid[netId][layId].push_back(rGrid);
                Candidate.push_back(GridID);
                rGrid->setNetId(netId);
                rGrid->setOccupied(1);
            }
        }
        if (legal(xId-1, yId)) {
            Grid* lGrid = _vGrid[layId][xId-1][yId];
            if (!lGrid->occupied()) {
                int GridID = _vNetGrid[netId][layId].size();
                _vNetGrid[netId][layId].push_back(lGrid);
                Candidate.push_back(GridID);
                lGrid->setNetId(netId);
                lGrid->setOccupied(1);
            }
        }
        if (legal(xId, yId+1)) {
            Grid* uGrid = _vGrid[layId][xId][yId+1];
            if (!uGrid->occupied()) {
                int GridID = _vNetGrid[netId][layId].size();
                _vNetGrid[netId][layId].push_back(uGrid);
                Candidate.push_back(GridID);
                uGrid->setNetId(netId);
                uGrid->setOccupied(1);
                
            }
        }
        if (legal(xId, yId-1)) {
            Grid* dGrid = _vGrid[layId][xId][yId-1];
            if (!dGrid->occupied()) {
                int GridID = _vNetGrid[netId][layId].size();
                _vNetGrid[netId][layId].push_back(dGrid);
                Candidate.push_back(GridID);
                dGrid->setNetId(netId);
                dGrid->setOccupied(1);
            }
        }
    }

    //cout << "size of adding neighbor : " << _vNetGrid[netId][layId].size()<<endl;

    //only keep k nodes and remove other nodes
    int removeNum = Candidate.size() - k;
    int alreadyRemove = 0;

    if(removeNum > 0){
        //DO PEEC CURRENT SIMULATION
        buildSingleNetMtx(netId);

        vector<pair<double,int>> NodeCurrent;

        for(size_t i = 0; i < Candidate.size();i++){
            int GridID = Candidate[i];
            Grid* grid = _vNetGrid[netId][layId][GridID];
            double current = grid->current(); ///*_vTPortCurr[netId][GridID];*/   /*PEEC GridID CURRENT*/
            NodeCurrent.push_back(make_pair(current,GridID));
        }

        //sort 
        std::sort(NodeCurrent.begin(), NodeCurrent.end(), compareByCurrent);

        Grid* r = new Grid(-1,-1);//new a pointer for later remove operation
        for(int i = 0; alreadyRemove < removeNum; i++){
            
            int gridId = NodeCurrent[i].second;
            Grid* grid = _vNetGrid[netId][layId][gridId];
            
            grid->setNetId(-1);//remove it from net
            grid->setOccupied(0);
            _vNetGrid[netId][layId][gridId] = r; //set pointer to r and delete later (avoid changing the size if vNetGrid[netId][layId])
            //cout << "Remove GridID : " << gridId << " ";
            //cout << "Remove current : " << NodeCurrent[i].first<<endl;
            alreadyRemove ++;
            
        }

        //delete removed grid
        _vNetGrid[netId][layId].erase(std::remove(_vNetGrid[netId][layId].begin(),_vNetGrid[netId][layId].end(), r), _vNetGrid[netId][layId].end());
        delete r;
    }   

    if(k < Candidate.size()) return k;
    else return Candidate.size();

    //cout << "size of NetGrid after smartgrow : " << _vNetGrid[netId][layId].size() << endl;  

}

void DetailedMgr::SmartRefine(size_t layId, size_t netId, int k){

    cout << "###########Smart Refine###########" << endl;

    buildSingleNetMtx(netId);

    vector<pair<double,int>> NodeCurrent;

    for(size_t gridId = 0; gridId < _vNetGrid[netId][layId].size() ; gridId++){
        Grid* grid = _vNetGrid[netId][layId][gridId];
        double current = grid->current();/*PEEC GridID CURRENT*/
        NodeCurrent.push_back(make_pair(current,gridId));
    }

    //sort 
    std::sort(NodeCurrent.begin(), NodeCurrent.end(), compareByCurrent);

    int alreadyRemoved = 0;

    //remove k nodes
    Grid* r = new Grid(-1,-1);//new a pointer for later remove operation
    for(int i = 0; i < NodeCurrent.size() && alreadyRemoved < k ; i++){
        int gridId = NodeCurrent[i].second;
        bool CanRemove = false;

        Grid* grid = _vNetGrid[netId][layId][gridId];

        if(grid->IsPort()) continue;

        for(size_t j = 0; j < grid->numNeighbors();j++ ){
            Grid* nGrid = grid->vNeighbor(j);
            if(nGrid->netId() != grid->netId()){
                CanRemove = true;
                break;
            }
        }

        if(CanRemove){
            grid->setNetId(-1);//remove it from net
            grid->setOccupied(0);
            _vNetGrid[netId][layId][gridId] = r; //set pointer to r and delete later (avoid changing the size if vNetGrid[netId][layId])
            //cout << "Remove GridID : " << gridId << " ";
            //cout << "Remove current : " << NodeCurrent[i].first<<endl;
            alreadyRemoved ++;
        }
        
    }

    //delete removed grid
    _vNetGrid[netId][layId].erase(std::remove(_vNetGrid[netId][layId].begin(),_vNetGrid[netId][layId].end(), r), _vNetGrid[netId][layId].end());
    delete r;

    //do SmartGrow for k nodes
    SmartGrow(layId, netId, alreadyRemoved);
}


//function for layer sortint
bool compareByThickness(const std::pair<double, int>& a, const std::pair<double, int>& b) {
    return a.first > b.first;
}

void DetailedMgr::SPROUT(){
    
    cout << "####SPROUT####"<<endl;
    synchronize();
    vector<double> target; // target[netId]

    int targetsize = 0;
    

    for(size_t netId = 0; netId < _vNetGrid.size(); ++netId){
        targetsize = 6 * _vNetGrid[netId][0].size();
        cout << "NET " << netId << " size : " << targetsize << endl; 
        target.push_back(targetsize);
    }

    /////////////////routing layer order assign////////////////
    vector<pair<double,int>> RLayer;
    for(int layId = 0; layId < _vNetGrid[0].size(); layId ++){
        pair<double,int> temp = make_pair(_db.vMetalLayer(layId)->thickness(),layId);
        RLayer.push_back(temp);
    }
    std::sort(RLayer.begin(), RLayer.end(), compareByThickness);
    cout << "order : "<<endl;
    for(size_t i = 0; i < RLayer.size();++i){
        cout <<  RLayer[i].second << " thickness = " << RLayer[i].first << endl; 
    }

    cout << endl;
    ///////////////////////////////////////////////////////////
        
    /////////////////Initialize HasAStar///////////////////////
    vector<vector<bool>> HasAStar; //[netId][layId]

    for(size_t netId = 0; netId < _vNetGrid.size(); ++netId){
        vector<bool> temp;
        for(size_t layId = 0; layId < _vNetGrid[netId].size();++layId){
            temp.push_back(false);
        }
        HasAStar.push_back(temp);
    }
    ////////////////////////////////////////////////////////////

    //////////////////Initialize some parameter/////////////////
    bool ReachTarget = false;
    size_t netId = 0;
    bool canGrow = true;
    //int count = 0;
    ////////////////////////////////////////////////////////////


    //for loop for each layer and net
    while(netId < _vNetGrid.size()){
        cout << "##########NET " << netId << " DO SMARTGROW##########" << endl;
        int Area = 0;
        for(size_t layId = 0; layId < _vNetGrid[netId].size();++layId) Area += _vNetGrid[netId][layId].size();
        int k = Area/5;

        for(size_t i = 0; i < RLayer.size();++i){
           
            size_t layId = RLayer[i].second;
            //cout << "#########LAYER " << layId << " ##########" << endl;

            canGrow = true;
            //SmartGrow stage
            //If area constraint is not satisfied and this layer didn't do astar before -> do astar
            if(!HasAStar[netId][layId] && Area < target[netId]){
                //do astar
                bool CanRoute = SingleNetAStar(netId,layId);
                if(!CanRoute){
                    continue; // this layer can't route, go to next layer
                }
                else{
                    HasAStar[netId][layId] = true;
                    synchronize(); // 同步vGrid and vNetGrid
                    buildSingleNetMtx(netId);
                }
            }

            //smartgrow stage
            while(Area < target[netId] && HasAStar[netId][layId]){
                size_t grownum = SmartGrow(layId,netId,k);
                //k = (int)(k/1.1);//隨便設一個遞減函數
                Area += grownum;
                //cout << "GROW " << grownum << endl;
                if(grownum == 0){
                    cout << "NO GROW" << endl;
                    canGrow = false;
                    break;
                }
            }

            //Refine stage
            // bool DoRefine = false;
            // for(size_t tPortId = 0; tPortId < _vTPortCurr[netId].size();tPortId++){
            //     if(_vTPortCurr[netId][tPortId] < _db.vNet(netId)->targetPort(tPortId)->current()){
            //         double Error = (_db.vNet(netId)->targetPort(tPortId)->current() - _vTPortCurr[netId][tPortId]) / _db.vNet(netId)->targetPort(tPortId)->current();
            //         if(Error < 0.05) DoRefine  = true;
            //         break;
            //     }
            //     if(_vTPortVolt[netId][tPortId] < _db.vNet(netId)->targetPort(tPortId)->voltage()){
            //         double Error = (_db.vNet(netId)->targetPort(tPortId)->voltage() - _vTPortVolt[netId][tPortId]) / (_db.vNet(netId)->targetPort(tPortId)->voltage());
            //         if(Error < 0.05) DoRefine = true;
            //         break;
            //     }
            // }

            // if(DoRefine && canGrow){
            //     int r = Area/50;
            //     //One iteration do 3 times refine
            //     for(int i = 0 ; i < 3 ; i++){
            //         SmartRefine(layId,netId,r);
            //     }
            // }
            
        }

        //simulation stage
        ReachTarget = true;
        buildSingleNetMtx(netId);

        if(!canGrow) break;

        //if not meet impedance constraint, augment the area upper bound
        for(size_t tPortId = 0; tPortId < _vTPortCurr[netId].size();tPortId++){
            if(_vTPortCurr[netId][tPortId] < _db.vNet(netId)->targetPort(tPortId)->current()){
                // double Error = ((_db.vNet(netId)->targetPort(tPortId)->current() - _vTPortCurr[netId][tPortId]) / _db.vNet(netId)->targetPort(tPortId)->current());
                // if(Error >= 0.01){
                //     ReachTarget = false;
                //     target[netId] *= 1.2;
                //     break;
                // }
                ReachTarget = false;
                target[netId] *= 1.2;
                break;
            }
            if(_vTPortVolt[netId][tPortId] < _db.vNet(netId)->targetPort(tPortId)->voltage()){
                // double Error = ((_db.vNet(netId)->targetPort(tPortId)->voltage() - _vTPortVolt[netId][tPortId]) / (_db.vNet(netId)->targetPort(tPortId)->voltage()));
                // if(Error >= 0.01){
                //     ReachTarget = false;
                //     target[netId] *= 1.2;
                //     break;
                // }
                ReachTarget = false;
                target[netId] *= 1.2;
                break;
            }
        }
        
        if(ReachTarget) {
            cout << "#### NET "<< netId << " FINISH####" << endl;
            cout << "Target AREA : " << target[netId] << endl;
            netId++;
        }
    }

    bool AllReach = true;
    for(size_t netId = 0; netId < _vNetGrid.size(); ++ netId){
        for(size_t tPortId = 0; tPortId < _vTPortCurr[netId].size();tPortId++){
            if(_vTPortCurr[netId][tPortId] < _db.vNet(netId)->targetPort(tPortId)->current()){
                // double Error = (_db.vNet(netId)->targetPort(tPortId)->current() - _vTPortCurr[netId][tPortId]) / _db.vNet(netId)->targetPort(tPortId)->current();
                // if(Error >= 0.01) AllReach = false;
                // break;
                AllReach = false;
                break;
            }
            if(_vTPortVolt[netId][tPortId] < _db.vNet(netId)->targetPort(tPortId)->voltage()){
                // double Error = (_db.vNet(netId)->targetPort(tPortId)->voltage() - _vTPortVolt[netId][tPortId]) / (_db.vNet(netId)->targetPort(tPortId)->voltage());
                // if(Error >= 0.01) AllReach = false;
                // break;
                AllReach = false;
                break;
            }
        }
    }


    if(AllReach){
        cout << "###REACH TARGET###"<< endl;
        cout << "ALL NET AREA UPPER BOUND IS : " << endl;
        for(size_t netId = 0; netId < _vNetGrid.size(); ++netId){
            cout << "NETID : " << netId << " AREA : " << target[netId] << endl;
        }
    }
    else{
        cout << "###CANT REACH TARGET###"<<endl;
        cout << "ALL NET AREA UPPER BOUND IS : " << endl;
        for(size_t netId = 0; netId < _vNetGrid.size(); ++netId){
            cout << "NETID : " << netId << " AREA : " << target[netId] << endl;
        }
    }
}

void DetailedMgr::writeColorMap_v2(const char* path, bool isVoltage) {

    vector<double> valList;
    valList.clear();
    for(int layId=0; layId<_db.numLayers(); layId++) {
        for(int netId=0; netId<_db.numNets(); netId++) {
            for(int gridId=0; gridId<_vNetGrid[netId][layId].size(); gridId++) {
                int x = _vNetGrid[netId][layId][gridId]->xId(), y = _vNetGrid[netId][layId][gridId]->yId();
                valList.push_back(isVoltage? _vGrid[layId][x][y]->voltage(): _vGrid[layId][x][y]->current());
            }
        }
    }
    std::sort(valList.begin(), valList.end());

    

    FILE *fp = fopen(path, "w");

    fprintf(fp, "%d\n", _db.numNets());
    fprintf(fp, "%d\n", _db.numLayers());
    fprintf(fp, "%d\n", _numXs);
    fprintf(fp, "%d\n", _numYs);
    
    std::sort(valList.begin(), valList.end());
    fprintf(fp, "%16.10f %16.10f\n", valList[(int)(0.01*valList.size())], valList[(int)(0.99*valList.size())]);

    fprintf(fp, "%d\n\n", _db.numVias());
    for(int i=0; i<_db.numVias(); i++)
        fprintf(fp, "%13.6f %13.6f %13.6f\n", (double)_db.vVia(i)->shape()->ctrX() / _gridWidth, (double)_db.vVia(i)->shape()->ctrY() / _gridWidth, (double)_db.vVia(i)->drillRadius() / _gridWidth);
    
    fprintf(fp, "\n");

    for(int layId=0; layId<_db.numLayers(); layId++) {

        for(int netId=0; netId<_db.numNets(); netId++) {
            
            // fprintf(fp, "lay: %d, net: %d\n", layId, netId);
            fprintf(fp, "%d\n", _vNetGrid[netId][layId].size());
            for(int gridId=0; gridId<_vNetGrid[netId][layId].size(); gridId++) {
                int x = _vNetGrid[netId][layId][gridId]->xId(), y = _vNetGrid[netId][layId][gridId]->yId();
                fprintf(fp, "%4d %4d %18.12f\n", x, y, isVoltage? _vGrid[layId][x][y]->voltage(): _vGrid[layId][x][y]->current());
            }
            fprintf(fp, "\n");
        }
    }

    printf("--- finish write color map ---\n");
    fclose(fp);
}

void DetailedMgr::RemoveIsolatedGrid(){
    for(size_t netId=0; netId < _vNetGrid.size(); netId++){
        for(size_t layId=0; layId<_vNetGrid[netId].size(); layId++){
            Grid* r = new Grid(-1,-1);
            for(size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId++){
                int Remove = 0;
                Grid* grid = _vNetGrid[netId][layId][gridId];
                int xId = grid->xId();
                int yId = grid->yId();
            
                if (legal(xId+1, yId)) {
                    Grid* rGrid = _vGrid[layId][xId+1][yId];
                    if (rGrid->netId() == netId) {
                        Remove += 1;
                    }
                }
                if (legal(xId-1, yId)) {
                    Grid* lGrid = _vGrid[layId][xId-1][yId];
                    if (lGrid->netId() == netId) {
                        Remove += 1;
                    }
                }
                if (legal(xId, yId+1)) {
                    Grid* uGrid = _vGrid[layId][xId][yId+1];
                    if (uGrid->netId() == netId) {
                        Remove += 1;
                    }
                }
                if (legal(xId, yId-1)) {
                    Grid* dGrid = _vGrid[layId][xId][yId-1];
                    if (dGrid->netId() == netId) {
                        Remove += 1;
                    }
                }
                //Remove
                if(Remove < 2){
                    grid->setNetId(-1);//remove it from net
                    grid->setOccupied(0);
                    _vNetGrid[netId][layId][gridId] = r; 
                }
            }
            for(size_t layId = 0; layId < _vNetGrid[netId].size();layId ++){
                _vNetGrid[netId][layId].erase(std::remove(_vNetGrid[netId][layId].begin(),_vNetGrid[netId][layId].end(), r), _vNetGrid[netId][layId].end());
            }
            delete r;
        }
    }
}

void DetailedMgr::buildMtx() {
    cerr << "PEEC Simulation start..." << endl;
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
            int numSVias = 0;
            vector<int> numTVias(_db.vNet(netId)->numTPorts(), 0);
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                // cerr << "grid = (" << grid_i->xId() << " " << grid_i->yId() << ")" << endl;
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
            //    printf("x: %-4d, y: %-4d, lay: %-4d, ID: %-4d\n", i->xId(), i->yId(), layId, getID[make_tuple(layId, i->xId(), i->yId())]);
            
                double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
                double via_condutance_up, via_condutance_down;
                if (layId > 0) {
                    via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
                }
                if (layId < _db.numLayers() - 1) {
                    via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
                }
                double small_conductance = 1.0;
                // if (gridId == 0) {
                //     cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
                //     cerr << ", via_conductance_down = " << via_condutance_down << endl;
                // }

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
                        numSVias ++;
                        // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
                        if (layId > 0) {
                            // mtx[node_id][node_id] += via_condutance;
                            // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                            vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                        } else {
                            // if (netId != 1) {
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                            I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * via_condutance_up;
                            // }
                            // cerr << "layer" << layId << " node" << node_id;
                            // cerr << ": sVolt = " << _db.vNet(netId)->sourcePort()->voltage();
                            // cerr << ", via_conductance_up = " << via_condutance_up;
                            // cerr << ", I" << node_id << " = " << I(node_id) << endl;
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
                            numTVias[tPortId] ++;
                            if (layId > 0) {
                                // mtx[node_id][node_id] += via_condutance;
                                // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                                vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                            } else {
                                double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
                                //  * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias()
                                // if (netId != 1) {
                                vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_up + 1.0/loadConductance)));
                                // } else {
                                //     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_down + 1.0/loadConductance)));
                                // }
                                // cerr << "layer" << layId << " node" << node_id;
                                // cerr << ": tPort" << tPortId ;
                                // cerr << ", total conductance = " << 1.0/(1.0/via_condutance_up + 1.0/loadConductance) << endl;
                                // << ": via_conductance = " << via_condutance_up << ", loadConductance = " << loadConductance;
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
            // Port* sPort = _db.vNet(netId)->sourcePort();
            // int realNumSVias = sPort->viaCluster()->numVias();
            // cerr << "layer" << layId << ": numSVias = " << numSVias << ", real numSVias = " << realNumSVias << endl;
            // for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            //     Port* tPort = _db.vNet(netId)->targetPort(tPortId);
            //     int realNumTVias = tPort->viaCluster()->numVias();
            //     cerr << "layer" << layId << " tPort" << tPortId << ": numTVias = " << numTVias[tPortId] << ", real numTVias = " << realNumTVias << endl;
            // }
        }

        // if (netId == 1) {
        //     cerr << "net1, I = " << endl;
        //     for (size_t i = 0; i < I.size(); ++ i) {
        //         if (I[i] > 0) {
        //             cerr << "I[" << i << "] = " << I[i] << endl;
        //         }
        //     }
        // }

        Y.setFromTriplets(vTplY.begin(), vTplY.end());
        // Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IdentityPreconditioner> solver;
        Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
        // solver.setMaxIterations(1000000);
        // solver.setTolerance(1e-14);
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
                // assert(grid_i->voltage(netId) <= _db.vNet(netId)->sourcePort()->voltage());
            }
        }

        // set current of each grid
        for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
            _vTPortCurr[netId][tPortId] = 0.0;
        }
        assert(_vNetGrid[netId].size() == _db.numLayers());
        for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid_i = _vNetGrid[netId][layId][gridId];
                size_t xId = grid_i->xId();
                size_t yId = grid_i->yId();
                size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
                double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
                double via_condutance_up, via_condutance_down;
                if (layId > 0) {
                    via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
                }
                if (layId < _db.numLayers() - 1) {
                    via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
                }
                if (gridId == 0) {
                    // cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
                    // cerr << ", via_conductance_down = " << via_condutance_down << endl;
                }
                double small_conductance = 1.0;
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
                    // double tPortCurr = 0;
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
                                //cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage();
                                //cerr << ", current = " << abs(grid_i->voltage()) /(1.0/via_condutance_up + 1.0/loadConductance) << endl;
                            }
                            if (layId < _db.numLayers()-1) {
                                current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
                            } 
                        }
                    }
                    // _vTPortCurr[netId][tPortId] += tPortCurr;
                    // _vTPortCurr[netId].push_back(tPortCurr);
                    // _vTPortVolt[netId].push_back(tPortCurr / loadConductance);
                    // cerr << "tPortCurr = " << tPortCurr << ", tPortVolt = " << _vTPortVolt[netId][tPortId] << endl;
                }
                grid_i->setCurrent(current * 0.5);
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
            _vTPortVolt[netId][tPortId] = _vTPortCurr[netId][tPortId] * loadResistance;
            cerr << "net" << netId << " tPort" << tPortId << ": current = " << _vTPortCurr[netId][tPortId];
            cerr << ", voltage = " << _vTPortVolt[netId][tPortId] << endl;
        }
    }
}

void DetailedMgr::buildSingleNetMtx(size_t netId) {
    cerr << "Single Net PEEC Simulation start..." << endl;

    auto gridEnclose = [&] (Grid* grid, double x, double y) -> bool {
        double gridLX = grid->xId() * _gridWidth;
        double gridUX = (grid->xId()+1) * _gridWidth;
        double gridLY = grid->yId() * _gridWidth;
        double gridUY = (grid->yId()+1) * _gridWidth;
        // to avoid a via enclosed by multiple grids, set ">=" but "<" only
        return ((x >= gridLX) && (x < gridUX) && (y >= gridLY) && (y < gridUY));
    };
    
    //wait for the answer
    size_t numNode = 0;
    map< tuple<size_t, size_t, size_t>, size_t > getID; // i = getID[layID, xId, yId] = ith node
    for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
        for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
            getID[make_tuple(layId, _vNetGrid[netId][layId][gridId]->xId(), _vNetGrid[netId][layId][gridId]->yId())] = numNode;
            numNode++;
        }
    }

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
        int numSVias = 0;
        vector<int> numTVias(_db.vNet(netId)->numTPorts(), 0);
        for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
            Grid* grid_i = _vNetGrid[netId][layId][gridId];
            // cerr << "grid = (" << grid_i->xId() << " " << grid_i->yId() << ")" << endl;
            size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
        //    printf("x: %-4d, y: %-4d, lay: %-4d, ID: %-4d\n", i->xId(), i->yId(), layId, getID[make_tuple(layId, i->xId(), i->yId())]);
        
            double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
            double via_condutance_up, via_condutance_down;
            if (layId > 0) {
                via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
            }
            if (layId < _db.numLayers() - 1) {
                via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
            }
            double small_conductance = 1.0;
            // if (gridId == 0) {
            //     cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
            //     cerr << ", via_conductance_down = " << via_condutance_down << endl;
            // }

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
                    numSVias ++;
                    // cerr << "Enclose: net" << netId << " layer" << layId << " source, grid = (" << grid_i->xId() << ", " << grid_i->yId() << ")" << endl; 
                    if (layId > 0) {
                        // mtx[node_id][node_id] += via_condutance;
                        // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                        vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                        vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                    } else {
                        // if (netId != 1) {
                        vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_up));
                        I(node_id) = _db.vNet(netId)->sourcePort()->voltage() * via_condutance_up;
                        // }
                        // cerr << "layer" << layId << " node" << node_id;
                        // cerr << ": sVolt = " << _db.vNet(netId)->sourcePort()->voltage();
                        // cerr << ", via_conductance_up = " << via_condutance_up;
                        // cerr << ", I" << node_id << " = " << I(node_id) << endl;
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
                        numTVias[tPortId] ++;
                        if (layId > 0) {
                            // mtx[node_id][node_id] += via_condutance;
                            // mtx[node_id][getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())]] -= via_condutance;
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, via_condutance_down));
                            vTplY.push_back(Eigen::Triplet<double>(node_id, getID[make_tuple(layId-1, grid_i->xId(), grid_i->yId())], -via_condutance_down));
                        } else {
                            double loadConductance = _db.vNet(netId)->targetPort(tPortId)->current() / (_db.vNet(netId)->targetPort(tPortId)->voltage() * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias());
                            //  * _db.vNet(netId)->targetPort(tPortId)->viaCluster()->numVias()
                            // if (netId != 1) {
                            vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_up + 1.0/loadConductance)));
                            // } else {
                            //     vTplY.push_back(Eigen::Triplet<double>(node_id, node_id, 1.0/(1.0/via_condutance_down + 1.0/loadConductance)));
                            // }
                            // cerr << "layer" << layId << " node" << node_id;
                            // cerr << ": tPort" << tPortId ;
                            // cerr << ", total conductance = " << 1.0/(1.0/via_condutance_up + 1.0/loadConductance) << endl;
                            // << ": via_conductance = " << via_condutance_up << ", loadConductance = " << loadConductance;
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
        // Port* sPort = _db.vNet(netId)->sourcePort();
        // int realNumSVias = sPort->viaCluster()->numVias();
        // cerr << "layer" << layId << ": numSVias = " << numSVias << ", real numSVias = " << realNumSVias << endl;
        // for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
        //     Port* tPort = _db.vNet(netId)->targetPort(tPortId);
        //     int realNumTVias = tPort->viaCluster()->numVias();
        //     cerr << "layer" << layId << " tPort" << tPortId << ": numTVias = " << numTVias[tPortId] << ", real numTVias = " << realNumTVias << endl;
        // }
    }

    // if (netId == 1) {
    //     cerr << "net1, I = " << endl;
    //     for (size_t i = 0; i < I.size(); ++ i) {
    //         if (I[i] > 0) {
    //             cerr << "I[" << i << "] = " << I[i] << endl;
    //         }
    //     }
    // }

    Y.setFromTriplets(vTplY.begin(), vTplY.end());
    // Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IdentityPreconditioner> solver;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::Upper> solver;
    // solver.setMaxIterations(1000000);
    // solver.setTolerance(1e-14);
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
            // assert(grid_i->voltage(netId) <= _db.vNet(netId)->sourcePort()->voltage());
        }
    }

    // set current of each grid
    for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
        _vTPortCurr[netId][tPortId] = 0.0;
    }
    assert(_vNetGrid[netId].size() == _db.numLayers());
    for (size_t layId = 0; layId < _vNetGrid[netId].size(); ++ layId) {
        for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
            Grid* grid_i = _vNetGrid[netId][layId][gridId];
            size_t xId = grid_i->xId();
            size_t yId = grid_i->yId();
            size_t node_id = getID[make_tuple(layId, grid_i->xId(), grid_i->yId())];
            double g2g_condutance = _db.vMetalLayer(layId)->conductivity() * _db.vMetalLayer(layId)->thickness() * 1E-3;
            double via_condutance_up, via_condutance_down;
            if (layId > 0) {
                via_condutance_down = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId-1)->thickness()+ _db.vMediumLayer(layId)->thickness()+0.5* _db.vMetalLayer(layId)->thickness()));
            }
            if (layId < _db.numLayers() - 1) {
                via_condutance_up = (_db.vMetalLayer(0)->conductivity() * _db.VIA16D8A24()->metalArea() * 1E-6) / (1E-3 * (0.5*_db.vMetalLayer(layId)->thickness()+ _db.vMediumLayer(layId+1)->thickness()+0.5* _db.vMetalLayer(layId+1)->thickness()));
            }
            if (gridId == 0) {
                // cerr << "layer" << layId << ": g2g_conductance = " << g2g_condutance << ", via_conductance_up = " << via_condutance_up;
                // cerr << ", via_conductance_down = " << via_condutance_down << endl;
            }
            double small_conductance = 1.0;
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
                // double tPortCurr = 0;
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
                            //cerr << "net" << netId << ", tPort" << tPortId << ": voltage = " << grid_i->voltage(netId);
                            //cerr << ", current = " << abs(grid_i->voltage(netId)) /(1.0/via_condutance_up + 1.0/loadConductance) << endl;
                        }
                        if (layId < _db.numLayers()-1) {
                            current += abs(grid_i->voltage() - _vGrid[layId+1][xId][yId]->voltage()) * via_condutance_up;
                        } 
                    }
                }
                // _vTPortCurr[netId][tPortId] += tPortCurr;
                // _vTPortCurr[netId].push_back(tPortCurr);
                // _vTPortVolt[netId].push_back(tPortCurr / loadConductance);
                // cerr << "tPortCurr = " << tPortCurr << ", tPortVolt = " << _vTPortVolt[netId][tPortId] << endl;
            }
            grid_i->setCurrent(current * 0.5);
            // cerr << "gridCurrent = " << current * 0.5 << endl;
        }
    }
    // for(int i=0; i<20; i++) {
    //     for(int j=0; j<20; j++)
    //         printf("%4.1f ", mtx[i][j]);
    //     printf("\n");
    // }
   
    for (size_t tPortId = 0; tPortId < _db.vNet(netId)->numTPorts(); ++ tPortId) {
        double loadResistance = _db.vNet(netId)->targetPort(tPortId)->voltage() / _db.vNet(netId)->targetPort(tPortId)->current();
        _vTPortVolt[netId][tPortId] = _vTPortCurr[netId][tPortId] * loadResistance;
        cerr << "net" << netId << " tPort" << tPortId << ": current = " << _vTPortCurr[netId][tPortId];
        cerr << ", voltage = " << _vTPortVolt[netId][tPortId] << endl;
    }
}


void DetailedMgr::findPointList(){
    cout << "***Start findpointList***" << endl;

    //reset _Ploted
    for(size_t netId = 0; netId < _db.numNets(); netId++){
        for(size_t layId = 0; layId < _db.numLayers(); layId++){
            for(size_t PortId = 0; PortId < _vNetPortGrid[netId].size(); PortId++){
                *(_Ploted[netId][layId][PortId]) = false;
            }
        }
    }
    //clear List
    _Netpolygon.clear();
    //search each net
    for(size_t netId = 0; netId < _db.numNets(); netId++){
        //search each layer
        vector<vector<vector<pair<double,double>>>> Net_Temp;
        for(size_t layId = 0; layId < _db.numLayers(); layId++){
            vector<vector<pair<double,double>>> Layer_Temp;
            for(size_t PortId = 0; PortId < _vNetPortGrid[netId].size(); PortId++){
                if(*(_Ploted[netId][layId][PortId]) == true) continue;
                //PortId = 0 -> source port, PortId > 0 -> Target port
                *(_Ploted[netId][layId][PortId]) = true;

                //take gridId 0 to start
                int x = _vNetPortGrid[netId][PortId][0].first;
                int y = _vNetPortGrid[netId][PortId][0].second;

                while(_vGrid[layId][x][y]->netId() == netId){
                    y++;
                    if(y == _vGrid[layId][x].size()) break;
                }
                int startX = x, startY = y;
                
                int dir = 2; //dir1 -> up, dir2 -> right, dir3 -> down, dir4-> left
                vector<pair<double,double>> contour_temp;
                contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                do{ 
                    //cout << "x : " << x << " , y : " << y <<" , dir : " << dir<< endl;
                    //direction is up
                    if(dir == 1){
                        Grid* left = (x==0)? nullptr : _vGrid[layId][x-1][y];
                        Grid* curr = _vGrid[layId][x][y];
                        if(curr->netId() == netId && (left == nullptr || left->netId() != netId)){
                            y++;
                            if(y == _vGrid[layId][x].size()){
                                dir = 2;
                                contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                            }
                        }
                        else if(curr->netId() != netId){
                            dir = 2;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                        else if(left->netId() == netId){
                            dir = 4;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                    }
                    //direction is right
                    else if(dir == 2){
                        Grid* up = (y == _vGrid[layId][x].size())? nullptr : _vGrid[layId][x][y];
                        Grid* curr = _vGrid[layId][x][y-1];
                        if(curr->netId() == netId && (up == nullptr || up->netId() != netId)){
                            x++;
                            if(x == _vGrid[layId].size()){
                                dir = 3;
                                contour_temp.push_back(make_pair((x)*_gridWidth, (y)*_gridWidth));
                            }
                        }
                        else if( curr->netId() != netId){
                            dir = 3;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                        else if(up->netId() == netId){
                            dir = 1;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                    }
                    //direction is down
                    else if(dir == 3){
                        Grid* right = (x == _vGrid[layId].size())? nullptr : _vGrid[layId][x][y-1];
                        Grid* curr = _vGrid[layId][x-1][y-1];
                        if(curr->netId() == netId && (right == nullptr || right->netId() != netId)){
                            y--;
                            if(y == 0){
                                dir = 4;
                                contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                            }
                        }
                        else if(curr->netId() != netId){
                            dir = 4;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                        else if(right->netId() == netId){
                            dir = 2;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                    }
                    //direction is left
                    else if(dir == 4){
                        Grid* down = (y == 0)? nullptr : _vGrid[layId][x-1][y-1];
                        Grid* curr = _vGrid[layId][x-1][y];
                        if(curr->netId() == netId && (down == nullptr || down->netId() != netId)){
                            x--;
                            if(x == 0){
                                dir = 1;
                                contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                            }
                        }
                        else if(curr->netId() != netId){
                            dir = 1;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                        else if(down->netId() == netId){
                            dir = 3;
                            contour_temp.push_back(make_pair(x*_gridWidth,y*_gridWidth));
                        }
                    }
                }while(x != startX || y != startY);
                Layer_Temp.push_back(contour_temp);
            }
        Net_Temp.push_back(Layer_Temp);
        }
    _Netpolygon.push_back(Net_Temp);
    }
}


void DetailedMgr::OutputTest(){
    for(size_t netId = 0; netId < _db.numNets(); netId++){
        //cout <<"output netId : " << netId << " , ";
        //search each layer
        for(size_t layId = 0; layId < _db.numLayers(); layId++){
            //cout << "layId : "<<layId << " , ";
            for(size_t polygonId = 0; polygonId < _Netpolygon[netId][layId].size(); polygonId++){
                //cout << "polyId : "<<polygonId << " : "<<endl;
                vector< pair<double, double> > vVtx;
                for(size_t pointId = 0; pointId < _Netpolygon[netId][layId][polygonId].size(); ++ pointId){
                    double x = _Netpolygon[netId][layId][polygonId][pointId].first;
                    double y = _Netpolygon[netId][layId][polygonId][pointId].second;
                    vVtx.push_back(make_pair(x,y));
                    cout << "x: " << x << " y: " << y << endl;
                }
                Polygon* p = new Polygon(vVtx, _plot);
                p->plot(SVGPlotColor::black, layId);
            }
        }
    }
}

void DetailedMgr::OutputFile(ifstream& fin, ofstream& fout){
    cout << "Start writing output file" << endl;
    size_t layId = 0;
    size_t polygonID = 1;
    bool Remove = false;

    vector<string> modifiedLines;
    string line;

    while (getline(fin, line)) {
        //find layer
        if (line.find(".Shape") == 0) {
            modifiedLines.push_back(line);
            size_t pos = line.find("Shape$");
            if (pos != std::string::npos) {
                istringstream iss(line.substr(pos + 6));
                string layerName;
                iss >> layerName;
                //cout << "Layer : " << layerName << endl;
                //need layname2Id
                layId = _db.layName2Id(layerName);
            }
            Remove = false;
        } 
        //find polygon
        else if(line.find("Polygon") == 0){
            string name;
            size_t num_pos = line.find("::");
            size_t color_pos = line.find("Color");

            if (num_pos != string::npos && color_pos != string::npos){
                //find name first 會在::後面 以及 Color前面兩個位置
                name = line.substr(num_pos+2, color_pos-2-(num_pos+2));
                //search net
                bool Searched = false;
                for(size_t netId = 0; netId < _db.numNets();++netId){
                    if(name == _db.vNetName(netId)){
                        Searched = true;
                        break;
                    }
                } 
                //不是我們要的Net
                if(!Searched){
                    string polygon_num = to_string(polygonID);
                    //reset the number of polygon
                    line.replace(7, num_pos - 7, polygon_num);
                    modifiedLines.push_back(line);
                    polygonID ++;
                    Remove = false;
                }
                //是要刪除的部分
                else{
                    Remove = true;
                }
            }
        }
        //others info, don't modify
        else if(line.find(".EndShape") == 0){
            //要把所有net補上
            for(size_t netId = 0; netId < _db.numNets();++netId){
                for(size_t polyId = 0; polyId < _Netpolygon[netId][layId].size(); polyId++){
                    // + 的部分
                    string name = _db.vNetName(netId);
                    ostringstream oss;
                    oss << "Polygon";
                    string polygon_num = to_string(polygonID);
                    polygonID++;
                    oss << polygon_num << "::"<< name << "+ Color = " << _plot.vId2Color(netId) << "   ";
                    //first two points 先處理前兩個點
                    int expX = 0;//暫時設這樣
                    int expY = 0;
                    double point1x = _Netpolygon[netId][layId][polyId][0].first;
                    double point1y = _Netpolygon[netId][layId][polyId][0].second;
                    while(point1x >= 10){
                        point1x /= 10;
                        expX ++;
                    }
                    while(point1y >= 10){
                        point1y /= 10;
                        expY ++;
                    }
                    oss << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                    oss << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";

                    expX = 0;
                    expY = 0;
                    point1x = _Netpolygon[netId][layId][polyId][1].first;
                    point1y = _Netpolygon[netId][layId][polyId][1].second;
                    while(point1x >= 10){
                        point1x /= 10;
                        expX ++;
                    }
                    while(point1y >= 10){
                        point1y /= 10;
                        expY ++;
                    }
                    oss << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                    oss << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                    string addLine = oss.str();
                    modifiedLines.push_back(addLine);
                    //後續是剩下的點，每兩個要一個+
                    ostringstream the_rest;
                    for(size_t pointId = 2; pointId < _Netpolygon[netId][layId][polyId].size(); pointId++){
                        if(pointId%2 == 0){
                            the_rest << "+   ";
                            expX = 0;
                            expY = 0;
                            point1x = _Netpolygon[netId][layId][polyId][pointId].first;
                            point1y = _Netpolygon[netId][layId][polyId][pointId].second;
                            while(point1x >= 10){
                                point1x /= 10;
                                expX ++;
                            }
                            while(point1y >= 10){
                                point1y /= 10;
                                expY ++;
                            }
                            the_rest << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                            the_rest << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                            //剛好是最後一個
                            if(pointId == _Netpolygon[netId][layId][polyId].size() - 1){
                                string temp = the_rest.str();
                                modifiedLines.push_back(temp);
                                the_rest.str("");
                            }
                        }
                        else if(pointId%2 == 1){
                            expX = 0;
                            expY = 0;
                            point1x = _Netpolygon[netId][layId][polyId][pointId].first;
                            point1y = _Netpolygon[netId][layId][polyId][pointId].second;
                            while(point1x >= 10){
                                point1x /= 10;
                                expX ++;
                            }
                            while(point1y >= 10){
                                point1y /= 10;
                                expY ++;
                            }
                            the_rest << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                            the_rest << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                            string temp = the_rest.str();
                            modifiedLines.push_back(temp);
                            the_rest.str("");
                        }
                    }
                    //-的部分,判斷是否需要挖掉
                    //先定義新的Shape
                    Polygon* p = new Polygon(_Netpolygon[netId][layId][polyId], _plot);
                    //先看其他net
                    for(size_t T_netId = 0; T_netId < _db.numNets(); T_netId ++){
                        if(netId != T_netId){
                            for(size_t T_polyId = 0; T_polyId < _Netpolygon[T_netId][layId].size(); T_polyId++){
                                //抓一個點就好
                                double T_pointX = _Netpolygon[T_netId][layId][T_polyId][0].first;
                                double T_pointY = _Netpolygon[T_netId][layId][T_polyId][0].second;
                                if(p->enclose(T_pointX,T_pointY)){
                                    oss.str(""); 
                                    oss << "Polygon";
                                    polygon_num = to_string(polygonID);
                                    polygonID++;
                                    oss << polygon_num << "::+"<< name << "- Color = " << _plot.vId2Color(netId) << "   ";
                                    //寫入其他點
                                    //first two points 先處理前兩個點
                                    expX = 0;//暫時設這樣
                                    expY = 0;
                                    point1x = _Netpolygon[T_netId][layId][T_polyId][0].first;
                                    point1y = _Netpolygon[T_netId][layId][T_polyId][0].second;
                                    while(point1x >= 10){
                                        point1x /= 10;
                                        expX ++;
                                    }
                                    while(point1y >= 10){
                                        point1y /= 10;
                                        expY ++;
                                    }
                                    oss << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                    oss << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";

                                    expX = 0;
                                    expY = 0;
                                    point1x = _Netpolygon[T_netId][layId][T_polyId][1].first;
                                    point1y = _Netpolygon[T_netId][layId][T_polyId][1].second;
                                    while(point1x >= 10){
                                        point1x /= 10;
                                        expX ++;
                                    }
                                    while(point1y >= 10){
                                        point1y /= 10;
                                        expY ++;
                                    }
                                    oss << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                    oss << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                                    addLine = oss.str();
                                    modifiedLines.push_back(addLine);
                                    //後續是剩下的點，每兩個要一個+
                                    the_rest.str("");
                                    for(size_t pointId = 2; pointId < _Netpolygon[netId][layId][polyId].size(); pointId++){
                                        if(pointId%2 == 0){
                                            the_rest << "+   ";
                                            expX = 0;
                                            expY = 0;
                                            point1x = _Netpolygon[T_netId][layId][T_polyId][pointId].first;
                                            point1y = _Netpolygon[T_netId][layId][T_polyId][pointId].second;
                                            while(point1x >= 10){
                                                point1x /= 10;
                                                expX ++;
                                            }
                                            while(point1y >= 10){
                                                point1y /= 10;
                                                expY ++;
                                            }
                                            the_rest << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                            the_rest << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                                            //剛好是最後一個
                                            if(pointId == _Netpolygon[T_netId][layId][T_polyId].size() - 1){
                                                string temp = the_rest.str();
                                                modifiedLines.push_back(temp);
                                                the_rest.str("");
                                            }
                                        }
                                        else if(pointId%2 == 1){
                                            expX = 0;
                                            expY = 0;
                                            point1x = _Netpolygon[T_netId][layId][T_polyId][pointId].first;
                                            point1y = _Netpolygon[T_netId][layId][T_polyId][pointId].second;
                                            while(point1x >= 10){
                                                point1x /= 10;
                                                expX ++;
                                            }
                                            while(point1y >= 10){
                                                point1y /= 10;
                                                expY ++;
                                            }
                                            the_rest << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                            the_rest << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                                            string temp = the_rest.str();
                                            modifiedLines.push_back(temp);
                                            the_rest.str("");
                                        }
                                    }
                                }
                            }
                        }
                    }
                    //其他obstacle
                    for (size_t obsId = 0; obsId < _db.vMetalLayer(layId)->numObstacles(); ++ obsId) {
                        Obstacle* obs = _db.vMetalLayer(layId)->vObstacle(obsId);
                        for (size_t shapeId = 0; shapeId < obs->numShapes(); ++ shapeId) {
                            //同樣先抓一個點
                            double T_pointX = obs->vShape(shapeId)->bPolygonX(0);//取vtxId = 0
                            double T_pointY = obs->vShape(shapeId)->bPolygonY(0);
                            if(p->enclose(T_pointX,T_pointY)){
                                oss.str(""); 
                                oss << "Polygon";
                                polygon_num = to_string(polygonID);
                                polygonID++;
                                oss << polygon_num << "::+"<< name << "- Color = " << _plot.vId2Color(netId) << "   ";
                                //寫入其他點
                                //first two points 先處理前兩個點
                                expX = 0;//暫時設這樣
                                expY = 0;
                                point1x = obs->vShape(shapeId)->bPolygonX(0);
                                point1y = obs->vShape(shapeId)->bPolygonY(0);
                                while(point1x >= 10){
                                    point1x /= 10;
                                    expX ++;
                                }
                                while(point1y >= 10){
                                    point1y /= 10;
                                    expY ++;
                                }
                                oss << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                oss << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";

                                expX = 0;
                                expY = 0;
                                point1x = obs->vShape(shapeId)->bPolygonX(1);
                                point1y = obs->vShape(shapeId)->bPolygonY(1);
                                while(point1x >= 10){
                                    point1x /= 10;
                                    expX ++;
                                }
                                while(point1y >= 10){
                                    point1y /= 10;
                                    expY ++;
                                }
                                oss << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                oss << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                                addLine = oss.str();
                                modifiedLines.push_back(addLine);
                                //後續是剩下的點，每兩個要一個+
                                the_rest.str("");
                                for(size_t vtxId = 2; vtxId < obs->vShape(shapeId)->numBPolyVtcs(); ++ vtxId){
                                    if(vtxId%2 == 0){
                                        the_rest << "+   ";
                                        expX = 0;
                                        expY = 0;
                                        point1x =  obs->vShape(shapeId)->bPolygonX(vtxId);
                                        point1y =  obs->vShape(shapeId)->bPolygonY(vtxId);
                                        while(point1x >= 10){
                                            point1x /= 10;
                                            expX ++;
                                        }
                                        while(point1y >= 10){
                                            point1y /= 10;
                                            expY ++;
                                        }
                                        the_rest << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                        the_rest << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                                        //剛好是最後一個
                                        if(vtxId == obs->vShape(shapeId)->numBPolyVtcs() - 1){
                                            string temp = the_rest.str();
                                            modifiedLines.push_back(temp);
                                            the_rest.str("");
                                        }
                                    }
                                    else if(vtxId%2 == 1){
                                        expX = 0;
                                        expY = 0;
                                        point1x =  obs->vShape(shapeId)->bPolygonX(vtxId);
                                        point1y =  obs->vShape(shapeId)->bPolygonY(vtxId);
                                        while(point1x >= 10){
                                            point1x /= 10;
                                            expX ++;
                                        }
                                        while(point1y >= 10){
                                            point1y /= 10;
                                            expY ++;
                                        }
                                        the_rest << fixed << setprecision(6) << point1x << "e+0" << to_string(expX) << "mm ";
                                        the_rest << fixed << setprecision(6) << point1y << "e+0" << to_string(expY) << "mm ";
                                        string temp = the_rest.str();
                                        modifiedLines.push_back(temp);
                                        the_rest.str("");
                                    }
                                }
                            }
                        }
                    }
                    
                //判斷完畢，清掉指標 (之後優化可以額外寫一個vector存這些新的shape指標，就不用每次判斷都要new一個新的)
                delete p;
                }
            
            } 
            //最後補回.Endshape
            modifiedLines.push_back(".EndShape");
            Remove = false;
        }
        else if(line.find("+") == 0){
            //不需要Remove的話就寫回去
            if(!Remove){
                modifiedLines.push_back(line);
            }
        }
        else{
            modifiedLines.push_back(line);
            Remove = false;
        }
    }
    
    
    for (const auto& modifiedLine : modifiedLines) {
        fout << modifiedLine << '\n';
    }

    cout << "Finish writing output file"<< endl;
}

void DetailedMgr::printResult() {
    int area = 0;
    int overlapArea = 0;
    // for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
    //     for (size_t xId = 0; xId < _numXs; ++ xId) {
    //         for (size_t yId = 0; yId < _numYs; ++ yId) {
    //             Grid* grid = _vGrid[layId][xId][yId];
    //             if (grid->congestCur() >= 1 && !grid->hasObs()) area++;
    //             overlapArea += grid->congestCur();
    //         }
    //     }
    // }
    for (size_t netId = 0; netId < _db.numNets(); ++ netId) {
        for (size_t layId = 0; layId < _db.numLayers(); ++ layId) {
            for (size_t gridId = 0; gridId < _vNetGrid[netId][layId].size(); gridId ++) {
                Grid* grid = _vNetGrid[netId][layId][gridId];
                area++;
                // overlapArea += grid->congestCur();
            }
        }
    }
    overlapArea -= area;
    cerr << "area = " << area << endl;
    // cerr << "overlapArea = " << overlapArea << endl;
}