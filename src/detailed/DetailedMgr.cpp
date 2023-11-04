#include "DetailedMgr.h"
#include <Eigen/IterativeLinearSolvers>

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
                        grid->setNetId(netId);
                        // grid->setOccupied(true);
                        _vNetGrid[netId][layId].push_back(grid);
                    }
                } else {
                    cerr << "BFS fail: lay" << layId << " net" << netId << " s->t" << tPortId << endl;
                }
                
                for (size_t tPortId1 = tPortId+1; tPortId1 < net->numTPorts(); ++ tPortId1) {
                    AStarRouter tRouter(_vGrid[layId], _vTGrid[netId][layId][tPortId], _vTGrid[netId][layId][tPortId1], _gridWidth);
                    if (tRouter.BFS()) {
                        for (size_t pGridId = 0; pGridId < tRouter.numPath(); ++ pGridId) {
                            Grid* grid = tRouter.path(pGridId);
                            grid->setNetId(netId);
                            // grid->setOccupied(true);
                            _vNetGrid[netId][layId].push_back(grid);
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
