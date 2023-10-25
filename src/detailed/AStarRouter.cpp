#include "AStarRouter.h"
// #include "../base/Include.h"
// using namespace std;

bool AStarRouter::BFS() {
    queue<GNode*> Queue;
    _sNode->setParent(_sNode);
    Queue.push(_sNode);
    _sNode->setStatus(GNodeStatus::InQueue);

    auto step = [&](Grid* orgGrid, size_t nbrId) -> bool {
        GNode* orgNode;
        if (orgGrid == _sGrid) {
            orgNode = _sNode;
        } else {
            assert(orgNode != _tNode);
            orgNode = _vGNode[orgGrid->xId()][orgGrid->yId()];
        }
        Grid* nbrGrid = orgGrid->vNeighbor(nbrId);
        double newCost;
        if (nbrGrid == _tGrid) {
            _tNode->setParent(orgNode);
            backTrace();
            return true;
        } else {
            // assert(nbrGrid != _sGrid);
            GNode* nbrNode = _vGNode[nbrGrid->xId()][nbrGrid->yId()];
            if (nbrNode->status() != GNodeStatus::InPath) {
                newCost = orgNode->cost() + 1;
                if (nbrNode->status() == GNodeStatus::Init) {
                    // cerr << stepNode << ": Init" << endl;
                    nbrNode->setCost(newCost);
                    nbrNode->setParent(orgNode);
                    nbrNode->setStatus(GNodeStatus::InQueue);
                    Queue.push(nbrNode);
                } else {
                    assert(nbrNode->status() == GNodeStatus::InQueue);
                    if (nbrNode->cost() > newCost) {
                        // cerr << stepNode << ": InQueue" << endl;
                        nbrNode->setCost(newCost);
                        nbrNode->setParent(orgNode);
                        Queue.push(nbrNode);
                    }
                }
            }
            return false;
        }
        return false;
    };

    while (!Queue.empty()) {
        GNode* node = Queue.front();
        Queue.pop();
        node->setStatus(GNodeStatus::InPath);
        Grid* grid = node->grid();
        for (size_t nbrId = 0; nbrId < grid->numNeighbors(); ++ nbrId) {
            if (!grid->vNeighbor(nbrId)->occupied()) {
                if (step(grid, nbrId)) return true;
            }
        }
    }
    return false;
}

void AStarRouter::backTrace() {
    GNode* node = _tNode;
    while(node != _sNode) {
        _path.push_back(node->grid());
        node = node->parent();
    }
    _path.push_back(node->grid());
}

// bool AStarRouter::route() {
//     // define the priority queue
//     // auto cmp = [](GNode* a, GNode* b) { return a->cost() < b->cost(); };
//     // multiset<GNode*, decltype(cmp) > Q(cmp);
//     queue<GNode*> Queue;

//     // initialize the source node
//     GNode* sNode = _vGNode[_sPos.first][_sPos.second];
//     sNode->setParent(sNode);
//     // sNode->setCost(0.0);
//     // sNode->setCurDist(0.0);
//     // sNode->setEstDist(0.0);
//     // sNode->setCongest(0.0);
//     // Q.insert(sNode);
//     Queue.push(sNode);
//     sNode->setStatus(GNodeStatus::InQueue);

//     // define lambda functions
//     auto step = [&](GNode* orgNode, int xId, int yId, Direction dir) -> bool {
//         GNode* stepNode = _vGNode[xId][yId];
//         // double newCurDist, newEstDist, newLineDist, newCongest, newCost;
//         if (xId == _tPos.first && yId == _tPos.second) {
//         // if ((dir == Direction::Up && xId == _tPos.first && yId + floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.second) ||
//         //     (dir == Direction::Down && xId == _tPos.first && yId - floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.second) ||
//         //     (dir == Direction::Right && xId + floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.first && yId == _tPos.second) ||
//         //     (dir == Direction::Left && xId - floor(ceil(_lbWidth/_gridWidth)/2.0) == _tPos.first && yId == _tPos.second)) {
//             stepNode->setParent(orgNode);
//             backTrace(xId, yId);
//             // cerr << _vGNode[-1][-1] << endl;
//             return true;
//         } else if (stepNode->status() != GNodeStatus::InPath) {
//             // cerr << "stepNode = (" << stepNode->xId() << ", " << stepNode->yId() << "), address = " << stepNode;
//             // cerr << ", cost = " << stepNode->cost() << ;
//             newCurDist = orgNode->curDist()+1;
//             newEstDist = estDistCost(xId, yId);
//             newLineDist = lineDistCost(xId, yId);
//             // cerr << ", lineDist = " << newLineDist;
//             newCongest = marginCongestCost(xId, yId, dir) + orgNode->congest();
//             // cerr << ", marginCongestCost = " << marginCongestCost(xId, yId, dir);
//             newCost = _distWeight*(newCurDist + newEstDist + 0*newLineDist) + (1-_distWeight)*newCongest;
//             if (stepNode->status() == GNodeStatus::Init) {
//                 // cerr << stepNode << ": Init" << endl;
//                 stepNode->setCurDist(newCurDist);
//                 stepNode->setEstDist(newEstDist);
//                 stepNode->setCongest(newCongest);
//                 stepNode->setCost(newCost);
//                 stepNode->setParent(orgNode);
//                 stepNode->setStatus(GNodeStatus::InQueue);
//                 Q.insert(stepNode);
//             } else {
//                 assert(stepNode->status() == GNodeStatus::InQueue);
//                 if (stepNode->cost() > newCost) {
//                     // cerr << stepNode << ": InQueue" << endl;
//                     // Q.erase(Q.find(stepNode));
//                     stepNode->setCurDist(newCurDist);
//                     stepNode->setEstDist(newEstDist);
//                     stepNode->setCongest(newCongest);
//                     stepNode->setCost(newCost);
//                     stepNode->setParent(orgNode);
//                     Q.insert(stepNode);
//                 }
//             }
//             // cerr << "stepNode = (" << stepNode->xId() << ", " << stepNode->yId() << "), cost = " << stepNode->cost() << ", address = " << stepNode << endl;
//             // cerr << ", cost = " << stepNode->cost() << endl;
//             return false;
//         }
//         return false;
//     };

//     // start searching
//     while (!Q.empty()) {
//         GNode* node = *Q.begin();
//         Q.erase(Q.begin());
//         // cerr << "node = (" << node->xId() << ", " << node->yId() << "), ";
//         // cerr << "node->cost() = " << node->cost() << endl;
//         node->setStatus(GNodeStatus::InPath);
//         if (legal(node->xId(), node->yId()+1)) {
//             if (step(node, node->xId(), node->yId()+1, Direction::Up)) return true;
//         }
//         if (legal(node->xId(), node->yId()-1)) {
//             if (step(node, node->xId(), node->yId()-1, Direction::Down)) return true;
//         }
//         if (legal(node->xId()+1, node->yId())) {
//             if (step(node, node->xId()+1, node->yId(), Direction::Right)) return true;
//         }
//         if (legal(node->xId()-1, node->yId())) {
//             if (step(node, node->xId()-1, node->yId(), Direction::Left)) return true;
//         }
//         if (legal(node->xId()+1, node->yId()+1)) {
//             if (step(node, node->xId()+1, node->yId()+1, Direction::UpRight)) return true;
//         }
//         if (legal(node->xId()-1, node->yId()+1)) {
//             if (step(node, node->xId()-1, node->yId()+1, Direction::UpLeft)) return true;
//         }
//         if (legal(node->xId()+1, node->yId()-1)) {
//             if (step(node, node->xId()+1, node->yId()-1, Direction::DownRight)) return true;
//         }
//         if (legal(node->xId()-1, node->yId()-1)) {
//             if (step(node, node->xId()-1, node->yId()-1, Direction::DownLeft)) return true;
//         }
//         // if (node->yId() < numYId()-1-floor(ceil(_lbWidth)/2.0)) {
//         //     if (step(node, node->xId(), node->yId()+1, Direction::Up)) return true;
//         // }
//         // if (node->yId() > 0+floor(ceil(_lbWidth)/2.0)) {
//         //     if (step(node, node->xId(), node->yId()-1, Direction::Down)) return true;
//         // }
//         // if (node->xId() < numXId()-1-floor(ceil(_lbWidth)/2.0)) {
//         //     if (step(node, node->xId()+1, node->yId(), Direction::Right)) return true;
//         // }
//         // if (node->xId() > 0+floor(ceil(_lbWidth)/2.0)) {
//         //     if (step(node, node->xId()-1, node->yId(), Direction::Left)) return true;
//         // }
//         // cerr << "Q = " << endl;
//         // for (set<GNode*, decltype(cmp) >::iterator i=Q.begin(); i!=Q.end(); ++i) {
//         //     cerr << "(" << *i << ", " << *i << ")" << endl; 
//         // }
//         // vector<Grid*> gg;
//         // if (node->yId() == 58) cerr << gg[2] << endl;
//     }
//     return false;
// }

// void AStarRouter::backTrace(int tXId, int tYId) {
//     // GNode* node = _vGNode[_tPos.first][_tPos.second];
//     GNode* node = _vGNode[tXId][tYId];
//     while(node->parent() != node) {
//         _path.push_back(_vGrid[node->xId()][node->yId()]);
//         node = node->parent();
//     }
//     assert(node->xId() == _sPos.first && node->yId() == _sPos.second);
//     _path.push_back(_vGrid[node->xId()][node->yId()]);

//     _exactLength = _path.size()-1;
//     // double length = _exactLength * _gridWidth;
//     // _exactWidth = ceil(_lbWidth * _exactLength / _lbLength);
//     _exactWidth = ceil(_lbWidth/_gridWidth);
//     // cerr << "_length = " << _lbLength << ", _exactLength = " << _exactLength << " _width = " << _lbWidth << ", _exactWidth = " << _exactWidth << endl;

//     // around the ending grid
//     for (int xId = _tPos.first - floor(_exactWidth/2.0); xId <= _tPos.first + floor(_exactWidth/2.0); ++ xId) {
//         if (xId >= 0 && xId < numXId()) {
//             for (int yId = _tPos.second - floor(_exactWidth/2.0); yId <= _tPos.second + floor(_exactWidth/2.0); ++ yId) {
//                 if (yId >= 0 && yId < numYId()) {
//                     _vPGrid.push_back(_vGrid[xId][yId]);
//                 }
//             }
//         }
//     }
//     // around the path
//     node = _vGNode[_tPos.first][_tPos.second];
//     while(node->parent() != node) {
//         // cerr << "node->parent() = (" << node->parent()->xId() << ", " << node->parent()->yId() << ")" << endl;
//         // vector<Grid*> gg;
//         // if (node->parent()->xId() == 21 && node->parent()->yId() ==  58) cerr << gg[2] << endl;
//         if (node->parent()->xId() == node->xId()+1 && node->parent()->yId() == node->yId()) {   // Right
//             for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
//                 if (legal(node->parent()->xId() + floor(_exactWidth/2.0), y)) {
//                     _vPGrid.push_back(_vGrid[node->parent()->xId() + floor(_exactWidth/2.0)][y]);
//                 }
//             }
//             // if (node->parent()->xId() + floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.first && node->parent()->yId() == _sPos.second) {
//             //     break;
//             // }
//         } else if (node->parent()->xId() == node->xId()-1  && node->parent()->yId() == node->yId()) {   // Left
//             for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
//                 if (legal(node->parent()->xId() - floor(_exactWidth/2.0), y)) {
//                     _vPGrid.push_back(_vGrid[node->parent()->xId() - floor(_exactWidth/2.0)][y]);
//                 }
//             }
//             // if (node->parent()->xId() - floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.first && node->parent()->yId() == _sPos.second) {
//             //     break;
//             // }
//         } else if (node->parent()->xId() == node->xId() && node->parent()->yId() == node->yId()+1) {    // Up
//             for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
//                 if (legal(x, node->parent()->yId() + floor(_exactWidth/2.0))) {
//                     _vPGrid.push_back(_vGrid[x][node->parent()->yId() + floor(_exactWidth/2.0)]);
//                 }
//             }
//             // if (node->parent()->xId() == _sPos.first && node->parent()->yId() + floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.second) {
//             //     break;
//             // }
//         } else if (node->parent()->xId() == node->xId() && node->parent()->yId() == node->yId()-1) {    // Down
//             for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
//                 if (legal(x, node->parent()->yId() - floor(_exactWidth/2.0))) {
//                     _vPGrid.push_back(_vGrid[x][node->parent()->yId() - floor(_exactWidth/2.0)]);
//                 }
//             }
//             // if (node->parent()->xId() == _sPos.first && node->parent()->yId() - floor(ceil(_lbWidth/_gridWidth)/2.0) == _sPos.second) {
//             //     break;
//             // }
//         } else if (node->parent()->xId() == node->xId()+1 && node->parent()->yId() == node->yId()+1) {  // UpRight
//             for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
//                 if (legal(x, node->parent()->yId() + floor(_exactWidth/2.0))) {
//                     _vPGrid.push_back(_vGrid[x][node->parent()->yId() + floor(_exactWidth/2.0)]);
//                 }
//             }
//             for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0)-1; ++ y) {
//                 if (legal(node->parent()->xId() + floor(_exactWidth/2.0), y)) {
//                     _vPGrid.push_back(_vGrid[node->parent()->xId() + floor(_exactWidth/2.0)][y]);
//                 }
//             }
//         } else if (node->parent()->xId() == node->xId()-1 && node->parent()->yId() == node->yId()+1) {  // UpLeft
//             for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
//                 if (legal(x, node->parent()->yId() + floor(_exactWidth/2.0))) {
//                     _vPGrid.push_back(_vGrid[x][node->parent()->yId() + floor(_exactWidth/2.0)]);
//                 }
//             }
//             for (int y = node->parent()->yId()-floor(_exactWidth/2.0); y <= node->parent()->yId()+floor(_exactWidth/2.0)-1; ++ y) {
//                 if (legal(node->parent()->xId() - floor(_exactWidth/2.0), y)) {
//                     _vPGrid.push_back(_vGrid[node->parent()->xId() - floor(_exactWidth/2.0)][y]);
//                 }
//             }
//         } else if (node->parent()->xId() == node->xId()+1 && node->parent()->yId() == node->yId()-1) {  // DownRight
//             for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
//                 if (legal(x, node->parent()->yId() - floor(_exactWidth/2.0))) {
//                     _vPGrid.push_back(_vGrid[x][node->parent()->yId() - floor(_exactWidth/2.0)]);
//                 }
//             }
//             for (int y = node->parent()->yId()-floor(_exactWidth/2.0)+1; y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
//                 if (legal(node->parent()->xId() + floor(_exactWidth/2.0), y)) {
//                     _vPGrid.push_back(_vGrid[node->parent()->xId() + floor(_exactWidth/2.0)][y]);
//                 }
//             }
//         } else {  // DownLeft
//             assert(node->parent()->xId() == node->xId()-1 && node->parent()->yId() == node->yId()-1);
//             for (int x = node->parent()->xId()-floor(_exactWidth/2.0); x <= node->parent()->xId()+floor(_exactWidth/2.0); ++ x) {
//                 if (legal(x, node->parent()->yId() - floor(_exactWidth/2.0))) {
//                     _vPGrid.push_back(_vGrid[x][node->parent()->yId() - floor(_exactWidth/2.0)]);
//                 }
//             }
//             for (int y = node->parent()->yId()-floor(_exactWidth/2.0)+1; y <= node->parent()->yId()+floor(_exactWidth/2.0); ++ y) {
//                 if (legal(node->parent()->xId() - floor(_exactWidth/2.0), y)) {
//                     _vPGrid.push_back(_vGrid[node->parent()->xId() - floor(_exactWidth/2.0)][y]);
//                 }
//             }
//         }
//         node = node->parent();
//     } 
// }

// double AStarRouter::marginCongestCost(int xId, int yId, Direction dir) {
//     auto prob = [&](int w) -> double{
//         return pow(1-_widthRatio, w-ceil(_lbWidth/_gridWidth)) * _widthRatio;
//     };
//     double congestion = 0.0;
//     double threshold = 0.0001;
//     for (int w = ceil(_lbWidth/_gridWidth); prob(w) > threshold; ++ w) {
//         // cerr << "w = " << w << ", prob = " << prob(w) << endl;
//         double wCongestion = 0.0;
//         if (dir == Direction::Up) {
//             // cerr << "Up" << endl;
//             for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
//                 // cerr << "x = " << x << ", y = " << yId+floor(w/2.0) << endl;
//                 if (legal(x, yId+floor(w/2.0))) {
//                     wCongestion += _vGrid[x][yId+floor(w/2.0)]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         } else if (dir == Direction::Down) {
//             // cerr << "Down" << endl;
//             for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
//                 // cerr << "x = " << x << ", y = " << yId-floor(w/2.0) << endl;
//                 if (legal(x, yId-floor(w/2.0))) {
//                     wCongestion += _vGrid[x][yId-floor(w/2.0)]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         } else if (dir == Direction::Right) {
//             // cerr << "Right" << endl;
//             for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0); ++ y) {
//                 // cerr << "x = " << xId+floor(w/2.0) << ", y = " << y << endl;
//                 if (legal(xId+floor(w/2.0), y)) {
//                     wCongestion += _vGrid[xId+floor(w/2.0)][y]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         } else if (dir == Direction::Left) {
//             // cerr << "Left" << endl;
//             for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0); ++ y) {
//                 // cerr << "x = " << xId-floor(w/2.0) << ", y = " << y << endl;
//                 if (legal(xId-floor(w/2.0), y)) {
//                     wCongestion += _vGrid[xId-floor(w/2.0)][y]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         } else if (dir == Direction::UpRight) {
//             for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
//                 // cerr << "x = " << x << ", y = " << yId+floor(w/2.0) << endl;
//                 if (legal(x, yId+floor(w/2.0))) {
//                     wCongestion += _vGrid[x][yId+floor(w/2.0)]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//             for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0)-1; ++ y) {
//                 // cerr << "x = " << xId+floor(w/2.0) << ", y = " << y << endl;
//                 if (legal(xId+floor(w/2.0), y)) {
//                     wCongestion += _vGrid[xId+floor(w/2.0)][y]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         } else if (dir == Direction::UpLeft) {
//             for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
//                 // cerr << "x = " << x << ", y = " << yId+floor(w/2.0) << endl;
//                 if (legal(x, yId+floor(w/2.0))) {
//                     wCongestion += _vGrid[x][yId+floor(w/2.0)]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//             for (int y = yId-floor(w/2.0); y <= yId+floor(w/2.0)-1; ++ y) {
//                 // cerr << "x = " << xId-floor(w/2.0) << ", y = " << y << endl;
//                 if (legal(xId-floor(w/2.0), y)) {
//                     wCongestion += _vGrid[xId-floor(w/2.0)][y]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         } else if (dir == Direction::DownRight) {
//             for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
//                 // cerr << "x = " << x << ", y = " << yId-floor(w/2.0) << endl;
//                 if (legal(x, yId-floor(w/2.0))) {
//                     wCongestion += _vGrid[x][yId-floor(w/2.0)]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//             for (int y = yId-floor(w/2.0)+1; y <= yId+floor(w/2.0); ++ y) {
//                 // cerr << "x = " << xId+floor(w/2.0) << ", y = " << y << endl;
//                 if (legal(xId+floor(w/2.0), y)) {
//                     wCongestion += _vGrid[xId+floor(w/2.0)][y]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         } else {
//             assert(dir == Direction::DownLeft);
//             for (int x = xId-floor(w/2.0); x <= xId+floor(w/2.0); ++ x) {
//                 // cerr << "x = " << x << ", y = " << yId-floor(w/2.0) << endl;
//                 if (legal(x, yId-floor(w/2.0))) {
//                     wCongestion += _vGrid[x][yId-floor(w/2.0)]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//             for (int y = yId-floor(w/2.0)+1; y <= yId+floor(w/2.0); ++ y) {
//                 // cerr << "x = " << xId-floor(w/2.0) << ", y = " << y << endl;
//                 if (legal(xId-floor(w/2.0), y)) {
//                     wCongestion += _vGrid[xId-floor(w/2.0)][y]->congestion();
//                 } else {
//                     wCongestion += _obsCongest;
//                 }
//             }
//         }
//         wCongestion *= prob(w);
//         congestion += wCongestion;
//     }
//     return congestion;
// }

// double AStarRouter::pathLength(int threshold) {
//     int gridCount = 0;
//     double pathLength = 0;
//     int buffer = 0;
//     bool turned = false;
//     bool turn = false;
//     GNode* node = _vGNode[_tPos.first][_tPos.second];
//     Direction dir;
//     if (node->parent()->xId() == node->xId()+1) {
//         dir = Direction::Right;
//     } else if (node->parent()->xId() == node->xId()-1) {
//         dir = Direction::Left;
//     } else if (node->parent()->yId() == node->yId()+1) {
//         dir = Direction::Up;
//     } else {
//         assert(node->parent()->yId() == node->yId()-1);
//         dir = Direction::Down;
//     }
//     buffer ++;
//     node = node->parent();

//     while(node->parent() != node) {
//         turn = false;
//         if (node->parent()->xId() == node->xId()+1) {
//             if (dir == Direction::Right) {
//                 turn = true;
//             }
//         } else if (node->parent()->xId() == node->xId()-1) {
//             if (dir == Direction::Left) {
//                 turn = true;
//             }
//         } else if (node->parent()->yId() == node->yId()+1) {
//             if (dir == Direction::Up) {
//                 turn = true;
//             }
//         } else {
//             assert(node->parent()->yId() == node->yId()-1);
//             if (dir == Direction::Down) {
//                 turn = true;
//             }
//         }
//         if (turned) {
//             if (buffer <= threshold) {

//             }
//         }
//         node = node->parent();
//     }

//     // buffer ++;
//     //             if (!turned && buffer > threshold) {
//     //                 pathLength += buffer;
//     //                 buffer = 0;
//     //             }
//     //         } else {
//     //             if (!turned) {
//     //                 turned = true;
//     //             }
//     for (size_t pGridId = 0; pGridId < _path.size(); ++pGridId) {
//         Grid* grid = _path[pGridId];
//     }
// }