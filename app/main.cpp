#include "base/Include.h"
#include "base/DB.h"
#include "base/Parser.h"
#include "base/SVGPlot.h"
#include "detailed/DetailedMgr.h"
#include "PreMgr.h"
#include  <time.h>
using namespace std;

int main(int argc, char* argv[]){

    ifstream finST, fin;
    ifstream finOb;
    ofstream fout;
    ofstream file_output;
    finST.open(argv[1], ifstream::in);
    if (finST.is_open()) {
        cout << "input file (st components) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file (st components)" << endl;
    }
    fin.open(argv[2], ifstream::in);
    if (fin.is_open()) {
        cout << "input file is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }
    fout.open(argv[3], ofstream::out);
    if (fout.is_open()) {
        cout << "output file is opened successfully" << endl;
    } else {
        cerr << "Error opening output file" << endl;
    }
    finOb.open(argv[4], ifstream::in);
    if (finOb.is_open()) {
        cout << "input file (obstacle) is opened successfully" << endl;
    } else {
        cerr << "Error opening input file" << endl;
    }
    file_output.open(argv[5], ofstream::out);
    if (file_output.is_open()) {
        cout << "output file is opened successfully" << endl;
    } else {
        cerr << "Error opening output file" << endl;
    }
    
    // ofstream fout1;
    // fout1.open(argv[2], ofstream::out);
    // if (fout1.is_open()) {
    //     cout << "output file is opened successfully" << endl;
    // } else {
    //     cerr << "Error opening output file" << endl;
    // }

    // double gridWidth = 4;
    // double boardWidth = 15*gridWidth;
    // double boardHeight = 19*gridWidth;
    // size_t numLayers = 4;
    // double gridWidth = 8;
    // double boardWidth = 50*gridWidth;
    // double boardHeight = 15*gridWidth;
    // size_t numLayers = 12;
    double gridWidth = 1;
    // double boardWidth = 75*gridWidth;
    // double boardHeight = 40*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 40;
    // double offsetY = 40;

    // For Example 1
    // double boardWidth = 75*gridWidth;
    // double boardHeight = 40*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 40;
    // double offsetY = 40;

    // For Example 2 
    // double boardWidth = 100*gridWidth;
    // double boardHeight = 70*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 95;
    // double offsetY = 45;

    // // For Example 3 
    // double boardWidth = 100*gridWidth;
    // double boardHeight = 65*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 25;
    // double offsetY = 20;

    // // For Example 4 
    // double boardWidth = 80*gridWidth;
    // double boardHeight = 55*gridWidth;
    // size_t numLayers = 4;
    // double offsetX = 120;
    // double offsetY = 10;

    // For Example 2 single layer
    // double boardWidth = 100*gridWidth;
    // double boardHeight = 70*gridWidth;
    // size_t numLayers = 1;
    // double offsetX = 95;
    // double offsetY = 45;

    // // For Example 5 (smaller)
    double boardWidth = 50*gridWidth;
    double boardHeight = 55*gridWidth;
    size_t numLayers = 5;
    double offsetX = 130;
    double offsetY = 10;


    // SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 6.0);
    // SVGPlot plot(fout, boardWidth, boardHeight, gridWidth, numLayers, 6.0);
    SVGPlot plot(fout, 10.0);
    DB db(plot);
    // db.setBoundary(boardWidth, boardHeight);
    // Parser parser(finST, fin, finOb, db, offsetX, offsetY, plot);
    Parser parser(finST, fin, finOb, db, plot);
    parser.parse();

// /*
    //time
    time_t start, end;
    time(&start);
    // NetworkMgr mgr(db, plot);
    PreMgr preMgr(db, plot);
    preMgr.nodeClustering();
    preMgr.assignPortPolygon();
    preMgr.plotBoundBox();
    
    // replace this line with a real parser function
    // parser.testInitialize(boardWidth, boardHeight, gridWidth);

    // db.print();
    
    //GlobalMgr globalMgr(db, plot);
    

    // replace this line with a real OASG building function
    // globalMgr.buildTestOASG();
    // globalMgr.buildOASG();
    // globalMgr.plotOASG();
    // globalMgr.layerDistribution();
    // // globalMgr.plotRGraph();
    // globalMgr.buildTestNCOASG();
    // // globalMgr.plotNCOASG();
    // globalMgr.voltageAssignment();
    // try {
    //     // globalMgr.currentDistribution();
    //     globalMgr.voltCurrOpt();
    // } catch (GRBException e) {
    //     cerr << "Error = " << e.getErrorCode() << endl;
    //     cerr << e.getMessage() << endl;
    // }
    // globalMgr.plotCurrentPaths();

    DetailedMgr detailedMgr(db, plot, 2 * db.VIA16D8A24()->drillRadius());
    // // detailedMgr.eigenTest();
    detailedMgr.initGridMap();
    // detailedMgr.check();
    // detailedMgr.plotGraph();
    // // // detailedMgr.plotGridMap();
    //detailedMgr.naiveAStar();
    // detailedMgr.check();
    // detailedMgr.plotGridMap();
    //detailedMgr.synchronize();
    //detailedMgr.check();
    detailedMgr.addPortVia();
    detailedMgr.addViaGrid();
    //detailedMgr.naiveAStar();
    // detailedMgr.check();
    //detailedMgr.buildMtx(1);
    //detailedMgr.check();

    detailedMgr.SPROUT();
    detailedMgr.RemoveIsolatedGrid();
    time(&end);
    detailedMgr.plotDB();
    //detailedMgr.plotGridMapVoltage();
    //detailedMgr.plotGridMapCurrent();
    detailedMgr.plotGridMap();
    // detailedMgr.findPointList();
    //detailedMgr.OutputTest();
    ////output////
    //ifstream file_input;
    //ofstream file_output;

    // file_input.open(argv[5], ifstream::in);
    // if (file_input.is_open()) {
    //     cout << "input file is opened successfully" << endl;
    // } else {
    //     cerr << "Error opening input file" << endl;
    // }
    
    fin.seekg(0, std::ios::beg);
    detailedMgr.OutputFile(fin,file_output);
    /////////////////
    //detailedMgr.buildMtx();
    //time(&end);
    double time_used = double(end - start);
    int hour = 0, min = 0;
    if(time_used >= 60){
        min = time_used/60;
        time_used = time_used - min*60;
    }
    if(min >= 60){
        hour = min/60;
        min = min%60;
    }
    cout << "Time : " << hour << " hours " << min <<" mins "<< fixed << setprecision(5) << time_used << " sec " << endl; 
    detailedMgr.buildMtx();
    detailedMgr.printResult();
// */
    //detailedMgr.writeColorMap_v2("../../exp/output/voltageColorMap.txt", 1);
    //detailedMgr.writeColorMap_v2("../../exp/output/currentColorMap.txt", 0);

    // for (size_t layId = 0; layId < db.numLayers(); ++ layId) {
    //     for (size_t netId = 0; netId < db.numNets(); ++netId){
    //         detailedMgr.fillInnerCircle(layId, netId);
    //     }        
    // }    
    // // detailedMgr.fillInnerCircle(0, 1);
    

    //detailedMgr.plotGridMap();
    // detailedMgr.plotGridMapCurrent();

    //globalMgr.plotDB();


    // mgr.genRGraph();
    // // mgr.drawRGraph();
    // mgr.distrNet();

    // // draw routing graph
    // // mgr.drawRGraph(true);
    // mgr.drawDB();
    // fout.close();
    return 0;
}