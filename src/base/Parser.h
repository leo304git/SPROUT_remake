#ifndef PARSER_H
#define PARSER_H

#include "Include.h"
#include "DB.h"
#include "SVGPlot.h"
#include "Shape.h"

class Parser {
    public:
        //Parser(ifstream& finST, ifstream& fin, DB& db, SVGPlot& plot) : _finST(finST), _fin(fin), _db(db), _plot(plot) {}
        //Parser(ifstream& finST, ifstream& fin, DB& db, double offsetX, double offsetY, SVGPlot& plot) : _finST(finST), _fin(fin), _db(db), _offsetX(offsetX), _offsetY(offsetY), _plot(plot) {}
        Parser(ifstream& finST, ifstream& fin, ifstream& finOb, DB& db, double offsetX, double offsetY, SVGPlot& plot) : _finST(finST), _fin(fin), _finOb(finOb), _db(db), _offsetX(offsetX), _offsetY(offsetY), _plot(plot) {}
        ~Parser() {}

        void testInitialize(double boardWidth, double boardHeight, double gridWidth);
        void parse();
    private:
        void parseST();
        void parseLayer();
        void parseShape();
        void parseObstacle();
        string parseNodeTrace();
        void parseVia(string data);
        void parseConnect();
        string toLineBegin(string word);
        double extractDouble(stringstream& ss, int eraseLength);
        ifstream& _finST;
        ifstream& _fin;
        ifstream& _finOb;
        DB& _db;
        SVGPlot& _plot;
        map<string, int> _layName2Id;

        vector< string > _vNetName;         // index = [netId]
        vector< vector< string > > _vVRM;   // index = [netId] [VRMId]
        vector< vector< string > > _vSINK;  // index = [netId] [SINKId]
        // vector< vector< string > > _vSNode; // index = [netId] [sNodeId]
        // vector< vector< string > > _vTNode; // index = [netId] [tNodeId]
        double _offsetX;
        double _offsetY;

};

#endif