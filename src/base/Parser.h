#ifndef PARSER_H
#define PARSER_H

#include "Include.h"
#include "DB.h"
#include "SVGPlot.h"

class Parser {
    public:
        Parser(ifstream& fin, DB& db, SVGPlot& plot) : _fin(fin), _db(db), _plot(plot) {}
        ~Parser() {}

        void testInitialize(double boardWidth, double boardHeight, double gridWidth);
        void parse();
    private:
        void parseLayer();
        void parseShape();
        string parseNodeTrace();
        void parseVia(string data);
        string toLineBegin(string word);
        double extractDouble(stringstream& ss, int eraseLength);
        ifstream& _fin;
        DB& _db;
        SVGPlot& _plot;
        map<string, int> _layName2Id;

};

#endif