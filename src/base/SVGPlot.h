#ifndef SVGPLOT_H
#define SVGPLOT_H

#include "Include.h"
using namespace std;

enum SVGPlotColor {
    lightsalmon,gold,greenyellow,lightblue,mediumpurple,red,orange,green,blue,purple,gray,black,white
};

class SVGPlot {
    public:
        // SVGPlot(ofstream& fout, double boardWidth, double boardHeight, double gridWidth, size_t numLayers, double plotRatio)
        // : _fout(fout), _boardWidth(boardWidth), _boardHeight(boardHeight), _gridWidth(gridWidth), _numLayers(numLayers), _plotRatio(plotRatio) {
        //     _vColor = {"lightsalmon","gold","greenyellow","lightblue","mediumpurple","red", "orange", "green", "blue", "purple", "gray", "black", "white"};
        //     startPlot(_boardWidth*_plotRatio*numLayers, _boardHeight*plotRatio);
        // }
        SVGPlot(ofstream& fout, double plotRatio) : _fout(fout), _plotRatio(plotRatio) {
            _vColor = {"lightsalmon","gold","greenyellow","lightblue","mediumpurple","red", "orange", "green", "blue", "purple", "gray", "black", "white"};
        }
        ~SVGPlot() {}

        void setBoard(double boardWidth, double boardHeight, size_t numLayers) {
            _boardWidth = boardWidth;
            _boardHeight = boardHeight;
            _numLayers = numLayers;
            _gridWidth = 1.0;
            startPlot(_boardWidth*_plotRatio*numLayers, _boardHeight*_plotRatio);
        }
        void startPlot(double canvasW, double canvasH);
        void endPlot();
        void drawSquare(double leftX, double topY, double width, size_t colorId, size_t layId);
        void drawSquareValue(double leftX, double bottY, double width, double colorValue, size_t layId);
        void drawRect(double leftX, double bottY, double width, double height, size_t colorId, size_t layId);
        void drawCircle(double centerX, double centerY, double r, size_t colorId, size_t layId);
        void drawLine(double x1, double y1, double x2, double y2, size_t colorId, size_t layId, double width = 5);
        void drawPolygon(vector< pair<double, double> > vVtx, size_t colorId, size_t layId);
        void drawText(double x, double y, string text, size_t colorId, size_t layId, double fontSize = 12);

        double gridWidth() const { return _gridWidth; }
        tuple<int, int, int> value2color(double value);
        void setColorValueRange(double lb, double ub) { _lbColorValue = lb; _ubColorValue = ub; }
        string vId2Color(size_t netId) { return _vColor[netId]; }

    private:
        ofstream& _fout;
        vector<string> _vColor;
        double _gridWidth;
        double _boardWidth;
        double _boardHeight;
        double _plotRatio;
        double _lbColorValue;
        double _ubColorValue;
        size_t _numLayers;
};

#endif