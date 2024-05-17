#include "SVGPlot.h"

void SVGPlot::startPlot(double canvasW, double canvasH) {
    _fout << "<html>" << endl;
    _fout << "<body>" << endl;
    _fout << "<svg width=\"" << canvasW <<"\" height=\"" << canvasH << "\">" << endl;
    // drawRect(0, 0, canvasW, canvasH, SVGPlotColor::black, 0);
    for (size_t layId = 0; layId < _numLayers; ++ layId) {
        drawRect(0, 0, _boardWidth, _boardHeight, SVGPlotColor::white, layId);
    }
}

void SVGPlot::endPlot() {
    _fout << "</svg>" << endl;
    _fout << "</body>" << endl;
    _fout << "</html>" << endl;
}

void SVGPlot::drawSquare(double leftX, double bottY, double width, size_t colorId, size_t layId) {
    _fout << "  <rect x=\"" << (leftX + _boardWidth*layId)*_plotRatio << "\" y=\"" << bottY*_plotRatio << "\" width=\"" << width*_plotRatio << "\" height=\"" << width*_plotRatio;
    _fout << "\" style=\"fill:" << _vColor[colorId] << ";stroke:gray;stroke-width:1\" />\n";
}

void SVGPlot::drawRect(double leftX, double bottY, double width, double height, size_t colorId, size_t layId) {
    _fout << "  <rect x=\"" << (leftX + _boardWidth*layId)*_plotRatio << "\" y=\"" << bottY*_plotRatio << "\" width=\"" << width*_plotRatio << "\" height=\"" << height*_plotRatio;
    _fout << "\" style=\"fill:" << _vColor[colorId] << ";stroke:black;stroke-width:3\" />\n";
}

void SVGPlot::drawCircle(double centerX, double centerY, double r, size_t colorId, size_t layId) {
    _fout << "  <circle cx=\"" << (centerX + _boardWidth*layId)*_plotRatio << "\" cy=\"" << (_boardHeight - centerY)*_plotRatio << "\" r=\"" << r*_plotRatio;
    _fout << "\" fill=\"" << _vColor[colorId] << "\" stroke=\"gray\" stroke-width=\"1\" />\n";
}

void SVGPlot::drawLine(double x1, double y1, double x2, double y2, size_t colorId, size_t layId, double width) {
    _fout << "  <line x1=\"" << (x1 + _boardWidth*layId)*_plotRatio << "\" y1=\"" << (_boardHeight-y1)*_plotRatio << "\" x2=\"" << (x2 + _boardWidth*layId)*_plotRatio << "\" y2=\"" << (_boardHeight-y2)*_plotRatio;
    _fout << "\" style=\"stroke:" << _vColor[colorId] << ";stroke-width:" << width << "\" />\n";
}

void SVGPlot::drawPolygon(vector< pair<double, double> > vVtx, size_t colorId, size_t layId) {
    _fout << "  <polygon points=\"";
    for (size_t vtxId = 0; vtxId < vVtx.size(); ++ vtxId) {
        _fout << (vVtx[vtxId].first + _boardWidth*layId)*_plotRatio << "," << (_boardHeight - vVtx[vtxId].second)*_plotRatio;
        if (vtxId < vVtx.size()-1) _fout << " ";
    }
    _fout << "\" style=\"fill:" << _vColor[colorId] << ";stroke:gray;stroke-width:0;fill-opacity:0.7\" />" << endl;
}

void SVGPlot::drawText(double x, double y, string text, size_t colorId, size_t layId, double fontSize) {
    _fout << "  <text fill=\"" << _vColor[colorId] << "\" font-size=\"" << fontSize << "\" font-family=\"Verdana\" x=\"" << (x + _boardWidth*layId)*_plotRatio << "\" y=\"" << (_boardHeight-y)*_plotRatio << "\">" << text << "</text>" << endl;
}

tuple<int, int, int> SVGPlot::value2color(double value) {
    double f = (value - _lbColorValue) / (_ubColorValue - _lbColorValue);
    // cerr << "f = " << f << endl;
    double a=(1-f)/0.25;
    int x = floor(a);
    int y = floor(255*(a - x));
    int r, g, b;
    switch(x) {
        case 0: r=255;   g=y;     b=0;   break;
        case 1: r=255-y; g=255;   b=0;   break;
        case 2: r=0;     g=255;   b=y;   break;
        case 3: r=0;     g=255-y; b=255; break;
        case 4: r=0;     g=0;     b=255; break;
        default: r=165;  g=42;   b=42; break;
    }
    return make_tuple(r, g, b);
}

void SVGPlot::drawSquareValue(double leftX, double bottY, double width, double colorValue, size_t layId) {
    tuple<int, int, int> rgb = value2color(colorValue);
    _fout << "  <rect x=\"" << (leftX + _boardWidth*layId)*_plotRatio << "\" y=\"" << (_boardHeight - bottY - width)*_plotRatio << "\" width=\"" << width*_plotRatio << "\" height=\"" << width*_plotRatio;
    _fout << "\" style=\"fill:rgb(" << get<0>(rgb) << "," << get<1>(rgb) << "," << get<2>(rgb) << ");stroke:gray;stroke-width:1\" />\n";
}