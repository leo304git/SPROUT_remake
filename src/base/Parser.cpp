#include "Parser.h"
using namespace std;

void Parser::testInitialize(double boardWidth, double boardHeight, double gridWidth) {
    // double gridWidth = 40.0;
    // _db.setBoundary(15*gridWidth, 19*gridWidth);
    _db.setBoundary(boardWidth, boardHeight);
    _db.setVIA16D8A24();

    _db.addMediumLayer("medium64", 1.524000e-02, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("BOTTOM", 3.556000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium62", 4.927600e-02, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("VCC1", 3.048000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium52", 1.016000e-01, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("VCC", 3.048000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium42", 4.927600e-02, 4.500000e+00, 3.500000e-02);
    _db.addMetalLayer("TOP", 3.556000e-02, 5.959000e+07, 4.500000e+00);
    _db.addMediumLayer("medium40", 1.524000e-02, 4.500000e+00, 3.500000e-02);

    _db.initNet(3);

    ViaCluster* viaCstr;
    double currentBase = 160;

    // _vNet[0]
    _db.addCircleVia(2*gridWidth, 2*gridWidth, 0, ViaType::Source);
    _db.addCircleVia(3*gridWidth, 2*gridWidth, 0, ViaType::Source);
    _db.addCircleVia(2*gridWidth, 3*gridWidth, 0, ViaType::Source);
    _db.addCircleVia(3*gridWidth, 3*gridWidth, 0, ViaType::Source);
    viaCstr = _db.clusterVia({0,1,2,3});
    _db.addPort(1.7, currentBase, viaCstr);
    _db.addCircleVia(5*gridWidth, 14*gridWidth, 0, ViaType::Target);
    _db.addCircleVia(6*gridWidth, 14*gridWidth, 0, ViaType::Target);
    _db.addCircleVia(5*gridWidth, 15*gridWidth, 0, ViaType::Target);
    _db.addCircleVia(6*gridWidth, 15*gridWidth, 0, ViaType::Target);
    viaCstr = _db.clusterVia({4,5,6,7});
    _db.addPort(1.65, currentBase, viaCstr);
    // _vNet[1]
    _db.addCircleVia(6*gridWidth, 2*gridWidth, 1, ViaType::Source);
    _db.addCircleVia(7*gridWidth, 2*gridWidth, 1, ViaType::Source);
    _db.addCircleVia(6*gridWidth, 3*gridWidth, 1, ViaType::Source);
    _db.addCircleVia(7*gridWidth, 3*gridWidth, 1, ViaType::Source);
    viaCstr = _db.clusterVia({8,9,10,11});
    _db.addPort(1.7, 2*currentBase, viaCstr);
    _db.addCircleVia(7*gridWidth, 12*gridWidth, 1, ViaType::Target);
    _db.addCircleVia(8*gridWidth, 12*gridWidth, 1, ViaType::Target);
    viaCstr = _db.clusterVia({12,13});
    _db.addPort(1.65, currentBase, viaCstr);
    _db.addCircleVia(12*gridWidth, 12*gridWidth, 1, ViaType::Target);
    _db.addCircleVia(13*gridWidth, 12*gridWidth, 1, ViaType::Target);
    viaCstr = _db.clusterVia({14,15});
    _db.addPort(1.5, currentBase, viaCstr);
    // _vNet[2]
    _db.addCircleVia(10*gridWidth, 2*gridWidth, 2, ViaType::Source);
    _db.addCircleVia(11*gridWidth, 2*gridWidth, 2, ViaType::Source);
    _db.addCircleVia(10*gridWidth, 3*gridWidth, 2, ViaType::Source);
    _db.addCircleVia(11*gridWidth, 3*gridWidth, 2, ViaType::Source);
    viaCstr = _db.clusterVia({16,17,18,19});
    _db.addPort(1.7, currentBase, viaCstr);
    // _db.addCircleVia(10*gridWidth, 11*gridWidth, 2, ViaType::Target);
    // _db.addCircleVia(10*gridWidth, 12*gridWidth, 2, ViaType::Target);
    // viaCstr = _db.clusterVia({20,21});
    // _db.addPort(1.65, currentBase, viaCstr);
    _db.addCircleVia(9*gridWidth, 17*gridWidth, 2, ViaType::Target);
    _db.addCircleVia(10*gridWidth, 17*gridWidth, 2, ViaType::Target);
    _db.addCircleVia(11*gridWidth, 17*gridWidth, 2, ViaType::Target);
    viaCstr = _db.clusterVia({20,21,22});
    _db.addPort(1.5, currentBase, viaCstr);

    // Obstacle
    _db.addRectObstacle(1, 8*gridWidth, 12*gridWidth, 14*gridWidth, 15*gridWidth);
    _db.addRectObstacle(2, 2*gridWidth, 8*gridWidth, 6*gridWidth, 8*gridWidth);

    // area & via weight
    _db.setFlowWeight(0.5, 0.5);
}

void Parser::parse() {
    string data;
    stringstream ss;
    string word;

    // parse layers
    // data = toLineBegin("Medium");
    // ss.str(data);
    // ss.ignore(numeric_limits<streamsize>::max(), '$');
    // string name, sThickness;
    // double thickness, permittivity, lossTangent;
    // ss >> name;
    // // cerr << "name = " << name << endl;
    // ss.ignore(numeric_limits<streamsize>::max(), '=');
    // ss >> sThickness;
    // // cerr << "thickness = " << sThickness << endl;
    // sThickness.pop_back();
    // sThickness.pop_back();
    // thickness = stod(sThickness);
    // // cerr << "thickness = " << thickness << endl;
    // ss.ignore(numeric_limits<streamsize>::max(), '=');
    // ss >> permittivity;
    // // cerr << "permittivity = " << permittivity << endl;
    // ss.ignore(numeric_limits<streamsize>::max(), '=');
    // ss >> lossTangent;
    // // cerr << "lossTangent = " << lossTangent << endl;
    // _db.addMediumLayer(name, thickness, permittivity, lossTangent);
    // getline(_fin, data);
    // ss.str(data);
    // ss >> word;
    // stringstream sWord(word);
    // getline(sWord, word, '$');
    // // word = sWord.str();
    // // cerr << "word = " << word << endl;
    // while(word == "Medium" || word == "Signal" || word == "Plane") {
    //     // ss.ignore(numeric_limits<streamsize>::max(), '$');
    //     string name, sThickness;
    //     double thickness;
    //     // ss >> name;
    //     getline(sWord, name);
    //     // cerr << "name = " << name << endl;
    //     ss.ignore(numeric_limits<streamsize>::max(), '=');
    //     ss >> sThickness;
    //     // cerr << "thickness = " << sThickness << endl;
    //     sThickness.pop_back();
    //     sThickness.pop_back();
    //     thickness = stod(sThickness);
    //     // cerr << "thickness = " << thickness << endl;
    //     if (word == "Medium") {
    //         double permittivity, lossTangent;
    //         ss.ignore(numeric_limits<streamsize>::max(), '=');
    //         ss >> permittivity;
    //         // cerr << "permittivity = " << permittivity << endl;
    //         ss.ignore(numeric_limits<streamsize>::max(), '=');
    //         ss >> lossTangent;
    //         // cerr << "lossTangent = " << lossTangent << endl;
    //         _db.addMediumLayer(name, thickness, permittivity, lossTangent);
    //     } else if (word == "Signal" || word == "Plane") {
    //         double conductivity, permittivity;
    //         ss.ignore(numeric_limits<streamsize>::max(), '=');
    //         ss >> conductivity;
    //         // cerr << "conductivity = " << conductivity << endl;
    //         ss.ignore(numeric_limits<streamsize>::max(), '=');
    //         ss >> permittivity;
    //         // cerr << "permittivity = " << permittivity << endl;
    //         _db.addMetalLayer(name, thickness, conductivity, permittivity);
    //     }
    //     getline(_fin, data);
    //     ss.str(data);
    //     ss >> word;
    //     sWord.clear();
    //     sWord.str(word);
    //     // cerr << "sWord = " << sWord.str() << endl;
    //     getline(sWord, word, '$');
    //     // cerr << "word = " << word << endl;
    // }

    // _db.reverseMediumLayers();
    // _db.reverseMetalLayers();
    // for (int layId = 0; layId < _db.numLayers(); ++ layId) {
    //     _layName2Id[_db.vMetalLayer(layId)->layName()] = layId;
    //     // cerr << "layer[" << layId << "] = " << _db.vMetalLayer(layId)->layName() << endl;
    //     // _db.vMetalLayer(layId)->print();
    // }
    parseLayer();


    // parse shape
    _fin.seekg(_fin.beg);
    parseShape();
    data = parseNodeTrace();
    parseVia(data);
    

}

void Parser::parseLayer() {
    string data;
    stringstream ss;
    string word;

    data = toLineBegin("Medium");
    ss.str(data);
    ss.ignore(numeric_limits<streamsize>::max(), '$');
    string name, sThickness;
    double thickness, permittivity, lossTangent;
    ss >> name;
    // cerr << "name = " << name << endl;
    ss.ignore(numeric_limits<streamsize>::max(), '=');
    ss >> sThickness;
    // cerr << "thickness = " << sThickness << endl;
    sThickness.pop_back();
    sThickness.pop_back();
    thickness = stod(sThickness);
    // cerr << "thickness = " << thickness << endl;
    ss.ignore(numeric_limits<streamsize>::max(), '=');
    ss >> permittivity;
    // cerr << "permittivity = " << permittivity << endl;
    ss.ignore(numeric_limits<streamsize>::max(), '=');
    ss >> lossTangent;
    // cerr << "lossTangent = " << lossTangent << endl;
    _db.addMediumLayer(name, thickness, permittivity, lossTangent);
    getline(_fin, data);
    ss.str(data);
    ss >> word;
    stringstream sWord(word);
    getline(sWord, word, '$');
    // word = sWord.str();
    // cerr << "word = " << word << endl;
    while(word == "Medium" || word == "Signal" || word == "Plane") {
        // ss.ignore(numeric_limits<streamsize>::max(), '$');
        string name, sThickness;
        double thickness;
        // ss >> name;
        getline(sWord, name);
        // cerr << "name = " << name << endl;
        ss.ignore(numeric_limits<streamsize>::max(), '=');
        ss >> sThickness;
        // cerr << "thickness = " << sThickness << endl;
        sThickness.pop_back();
        sThickness.pop_back();
        thickness = stod(sThickness);
        // thickness = extractDouble(ss, 2);
        // cerr << "thickness = " << thickness << endl;
        if (word == "Medium") {
            double permittivity, lossTangent;
            ss.ignore(numeric_limits<streamsize>::max(), '=');
            ss >> permittivity;
            // cerr << "permittivity = " << permittivity << endl;
            ss.ignore(numeric_limits<streamsize>::max(), '=');
            ss >> lossTangent;
            // cerr << "lossTangent = " << lossTangent << endl;
            _db.addMediumLayer(name, thickness, permittivity, lossTangent);
        } else if (word == "Signal" || word == "Plane") {
            double conductivity, permittivity;
            ss.ignore(numeric_limits<streamsize>::max(), '=');
            ss >> conductivity;
            // cerr << "conductivity = " << conductivity << endl;
            ss.ignore(numeric_limits<streamsize>::max(), '=');
            ss >> permittivity;
            // cerr << "permittivity = " << permittivity << endl;
            _db.addMetalLayer(name, thickness, conductivity, permittivity);
        }
        getline(_fin, data);
        ss.str(data);
        ss >> word;
        sWord.clear();
        sWord.str(word);
        // cerr << "sWord = " << sWord.str() << endl;
        getline(sWord, word, '$');
        // cerr << "word = " << word << endl;
    }

    _db.reverseMediumLayers();
    _db.reverseMetalLayers();
    for (int layId = 0; layId < _db.numLayers(); ++ layId) {
        _layName2Id[_db.vMetalLayer(layId)->layName()] = layId;
        // cerr << "layer[" << layId << "] = " << _db.vMetalLayer(layId)->layName() << endl;
        // _db.vMetalLayer(layId)->print();
    }
}

void Parser::parseShape() {
    string data;
    stringstream ss;
    string garbage;

    for (size_t layId = 0; layId < _db.numLayers(); ++layId) {
        data = toLineBegin(".Shape");
        ss.str(data);
        ss.ignore(numeric_limits<streamsize>::max(), '$');
        string layName;
        ss >> layName;

        getline(_fin, data);
        ss.str(data);
        bool isShape = false;
        if(data.substr(0,7) == "Polygon" || data.substr(0,6) == "Circle") {
            isShape = true;
        } else {
            isShape = false;
            cerr << "ERROR! Undefined Shape" << endl;
        }
        while(isShape) {
            Shape* shape;
            if(data.substr(0,7) == "Polygon") {
                string netName;
                stringstream sNetName;
                ss.ignore(numeric_limits<streamsize>::max(), ':');
                ss.ignore(numeric_limits<streamsize>::max(), ':');
                ss >> netName;
                // Color = darkgreen
                ss >> garbage;
                ss >> garbage;
                ss >> garbage;
                // vertices positions
                vector< pair<double, double> > vVtx;
                double x, y;
                while (ss.peek() >= ' ') {
                    // cerr << ss.peek() << endl;
                    x = extractDouble(ss, 2);
                    y = extractDouble(ss, 2);
                    // cerr << "vtx = (" << x << ", " << y << ") " << endl;
                    vVtx.push_back(make_pair(x,y));
                }
                // cerr << endl;
                getline(_fin, data);
                ss.str(data);
                while (data[0] == '+') {
                    // cerr << "+ " << ends;
                    ss >> garbage;
                    while (ss.peek() >= ' ') {
                        x = extractDouble(ss, 2);
                        y = extractDouble(ss, 2);
                        // cerr << "vtx = (" << x << ", " << y << ") " << ends;
                        vVtx.push_back(make_pair(x,y));
                    }
                    // cerr << endl;
                    getline(_fin, data);
                    ss.str(data);
                }
                shape = new Polygon(vVtx, _plot);
                if (netName == "+VCCCORE+") {
                    shape->plot(SVGPlotColor::green, _layName2Id[layName]);
                } else if (netName == "+VCCGT+") {
                    shape->plot(SVGPlotColor::lightsalmon, _layName2Id[layName]);
                } else if (netName == "+VCCSA+") {
                    shape->plot(SVGPlotColor::purple, _layName2Id[layName]);
                } else {
                    // shape->plot(SVGPlotColor::gray, _layName2Id[layName]);
                } 
            } else if (data.substr(0,6) == "Circle") {
                string netName;
                stringstream sNetName;
                if (ss.str().find("::") != string::npos) {
                ss.ignore(numeric_limits<streamsize>::max(), ':');
                ss.ignore(numeric_limits<streamsize>::max(), ':');
                ss >> netName;
                // Color = darkgreen
                ss >> garbage;
                ss >> garbage;
                ss >> garbage;
                } else {
                    // circle with no net name
                    ss >> garbage;
                }
                // center position and radius
                double ctrX, ctrY, radius;
                // cerr << ss.str() << endl;
                ctrX = extractDouble(ss, 2);
                ctrY = extractDouble(ss, 2);
                radius = extractDouble(ss, 2);
                // construct circle
                shape = new Circle(ctrX, ctrY, radius, _plot);
                if (netName == "+VCCCORE+") {
                    shape->plot(SVGPlotColor::green, _layName2Id[layName]);
                } else if (netName == "+VCCGT+") {
                    shape->plot(SVGPlotColor::lightsalmon, _layName2Id[layName]);
                } else if (netName == "+VCCSA+") {
                    shape->plot(SVGPlotColor::purple, _layName2Id[layName]);
                } else {
                    // shape->plot(SVGPlotColor::gray, _layName2Id[layName]);
                }
                // new line
                getline(_fin, data);
                ss.str(data);
            }
            
            if(data.substr(0,7) == "Polygon" || data.substr(0,6) == "Circle") {
                isShape = true;
            } else {
                isShape = false;
                break;
            }
        }
    }
}

string Parser::parseNodeTrace() {
    string data;
    stringstream ss;
    string garbage;
    data = toLineBegin("Node");
    ss.str(data);
    // cerr << data << endl;
    while(data.substr(0,4) == "Node") {
        string nodeName;
        stringstream sNodeName;
        ss >> nodeName;
        sNodeName.clear();
        sNodeName.str(nodeName);
        getline(sNodeName, nodeName, ':');
        nodeName.erase(0,4);
        // cerr << "nodeName = " << nodeName << endl;
 
        double x, y;
        ss >> garbage;
        assert(garbage == "X");
        ss >> garbage;
        assert(garbage == "=");
        x = extractDouble(ss, 2);
        // cerr << " x = " << x << endl;
        ss >> garbage;
        assert(garbage == "Y");
        ss >> garbage;
        assert(garbage == "=");
        y = extractDouble(ss, 2);
        // cerr << " y = " << y << endl;

        string layName;
        ss >> garbage;
        assert(garbage == "Layer");
        ss >> garbage;
        assert(garbage == "=");
        ss.ignore(numeric_limits<streamsize>::max(), '$');
        ss >> layName;
        // cerr << " layName = " << layName << endl;

        _db.addNode(nodeName, x, y, _layName2Id[layName]);

        getline(_fin, data);
        ss.str(data);
    }
    assert(data.substr(0,5) == "Trace");
    while (data.substr(0,5) == "Trace") {
        string netName;
        if (ss.str().find("::") != string::npos) {
            ss.ignore(numeric_limits<streamsize>::max(), ':');
            ss.ignore(numeric_limits<streamsize>::max(), ':');
            ss >> netName;
            // cerr << "netName = " << netName << endl;
        } else {
            // circle with no net name
            ss >> garbage;
        }
        
        string nodeSName, nodeTName;
        stringstream sNodeName;
        // starting node
        ss >> garbage;
        assert (garbage == "StartingNode");
        ss >> garbage;
        assert (garbage == "=");
        ss >> nodeSName;
        sNodeName.clear();
        sNodeName.str(nodeSName);
        getline(sNodeName, nodeSName, ':');
        nodeSName.erase(0,4);
        // cerr << "nodeSName = " << nodeSName << endl;
        // ending node
        ss >> garbage;
        assert (garbage == "EndingNode");
        ss >> garbage;
        assert (garbage == "=");
        ss >> nodeTName;
        sNodeName.clear();
        sNodeName.str(nodeTName);
        getline(sNodeName, nodeTName, ':');
        nodeTName.erase(0,4);
        // cerr << "nodeTName = " << nodeTName << endl;
        // width
        double width;
        ss >> garbage;
        assert (garbage == "Width");
        ss >> garbage;
        assert (garbage == "=");
        width = extractDouble(ss, 2);
        // cerr << "width = " << width << endl;
        Shape* shape = new Trace(_db.vDBNode(nodeSName)->node(), _db.vDBNode(nodeTName)->node(), width, _plot);
        if (netName == "+VCCCORE+") {
            shape->plot(SVGPlotColor::green, _db.vDBNode(nodeSName)->layId());
        } else if (netName == "+VCCGT+") {
            shape->plot(SVGPlotColor::lightsalmon, _db.vDBNode(nodeSName)->layId());
        } else if (netName == "+VCCSA+") {
            shape->plot(SVGPlotColor::purple, _db.vDBNode(nodeSName)->layId());
        } else {
            // shape->plot(SVGPlotColor::black, _db.vDBNode(nodeSName)->layId());
        }

        getline(_fin, data);
        ss.str(data);

    }
    return data;
}

void Parser::parseVia(string data) {
    stringstream ss;
    string garbage;
    ss.str(data);
    while (data.substr(0,3) == "Via") {
        string netName;
        if (ss.str().find("::") != string::npos) {
            ss.ignore(numeric_limits<streamsize>::max(), ':');
            ss.ignore(numeric_limits<streamsize>::max(), ':');
            ss >> netName;
            // cerr << "netName = " << netName << endl;
        } else {
            // via with no net name
            ss >> garbage;
        }
        string nodeUName, nodeLName;
        stringstream sNodeName;
        // starting node
        ss >> garbage;
        assert (garbage == "UpperNode");
        ss >> garbage;
        assert (garbage == "=");
        ss >> nodeUName;
        sNodeName.clear();
        sNodeName.str(nodeUName);
        getline(sNodeName, nodeUName, ':');
        nodeUName.erase(0,4);
        // cerr << "nodeUName = " << nodeUName << endl;
        // ending node
        ss >> garbage;
        assert (garbage == "LowerNode");
        ss >> garbage;
        assert (garbage == "=");
        ss >> nodeLName;
        sNodeName.clear();
        sNodeName.str(nodeLName);
        getline(sNodeName, nodeLName, ':');
        nodeLName.erase(0,4);
        // cerr << "nodeLName = " << nodeLName << endl;
        ss >> garbage;
        assert(garbage == "Color");
        ss >> garbage;
        assert(garbage == "=");
        ss >> garbage;
        ss >> garbage;
        assert(garbage == "PadStack");
        ss >> garbage;
        assert(garbage == "=");
        string padStackName;
        ss >> padStackName;
        // cerr << "padStackName = " << padStackName << endl;

        _db.addViaEdge(netName, nodeUName, nodeLName, padStackName);

        getline(_fin, data);
        ss.str(data);
    }
}

string Parser::toLineBegin(string word) {
    string data;
    getline(_fin, data);
    // stringstream ss;
    // ss.str(data);
    string word1;
    word1.assign(data, 0, word.size());
    while (word1 != word) {
        _fin.ignore(numeric_limits<streamsize>::max(), '\n');
        getline(_fin, data);
        // cerr << "data = " << data << endl;
        // ss.str("");
        // ss.str(data);
        // cerr << "ss = " << ss.str() << endl;
        word1.clear();
        word1.assign(data, 0, word.size());
        // ss >> word;
        // cerr << "word = " << word << endl;
    }
    return data;
}

double Parser::extractDouble(stringstream& ss, int eraseLength) {
    string word;
    ss >> word;
    for (int i = 0; i < eraseLength; ++ i) {
        word.pop_back();
    }
    return stod(word);
}