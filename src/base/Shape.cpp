#include "Shape.h"

bool Shape::enclose(double x, double y) {
    // reference: https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/?fbclid=IwAR2lh7li1psci6NgZkXxFz7uOBKn_UamDEXLASI11RjdtXo3E7IpsUNLMdY
    
    struct Point {
        double x, y;
    };
 
    struct Line {
        Point p1, p2;
    };

    auto onLine = [] (Line l1, Point p) -> bool {
        // Check whether p is on the line or not
        if (p.x <= max(l1.p1.x, l1.p2.x)
            && p.x >= min(l1.p1.x, l1.p2.x)
            && (p.y <= max(l1.p1.y, l1.p2.y)
                && p.y >= min(l1.p1.y, l1.p2.y)))
            return true;
    
        return false;
    };

    auto direction = [] (Point a, Point b, Point c) -> int {
        int val = (b.y - a.y) * (c.x - b.x) - (b.x - a.x) * (c.y - b.y);
        if (val == 0) return 0;         // Collinear
        else if (val < 0) return 2;     // Anti-clockwise direction
        return 1;                       // Clockwise direction
    };

    auto isIntersect = [&] (Line l1, Line l2) -> bool {
        // Four direction for two lines and points of other line
        int dir1 = direction(l1.p1, l1.p2, l2.p1);
        int dir2 = direction(l1.p1, l1.p2, l2.p2);
        int dir3 = direction(l2.p1, l2.p2, l1.p1);
        int dir4 = direction(l2.p1, l2.p2, l1.p2);
    
        // When intersecting
        if (dir1 != dir2 && dir3 != dir4)
            return true;
    
        // When p2 of line2 are on the line1
        if (dir1 == 0 && onLine(l1, l2.p1))
            return true;
    
        // When p1 of line2 are on the line1
        if (dir2 == 0 && onLine(l1, l2.p2))
            return true;
    
        // When p2 of line1 are on the line2
        if (dir3 == 0 && onLine(l2, l1.p1))
            return true;
    
        // When p1 of line1 are on the line2
        if (dir4 == 0 && onLine(l2, l1.p2))
            return true;
    
        return false;
    };

    // When polygon has less than 3 edge, it is not polygon
    if (numBPolyVtcs() < 3)
        return false;
 
    // Create a point at infinity, y is same as point p
    Point p = {x, y};
    // Line exline = { p, { -1.0, 10000 } };
    Line exline = { p, { 1000, y } };
    int count = 0;
    int i = 0;
    do {
 
        // Forming a line from two consecutive points of
        // poly
        Point p1 = {bPolygonX(i), bPolygonY(i)};
        Point p2 = {bPolygonX((i + 1) % numBPolyVtcs()), bPolygonY((i + 1) % numBPolyVtcs())};
        Line side = {p1, p2};
        if (isIntersect(side, exline)) {
            // If side is intersects exline
            if (direction(side.p1, p, side.p2) == 0)
                return onLine(side, p);
            count++;
        }
        i = (i + 1) % numBPolyVtcs();
    } while (i != 0);
 
    // When count is odd
    return count & 1;
}