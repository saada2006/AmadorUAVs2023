// Boring I/O code
#include <stdio.h>
void RedirectIO(const char* filename) {
    char buf[256];
    
    buf[0] = '\0';
    sprintf(buf, "%s.in", filename);
    freopen(buf, "r", stdin);


    buf[0] = '\0';
    sprintf(buf, "%s.out", filename);
    freopen(buf, "w", stdout);
}

#include <iostream>
#include <fstream>
#include <math.h>
#include <algorithm>

using namespace std;

// compare two doubles
bool equal(double a, double b) {
    // it is never a good idea to use == directly on two floating point values
    // instead, you should use |a-b|<elipson
    return (abs(a-b) < 1e-6);
}

// find the nearest coordinate on a specifc axis 
// note that that this function disregards rounding
int nearest(double offset, int lower, int top) {
    return (offset > 0.5 ? top : lower);
}

int main() {
    RedirectIO("find");

    int N, M, K;
    double dx, dy;
    std::cin >> N >> M >> dx >> dy >> K;

    for(int i = 0; i < K; i++) {
        double x, y;
        std::cin >> x >> y;

        // get our points within the grid
        double scaled_x = x / dx, scaled_y = y / dy;

        // Our coordinate lands in a "box" on the grid
        // We want to calculate the extents of this box, since one corner will be the point we pick

        int lower_x = (int)(scaled_x + 1e-5); // add elipson to prevent rounding errors for numbers like 1.999
        int top_x = lower_x + 1;
        
        int lower_y = (int)(scaled_y + 1e-5);
        int top_y = lower_y + 1;

        // calculate offset from lower left corner of the box
        double offset_x = scaled_x - lower_x;
        double offset_y = scaled_y - lower_y;

        // if we think about the rounding logic in the enterance assignment, we realize that rounding is not dependent on both axes
        // that is, we can evaluate rounding for one axis independently of the other
        // proof: if we are tied on the x axis (ie, you have to pick between multiple points), we will always pick the lower coordinate
        // but then the y-coordinate will be the same both points we had to round between, so it doesn't matter what we pick for x
        // you can expand on this idea for a more rigorous proof but that is general idea

        // check for ties on each axis 
        bool tie_x = equal(offset_x, 1 - offset_x);
        bool tie_y = equal(offset_y, 1 - offset_y);

        // evaluate each axis separately 
        int grid_x = (tie_x ? lower_x : nearest(offset_x, lower_x, top_x));
        int grid_y = (tie_y ? lower_y : nearest(offset_y, lower_y, top_y));

        // clamp our values
        grid_x = min(max(grid_x, 0), N - 1);
        grid_y = min(max(grid_y, 0), M - 1);

        // output our answer
        cout << grid_x << ' ' << grid_y << '\n';
    }

    return 0;
}