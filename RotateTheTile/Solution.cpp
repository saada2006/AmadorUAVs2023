// Boring redirect io code
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

// actual code begins here
#include <iostream>
#include <iomanip>
#include <math.h>

using namespace std;

#ifndef M_PI
#define M_PI 3.141529
#endif

int main() {
    RedirectIO("rotate");

    // round to 1 decimal place
    cout << fixed << setprecision(1);

    double h, v, l, x, y;
    cin >> h >> v >> l >> x >> y;

    // convert to radians
    h *= M_PI / 180;
    v *= M_PI / 180;

    // to derive these formulas, we can draw an isoceles triangle with the camera being at the vertex where the two congruent sides meet
    // we can then divide the triangle into two right triangles by bisecting the angle at that vertex
    // now, our trig functions become applicable and we can derive these formulas
    double z = l / (2 * tan(v / 2)); // z is the altitude 
    double width = 2 * z * tan(h / 2);
    cout << z << ' ' << width << ' ';
    
    // the idea is that the x-coordinate depends only on the pitch, and the yz plane only depends on the roll
    // from this, we can calculate the roll and pitch using inverse trig functions
    double roll = acos(y / sqrt(z * z + y * y)) * 180 / M_PI - 90; // subtract 90 to bring theta into the (-90, 90) set
    double pitch = asin(x / sqrt(x * x + y * y + z * z)) * 180 / M_PI;

    cout << roll << ' ' << pitch;

    return 0;
}