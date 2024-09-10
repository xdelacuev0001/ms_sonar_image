

// for projecting sonar data onto image

#include <iostream>
#include <cmath>

class SonarProjector {
public:

    // degrees for beamwidth
    SonarProjector() : verticalBeamwidth(25), horizontalBeamwidth(2) {}


    double getWidth(double distance) {
        double halfTheta = horizontalBeamwidth / 2.0;
        double width = 2 * sin(halfTheta * M_PI / 180) * distance; // converted to radians

        return width // in meters

    }

    double getHeight(double distance) {
        double halfTheta = verticalBeamwidth / 2.0;
        double height = 2 * sin(halfTheta * M_PI / 180) * distance; // converted to radians
        return height // in meters

    }



private:
    double verticalBeamwidth;
    double horizontalBeamwidth;
};

int main() {
    // Create an instance of SonarProjector
    SonarProjector projector;

    // Use the function to calculate something using sin
    projector.functon();

    return 0;
}