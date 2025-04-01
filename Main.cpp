
#include "rgb_depth_aligned.h"
#include <iostream>
#include "edge_detection.h"

int main() {
    using namespace std;
    int choice = 0;
    cout << "Select functionality to run:\n";
    cout << "1: DepthAI Edge Detection\n";
    cout << "2: RGB-Depth Alignment\n";
    cout << "Enter choice (1 or 2): ";
    cin >> choice;

    if (choice == 1) {
        runEdgeDetection();
    }
    else if (choice == 2) {
        runRGBDepth();
    }
    else {
        cout << "Invalid choice." << endl;
    }

    return 0;
}