#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#define CHECKPOINT_RADIUS 600
#define MAX_THRUST 100
#define MAX_ANGLE_ALLOWED_BOOST 10
#define MIN_DISTANCE_ALLOWED_BOOST 6000
#define SHIP_RADIUS 400
#define CLOSE_PADDING 100
#define PI 3.14159265
#define TARGET_AHEAD_DISTANCE 5000

using namespace std;

struct Point2D {
    int x, y;

    Point2D() = default;

    Point2D(int x, int y) : x(x), y(y) {}

    bool operator==(const Point2D &p) const {
        return x == p.x && y == p.y;
    }

    bool operator!=(const Point2D &p) const {
        return x != p.x || y != p.y;
    }
};

inline double angle(const Point2D& a, const Point2D& b) {
    return atan2(a.y - b.y, a.x - b.x) * 180 / PI;
}

inline double length(const Point2D& a, const Point2D& b) {
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

struct Ship2D {
    int vx, vy, angle, cpId;
    Point2D pos;

    Ship2D() = default;

    Ship2D(int x, int y, int vx, int vy, int angle, int cpId) : vx(vx), vy(vy), angle(angle), cpId(cpId) {
        pos = Point2D(x, y);
    }
};

class Track {
public:
    bool allCheckpointsFound = false;

    void onNewCheckpoint(Point2D point) {
        /*for (int i = 0; i < checkpoints.size(); i++) {
            cerr << "Cp: " << i << " " << checkpoints[i].x << " - " << checkpoints[i].y << endl;
        }*/
        if (allCheckpointsFound) return;

        if (checkpoints.empty()) {
            checkpoints.push_back(point);
        } else if (checkpoints.back() != point) {
            checkpoints.push_back(point);
            if (checkpoints.front() == point) {
                allCheckpointsFound = true;
            }
            cerr << "New point: " << checkpoints.back().x << " - " << checkpoints.back().y;
        }
    }

    /**
     * @param x ship x
     * @param y ship y
     * @param currentCP
     * @return - means left, + means right
     */
    double angleToNextCp(Point2D& ship, Point2D& currentCP, Point2D& nextCp) {
        double pointsAngle = angle(currentCP, nextCp);
        double shipAngle = angle(ship, currentCP);
        double angle = pointsAngle - shipAngle;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 260;
        return angle;
    }

    int getCPIndex(Point2D &cp) {
        if (checkpoints.empty()) return -1;
        auto itr = find_if(checkpoints.cbegin(), checkpoints.cend(), [&cp](Point2D const &ca) { return ca == cp; });
        if (itr != checkpoints.cend())
            return std::distance(checkpoints.cbegin(), itr);
        return -1;
    }

    Point2D getCp(int index) {
        return checkpoints[index];
    }

    double calcOptimalAngle(int distance, double nextAngle) {
        // x*x*x/8000 - x*x/60 + x/6+ 30
        if (abs(nextAngle) > 160) nextAngle = nextAngle > 0 ? 160 : -160;
        distance = distance / 100 - 70;
        if (distance > 75) distance = 75;
        cerr << "Optimal angle x: " << distance << endl;
        return -nextAngle * (distance * distance * distance / 6000.0 - distance * distance / 60.0 + distance / 6.0 + 30) / 170;
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-narrowing-conversions"

    Point2D calcOptimalTarget(Ship2D& ship) {
        Point2D currentCP = getCp(ship.cpId);
        Point2D nextCP = getCp((ship.cpId + 1) % checkpoints.size());
        double angle_ = angle(ship.pos, currentCP); // Reverse?

        double optimalAngle = calcOptimalAngle(length(ship.pos, currentCP), angleToNextCp(ship.pos, currentCP, nextCP));
        cerr << "CP angle: " << angle_ << endl;
        cerr << "Optimal angle: " << optimalAngle << endl;
        double radAngle = (optimalAngle + angle_) / 180 * PI;
        cerr << "DX: " << TARGET_AHEAD_DISTANCE * cos(radAngle) << endl;
        cerr << "DY: " << TARGET_AHEAD_DISTANCE * sin(radAngle) << endl;
        return Point2D(ship.pos.x - TARGET_AHEAD_DISTANCE * cos(radAngle), ship.pos.y - TARGET_AHEAD_DISTANCE * sin(radAngle));
    }

#pragma clang diagnostic pop

private:
    vector<Point2D> checkpoints;
};

bool canRam(int x, int y, int checkX, int checkY, int opponentX, int opponentY, int angle) {
    bool opponentInFront = abs(opponentX - checkX) < abs(x - checkX) && abs(opponentY - checkY) < abs(y - checkY);
    cerr << " In front:" << opponentInFront;
    bool opponentClose = abs(opponentY - y) < 600 && abs(opponentX - x) < 600;
    cerr << " close: " << opponentClose;
    bool opponentCloseToCheck =
            abs(opponentY - checkY) < CHECKPOINT_RADIUS && abs(opponentX - checkX) < CHECKPOINT_RADIUS;
    cerr << " close to check: " << opponentCloseToCheck;
    cerr << " angle: " << angle << endl;
    return opponentInFront && opponentClose && opponentCloseToCheck && abs(angle) < 30;
}

bool opponentClose(int x, int y, int opponentX, int opponentY) {
    return sqrt((x - opponentX) * (x - opponentX) + (y - opponentY) * (y - opponentY)) <
           SHIP_RADIUS * 2 + CLOSE_PADDING;
}


/**
 * This code automatically collects game data in an infinite loop.
 * It uses the standard input to place data into the game variables such as x and y.
 * YOU DO NOT NEED TO MODIFY THE INITIALIZATION OF THE GAME VARIABLES.
 **/
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    Track track;

    int lapsCount;
    int checkpointsCount;
    cin >> lapsCount >> checkpointsCount;
    int cX, cY;
    for (int i = 0; i < checkpointsCount;i++) {
        cin >> cX >> cY;
        track.onNewCheckpoint(Point2D(cX, cY));
    }

    Ship2D ship1, ship2, opponent1, opponent2;

    Point2D target;
    Point2D cp;
    string thrust;

    // game loop
    while (1) {
        int x, y, vx, vy, angle, cpId;
        cin >> x >> y >> vx >> vy>> angle >> cpId;
        ship1 = Ship2D(x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy>> angle >> cpId;
        ship2 = Ship2D(x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy>> angle >> cpId;
        opponent1 = Ship2D(x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy>> angle >> cpId;
        opponent2 = Ship2D(x, y, vx, vy, angle, cpId);

        thrust = to_string(MAX_THRUST);

        target = track.calcOptimalTarget(ship1);
        cout << target.x <<  " "<< target.y << " " << thrust << endl;


        thrust = to_string(MAX_THRUST);
        /*if (abs(ship2.angle) > 80) {
            thrust = "10";
        }*/
        target = track.calcOptimalTarget(ship2);
        cout << target.x <<  " "<< target.y << " " << thrust << endl;

    }
}

#pragma clang diagnostic pop