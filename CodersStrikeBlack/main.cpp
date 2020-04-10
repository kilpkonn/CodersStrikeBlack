#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

#define CHECKPOINT_RADIUS 600
#define MAX_THRUST 100
#define BOOST_THRUST 650
#define SHIELD_COOL_DOWN 3
#define MAX_ANGLE_ALLOWED_BOOST 10
#define MIN_DISTANCE_ALLOWED_BOOST 6000
#define SHIP_RADIUS 400
#define CLOSE_PADDING 100
#define PI 3.14159265
#define TARGET_AHEAD_DISTANCE 5000

using namespace std;

struct Vector2D {
    int x, y;

    Vector2D() = default;

    Vector2D(int x, int y) : x(x), y(y) {}

    bool operator==(const Vector2D &p) const {
        return x == p.x && y == p.y;
    }

    bool operator!=(const Vector2D &p) const {
        return x != p.x || y != p.y;
    }
};

inline double angle(const Vector2D &a, const Vector2D &b) {
    return atan2(b.y - a.y, b.x - a.x) * 180 / PI;
}

inline double length(const Vector2D &a, const Vector2D &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

inline double length(const Vector2D &a) {
    return sqrt(a.x * a.x + a.y * a.y);
}

inline double normalize_angle(double angle) {
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}

struct Ship2D {
    int angle, cpId;
    Vector2D pos, velocity;
    int thrust = MAX_THRUST;
    bool boostUsed = false;
    int shieldCoolDown = 0;
    Vector2D target = Vector2D(0, 0);

    Ship2D() : pos(Vector2D(0, 0)), velocity(Vector2D(0, 0)), angle(0), cpId(0) {};
};

class Track {
public:
    Ship2D pod1 = Ship2D();
    Ship2D pod2 = Ship2D();
    Ship2D opponent1 = Ship2D();
    Ship2D opponent2 = Ship2D();
    int lapsCount = 3;

    void updatePod(Ship2D* pod, int x, int y, int vx, int vy, int angle, int cpId) {
        pod->pos = Vector2D(x, y);
        pod->velocity = Vector2D(vx, vy);
        pod->angle = angle;
        pod->cpId = cpId;
    }

    void addNewCheckpoint(Vector2D point) {
        /*for (int i = 0; i < checkpoints.size(); i++) {
            cerr << "Cp: " << i << " " << checkpoints[i].x << " - " << checkpoints[i].y << endl;
        }*/
        checkpoints.push_back(point);
        cerr << "New point: " << checkpoints.back().x << " - " << checkpoints.back().y;
    }

    Vector2D getCp(unsigned index) {
        return checkpoints[index % checkpoints.size()];
    }

    double calcTurnAngle(Vector2D &ship, Vector2D &currentCP, Vector2D &nextCp) {
        double pointAngle = angle(ship, currentCP);
        // cerr << "x" << pointAngle << endl;
        double nextAngle = angle(currentCP, nextCp);
        // cerr << nextAngle << endl;
        double angle = nextAngle - pointAngle;
        return normalize_angle(angle);
    }

    double calcOptimalCpAngle(Ship2D &ship, Vector2D &cp, Vector2D &nextCp) {
        Vector2D medianPoint = Vector2D((ship.pos.x + nextCp.x), (ship.pos.y + nextCp.y) / 2);
        double radAngleFromCp = angle(cp, medianPoint) / 180 * PI;
        Vector2D newCp = Vector2D(cp.x + CHECKPOINT_RADIUS * cos(radAngleFromCp),
                                  cp.y + CHECKPOINT_RADIUS * radAngleFromCp);
        return angle(ship.pos, newCp);
    }

    double calcOptimalAngleOffset(Vector2D speed, Vector2D &pos, Vector2D &cp, Vector2D &nextCP) {
        double x = length(pos, cp) / 150; // Map 0 - 15000 to 0 - 100
        // cerr << "x " << x << endl;
        double turnAngle = calcTurnAngle(pos, cp, nextCP);
        // cerr << "a " << turnAngle << endl;
        // log(x + 1) * 80 - sqrt(x) * 16
        // log((x -10)/3 + 4) * 75 - sqrt((x)/2) * 16
        // return -(log10((x - 10) / 3 + 4) * 70 - sqrt((x + 2) / 2) * 16 + x / 10) * turnAngle / 50;

        // 20 * cos(log(x + 36) * 8) - x/18
        return 20 * cos(log10(x + 36) * 8) - x / 18 * turnAngle / 100;
    }

    void plan() {
        cerr << "============== SHIP 1 ==============" << endl;
        calcOptimalTarget(pod1);
        cerr << "============== SHIP 2 ==============" << endl;
        calcOptimalTarget(pod2);
    }

    void calcOptimalTarget(Ship2D &ship) {
        Vector2D currentCP = getCp(ship.cpId);
        Vector2D nextCP = getCp(ship.cpId + 1);
        double angleToCP = calcOptimalCpAngle(ship, currentCP, nextCP);
        double velocityAngle = atan2(ship.velocity.y, ship.velocity.x) * 180 / PI;
        double contVelAngle = normalize_angle(angleToCP - velocityAngle);
        // Fix standstill
        if (ship.velocity.x == 0 && ship.velocity.y == 0) contVelAngle = 0;

        double preferredAngleOffset = calcOptimalAngleOffset(ship.velocity, ship.pos, currentCP, nextCP);

        cerr << "Distance: " << length(currentCP, ship.pos) << endl;
        cerr << "Velocity angle: " << velocityAngle << endl;
        cerr << "Continuous vel angle:" << contVelAngle << endl;
        cerr << "CP angle: " << angleToCP << endl;
        cerr << "Pref angle offset: " << preferredAngleOffset << endl;
        double targetAngle = normalize_angle(angleToCP + preferredAngleOffset);
        cerr << "Target angle: " << targetAngle << endl;
        double error = normalize_angle(targetAngle - velocityAngle); // TODO: pid
        cerr << "Error: " << error << endl;
        double target = normalize_angle(targetAngle + error * 0.5);
        cerr << "Corrected target:  " << target << endl;

        if (contVelAngle == 0 || abs(error) > 25) {
            target = targetAngle;
        }

        Vector2D newPos = Vector2D(ship.pos.x + ship.velocity.x,
                                   ship.pos.y + ship.velocity.y); // Work on this

        double newAngleToCP = angle(newPos, currentCP);
        double newPreferredOffset = calcOptimalAngleOffset(ship.velocity, newPos, currentCP, nextCP);
        double newTargetAngle = normalize_angle(newAngleToCP + newPreferredOffset);

        cerr << "New target angle: " << newTargetAngle << endl;
        cerr << "Turn angle: " << newTargetAngle - velocityAngle << endl;

        if (contVelAngle != 0
            && abs(normalize_angle((newTargetAngle + target) / 2 - velocityAngle)) > 90
            && abs(normalize_angle(newTargetAngle + target) / 2 - velocityAngle) < 135 // Reverse lol
            && length(ship.velocity) > 150
            && length(ship.pos, currentCP) < 2500) { // speed

            ship.thrust = 0;
        } else {
            ship.thrust = MAX_THRUST;
        }
        double combinedTargetAngle = (target + newTargetAngle) / 2;
        cerr << "Combined target angle: " << combinedTargetAngle << endl;

        double radAngle = combinedTargetAngle / 180 * PI;
        ship.target= {(int) (ship.pos.x + TARGET_AHEAD_DISTANCE * cos(radAngle)),
                (int) (ship.pos.y + TARGET_AHEAD_DISTANCE * sin(radAngle))};
    }




private:
    vector<Vector2D> checkpoints;
};

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
    track.lapsCount = lapsCount;
    int cX, cY;
    for (int i = 0; i < checkpointsCount; i++) {
        cin >> cX >> cY;
        track.addNewCheckpoint(Vector2D(cX, cY));
    }

    int x, y, vx, vy, angle, cpId;


    // game loop
    while (1) {
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.pod1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.pod2, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.opponent1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.opponent2, x, y, vx, vy, angle, cpId);

        // cerr << "DT" << dt<<endl;

        track.plan();

        cout << track.pod1.target.x << " " << track.pod1.target.y << " ";
        if (track.pod1.thrust > MAX_THRUST) {
            cout << "BOOST" << endl;
        } else if (track.pod1.shieldCoolDown == SHIELD_COOL_DOWN) {
            cout << "SHIELD" << endl;
        } else {
            cout << track.pod1.thrust << endl;
        }

        cout << track.pod2.target.x << " " << track.pod2.target.y << " ";
        if (track.pod2.thrust > MAX_THRUST) {
            cout << "BOOST" << endl;
        } else if (track.pod2.shieldCoolDown == SHIELD_COOL_DOWN) {
            cout << "SHIELD" << endl;
        } else {
            cout << track.pod2.thrust << endl;
        }

    }
}

#pragma clang diagnostic pop