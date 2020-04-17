#include <iostream>
#include <vector>
#include <cmath>

#define CHECKPOINT_RADIUS 600
#define POD_RADIUS 400
#define MAX_CP_DISTANCE 16000

#define MAX_THRUST 100
#define BOOST_THRUST 650
#define SHIELD_COOL_DOWN 3
#define DRAG 0.85

#define MAX_ANGLE_ALLOWED_BOOST 15
#define MIN_DISTANCE_ALLOWED_BOOST 6000
#define TARGET_AHEAD_DISTANCE 5000
#define MIN_SHIELD_IMPULSE 200

#define SIMULATION_CHILD_COUNT 5
#define SIMULATION_DEPTH 5
#define SIMULATION_ANGLE_DIFF 50

#define PI 3.14159265

using namespace std;

struct Vector2D {
    double x, y;

    Vector2D() = default;

    Vector2D(double x, double y) : x(x), y(y) {}

    bool operator==(const Vector2D &v) const {
        return x == v.x && y == v.y;
    }

    bool operator!=(const Vector2D &v) const {
        return x != v.x || y != v.y;
    }

    Vector2D operator+(const Vector2D &v) const {
        return {x + v.x, y + v.y};
    }

    Vector2D operator-(const Vector2D &v) const {
        return {x - v.x, y - v.y};
    }
};

inline double angle(const Vector2D &a, const Vector2D &b) {
    return atan2(b.y - a.y, b.x - a.x) * 180 / PI;
}

inline double angle(const Vector2D &a) {
    return atan2(a.y, a.x) * 180 / PI;
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
    int cpId;
    double angle;
    Vector2D pos, velocity;
    int thrust = MAX_THRUST;
    bool boostUsed = false;
    int shieldCoolDown = 0;
    Vector2D target = Vector2D(0, 0);

    Ship2D() : pos(Vector2D(0, 0)), velocity(Vector2D(0, 0)), angle(0), cpId(0) {};

    Ship2D(const Vector2D &pos,
           const Vector2D &velocity,
           double angle,
           int cpId,
           int thrust,
           bool boostUsed,
           int shieldCoolDown,
           const Vector2D &target) :
            angle(angle),
            cpId(cpId),
            pos(pos),
            velocity(velocity),
            thrust(thrust),
            boostUsed(boostUsed),
            shieldCoolDown(shieldCoolDown),
            target(target) {}
};

class Track {
public:
    int lapsCount = 3;
    int checkpointsCount = 3;

    void addNewCheckpoint(const Vector2D point) {
        checkpoints.push_back(point);
        cerr << "New point: " << checkpoints.back().x << " - " << checkpoints.back().y;
    }

    Vector2D getCp(const unsigned index) {
        return checkpoints[index % checkpoints.size()];
    }

    static double calcTurnAngle(const Vector2D &ship, const Vector2D &currentCP, const Vector2D &nextCp) {
        // double pointAngle = angle(ship, currentCP);
        // double nextAngle = angle(currentCP, nextCp);
        return normalize_angle(angle(currentCP, nextCp) - angle(ship, currentCP));
    }

    static Vector2D calcOptimalCpPos(const Ship2D *ship, const Vector2D &cp, const Vector2D &nextCp) {
        Vector2D medianPoint = Vector2D((ship->pos.x + nextCp.x), (ship->pos.y + nextCp.y) / 2);
        double radAngleFromCp = angle(cp, medianPoint) / 180 * PI;
        return {cp.x + CHECKPOINT_RADIUS * 0.5 * cos(radAngleFromCp),
                cp.y + CHECKPOINT_RADIUS * 0.5 * radAngleFromCp};
    }

    static double calcOptimalAngleOffset(const Vector2D speed, const Vector2D &pos, const Vector2D &cp, const Vector2D &nextCP) {
        double x = length(pos, cp) / 150; // Map 0 - 15000 to 0 - 100
        // cerr << "x " << x << endl;
        double turnAngle = calcTurnAngle(pos, cp, nextCP);
        // cerr << "a " << turnAngle << endl;
        // log(x + 1) * 80 - sqrt(x) * 16
        // log((x -10)/3 + 4) * 75 - sqrt((x)/2) * 16
        // return -(log10((x - 10) / 3 + 4) * 70 - sqrt((x + 2) / 2) * 16 + x / 10) * turnAngle / 50;

        // 20 * cos(log(x + 36) * 8) - x/18
        // return 20 * cos(log10(x + 36) * 8) - x / 18 * turnAngle / 100;

        // (x-50)*(x-50)*(x-50)/6000 - (x-50)*(x-50)/60 + (x-50)/6+ 30
        return -((x - 60) * (x - 60) * (x - 60) / 6000 - (x - 60) * (x - 60) / 60 + (x - 60) / 6 + 30) * turnAngle /
               180;
    }

    static double calcRelativeCollisionImpulse(const Vector2D *base, const Vector2D *v) {
        //cerr << "Ipulse " << sin(angle(*base, *v) / 180 * PI) * length(*v) << endl;
        return sin(angle(*base, *v) / 180 * PI) * length(*v); // TODO: Validate if calculation is correct
    }

    double calcOptimalTargetAngle(const Ship2D *ship) {
        Vector2D nextCP = getCp(ship->cpId + 1);
        Vector2D tmpCP = getCp(ship->cpId);
        Vector2D currentCP = calcOptimalCpPos(ship, tmpCP, nextCP);
        double angleToCP = angle(ship->pos, currentCP);
        double velocityAngle = atan2(ship->velocity.y, ship->velocity.x) * 180 / PI;

        double preferredAngleOffset = calcOptimalAngleOffset(ship->velocity, ship->pos, currentCP, nextCP);
        double targetAngle = normalize_angle(angleToCP + preferredAngleOffset);

        Vector2D newPos = Vector2D(ship->pos.x + ship->velocity.x, ship->pos.y + ship->velocity.y);

        double newAngleToCP = angle(newPos, currentCP);
        double newPreferredOffset = calcOptimalAngleOffset(ship->velocity, newPos, currentCP, nextCP);
        double newTargetAngle = normalize_angle(newAngleToCP + newPreferredOffset);
        // cerr << "Base target " << (targetAngle + newTargetAngle) / 2 << endl;
        return (targetAngle + newTargetAngle) / 2;
    }

    static Vector2D calculateTarget(const Ship2D *ship, const double &angle) {
        double radAngle = angle / 180 * PI;
        return {ship->pos.x + TARGET_AHEAD_DISTANCE * cos(radAngle),
                ship->pos.y + TARGET_AHEAD_DISTANCE * sin(radAngle)};
    }

    static void evaluateShield(Ship2D *ship, const Ship2D *opponent1, const Ship2D *opponent2, const Vector2D &cp) {
        Vector2D newPos = Vector2D(ship->pos.x + ship->velocity.x,
                                   ship->pos.y + ship->velocity.y);
        Vector2D newOpponet1 = Vector2D(opponent1->pos.x + opponent1->velocity.x,
                                        opponent1->pos.y + opponent1->velocity.y);
        Vector2D newOpponet2 = Vector2D(opponent2->pos.x + opponent2->velocity.x,
                                        opponent2->pos.y + opponent2->velocity.y);

        // Track::calcRelativeCollisionImpulse(&ship->velocity, &opponent1->velocity);

        if (length(newPos, newOpponet1) < POD_RADIUS * 3.4) {
            if (abs(normalize_angle(angle(ship->velocity) - angle(opponent1->velocity))) > 45
                && abs(normalize_angle(angle(ship->velocity) - angle(opponent1->velocity))) < 135
                && abs(Track::calcRelativeCollisionImpulse(&ship->velocity, &opponent1->velocity)) > MIN_SHIELD_IMPULSE
                && length(newPos, cp) < length(newOpponet1, cp) - POD_RADIUS / 2.0) {
                cerr << "Shield for 1!" << endl;
                ship->shieldCoolDown = SHIELD_COOL_DOWN;
            }
        }

        if (length(newPos, newOpponet2) < POD_RADIUS * 3.4) {
            if (abs(normalize_angle(angle(ship->velocity) - angle(opponent2->velocity))) > 45
                && abs(normalize_angle(angle(ship->velocity) - angle(opponent2->velocity))) < 135
                && abs(Track::calcRelativeCollisionImpulse(&ship->velocity, &opponent2->velocity)) > MIN_SHIELD_IMPULSE
                && length(newPos, cp) < length(newOpponet2, cp) - POD_RADIUS / 2.0) {
                ship->shieldCoolDown = SHIELD_COOL_DOWN;
                cerr << "Shield for 1!" << endl;
            }
        }
    }

    void evaluateBoost(Ship2D *ship, Ship2D *opponent1, Ship2D* opponent2) {
        double effectiveImpulse = length(ship->pos, getCp(ship->cpId)) / length(ship->velocity) / (1 - DRAG) *
                                  cos((angle(ship->velocity) - angle(ship->pos, getCp(ship->cpId))) / 180 * PI);
        cerr << "Eff impulse: " << effectiveImpulse << endl;
        //cerr << "Turn: " << Track::calcTurnAngle(ship->pos, getCp(ship->cpId), getCp(ship->cpId + 1)) << endl;
        Vector2D cp = getCp(ship->cpId);

        cerr << angle(ship->pos, getCp(ship->cpId + 1)) << " - " << angle(ship->pos, cp) << " - " << angle(ship->velocity) << endl;

        if (!ship->boostUsed && length(ship->pos, getCp(ship->cpId)) > MIN_DISTANCE_ALLOWED_BOOST
            && abs(normalize_angle(ship->angle - angle(ship->pos, getCp(ship->cpId)))) < MAX_ANGLE_ALLOWED_BOOST) {
            ship->thrust = BOOST_THRUST;
            ship->boostUsed = true;
        } else if (abs(effectiveImpulse) < 30
                   && length(ship->pos, opponent1->pos) > 3000
                   && length(ship->pos, opponent2->pos) > 3000
                   && abs(Track::calcTurnAngle(ship->pos, cp, getCp(ship->cpId + 1))) > 45
                   && ((angle(ship->pos, getCp(ship->cpId + 1)) > angle(ship->pos, cp) && angle(ship->pos, cp) > angle(ship->velocity))
                       ||  (angle(ship->pos, getCp(ship->cpId + 1)) < angle(ship->pos, cp) && angle(ship->pos, cp) < angle(ship->velocity)))) {

            cerr << "======= SKIPPING CP =======" << endl;
            ship->thrust = min(100.0, 2 * length(ship->pos, getCp(ship->cpId)) / length(ship->velocity) / (1 - DRAG));
            cerr << "Pod 1 thrust: " << ship->thrust << endl;
            ship->target = calcOptimalCpPos(ship, getCp(ship->cpId + 1), getCp(ship->cpId + 2));
        } else {
            ship->thrust = MAX_THRUST;
        }
    }

    void evaluateBoost2(Ship2D *ship, Ship2D *opponent1, Ship2D *opponent2) {
        double effectiveImpulse = length(ship->pos, getCp(ship->cpId)) / length(ship->velocity) / (1 - DRAG) *
                                  cos((angle(ship->velocity) - angle(ship->pos, getCp(ship->cpId))) / 180 * PI);
        cerr << "Eff impulse: " << effectiveImpulse << endl;
        //cerr << "Turn: " << Track::calcTurnAngle(ship->pos, getCp(ship->cpId), getCp(ship->cpId + 1)) << endl;
        Vector2D cp = getCp(ship->cpId);

        double nextCpAngle = angle(ship->pos, getCp(ship->cpId + 1));
        double cpAngle = angle(ship->pos, cp);
        double velocityAngle = angle(ship->velocity);
        cerr << nextCpAngle << " - " << cpAngle << " - " << velocityAngle << endl;

        if (!ship->boostUsed && length(ship->pos, getCp(ship->cpId)) > MIN_DISTANCE_ALLOWED_BOOST
            && abs(normalize_angle(ship->angle - angle(ship->pos, getCp(ship->cpId)))) < MAX_ANGLE_ALLOWED_BOOST) {
            ship->thrust = BOOST_THRUST;
            ship->boostUsed = true;
        } else if (abs(effectiveImpulse) < 30
                   && length(ship->pos, opponent1->pos) > 2000
                   && length(ship->pos, opponent2->pos) > 2000
                   && abs(Track::calcTurnAngle(ship->pos, cp, getCp(ship->cpId + 1))) > 45
                   && normalize_angle(nextCpAngle - velocityAngle) / normalize_angle(cpAngle - velocityAngle) > 0) {

            cerr << "======= SKIPPING CP =======" << endl;
            ship->thrust = min(100.0, 2 * length(ship->pos, getCp(ship->cpId)) / length(ship->velocity) / (1 - DRAG));
            cerr << "Pod 1 thrust: " << ship->thrust << endl;
            ship->target = calcOptimalCpPos(ship, getCp(ship->cpId + 1), getCp(ship->cpId + 2));
        } else {
            ship->thrust = MAX_THRUST;
        }
    }

private:
    vector<Vector2D> checkpoints;
};

class SimulationNode {
public:
    Track *track;
    Ship2D pod1, pod2, opponent1, opponent2;
    double pod1BestAngle = 0, pod2BestAngle = 0;
    double score = 0;

    SimulationNode() : track(nullptr) {}

    SimulationNode(Track *track,
                   const Ship2D &pod1,
                   const Ship2D &pod2,
                   const Ship2D &opponent1,
                   const Ship2D &opponent2) :
            track(track),
            pod1(pod1),
            pod2(pod2),
            opponent1(opponent1),
            opponent2(opponent2) {}

    SimulationNode evaluate(int depth = SIMULATION_DEPTH) {
        if (depth <= 0) {
            score = getNodeScore(this);
            return *this;
        }

        double pod1BaseAngle = track->calcOptimalTargetAngle(&pod1);

        double pod1Angle;
        // cerr << depth << " Cpids: " << pod1.cpId << " - " << pod2.cpId << endl;

        SimulationNode node, best;

        for (short d1 = -SIMULATION_ANGLE_DIFF / 2;
             d1 <= SIMULATION_ANGLE_DIFF / 2; d1 += SIMULATION_ANGLE_DIFF / (SIMULATION_CHILD_COUNT - 1)) {

            // cerr << d1 << endl;
            pod1Angle = pod1BaseAngle + d1;
            Ship2D newPod1 = calculateNewPodLocation(&pod1, pod1Angle);
            Ship2D newPod2 = calculateNewPodLocation(&pod2, angle(pod2.velocity));

            Ship2D newOpponent1 = calculateNewPodLocation(&opponent1, angle(opponent2.velocity));
            Ship2D newOpponent2 = calculateNewPodLocation(&opponent2, angle(opponent2.velocity));

            // resolveCollisions(&newPod1, &newPod2, &newOpponent1, &newOpponent2);

            node = SimulationNode(track, newPod1, newPod2, opponent1, opponent2).evaluate(depth - 1);

            if (node.score > score) {
                // cerr << "Depth " << depth << endl;
                // cerr << "Pod1 angle " << pod1Angle;
                score = node.score;
                best = node;
                // We need pod angles from here, not from leaf node
                best.pod1BestAngle = pod1Angle;
            }

        }
        return best;
    }

    Ship2D calculateNewPodLocation(Ship2D *pod, double podAngle) {
        podAngle = podAngle / 180 * PI;
        Vector2D newVelocity = Vector2D((pod->velocity.x + cos(podAngle) * pod->thrust) * DRAG,
                                        (pod->velocity.y + sin(podAngle) * pod->thrust) * DRAG);
        Vector2D newPos = Vector2D(pod->pos.x + newVelocity.x, pod->pos.y + newVelocity.y); // Use average instead?
        int newCpId = length(pod->pos, track->getCp(pod->cpId)) < CHECKPOINT_RADIUS ? pod->cpId + 1 : pod->cpId;
        double newAngle = angle(newPos, track->getCp(newCpId));
        return {newPos, newVelocity, newAngle, newCpId, pod->thrust, pod->boostUsed, pod->shieldCoolDown - 1,
                pod->target}; // Remove target?
    }

private:
    void resolveCollisions(Ship2D *pod1, Ship2D *opponent1, Ship2D *opponent2) {
        // TODO: some calculations
    }

    double getNodeScore(SimulationNode *node) {
        int pod1CpId = node->pod1.cpId;

        double pod1CpDistance = MAX_CP_DISTANCE - length(node->pod1.pos, track->getCp(pod1CpId));

        pod1CpDistance += pod1CpId * MAX_CP_DISTANCE;
        // TODO: Subtract opponents
        return pod1CpDistance;
    }
};

class AI {
public:
    Track track = Track();

    Ship2D pod1 = Ship2D();
    Ship2D pod2 = Ship2D();
    Ship2D opponent1 = Ship2D();
    Ship2D opponent2 = Ship2D();

    AI() = default;

    void updatePod(Ship2D *pod, const int &x, const int &y, const int &vx, const int &vy, const int &angle, const int &cpId) {
        pod->pos = Vector2D(x, y);
        pod->velocity = Vector2D(vx, vy);
        pod->angle = angle;
        if (cpId != pod->cpId % track.checkpointsCount) {
            pod->cpId++;
            // cerr << "! ============= CP =============== !" << endl;
        }
    }

    void plan() {
        pod1.shieldCoolDown--;
        pod2.shieldCoolDown--;

        SimulationNode root = SimulationNode(&track, pod1, pod2, opponent1, opponent2);
        SimulationNode best = root.evaluate();

        pod1.target = Track::calculateTarget(&pod1, best.pod1BestAngle);

        Track::evaluateShield(&pod1, &opponent1, &opponent2, track.getCp(pod1.cpId));
        track.evaluateBoost2(&pod1, &opponent1, &opponent2);

        //Track::evaluateShield(&pod2, &opponent1, &opponent2, track.getCp(pod2.cpId));
        //track.evaluateBoost(&pod2);
        planRam(&pod2);
    }

private:
    bool isReadyToRam = false;
    bool isRamming = false;

    void planRam(Ship2D *pod) {
        Ship2D opponentToRam = opponent1;
        Vector2D cp = track.getCp(opponent1.cpId);
        if (opponent2.cpId > opponent1.cpId ||
            (opponent1.cpId == opponent2.cpId && length(opponent1.pos, cp) > length(opponent2.pos, cp) * 1.2)) {
            opponentToRam = opponent2;
            cp = track.getCp(opponent2.cpId);
        }

        if (!isRamming) {
            // Setup
            Vector2D torrentCp = track.getCp(opponentToRam.cpId);
            if (length(pod->pos, torrentCp) * 1.5 < length(opponentToRam.pos, torrentCp)) {
                cerr << "Lets just ram!" << endl;
                isRamming = true;
            } else {
                torrentCp = track.getCp(opponentToRam.cpId + 1);
            }

            if (length(pod->pos, torrentCp) > length(opponentToRam.pos, cp) + length(cp, torrentCp)) {
                cerr << "Gotta take shortcut" << endl;
                torrentCp = track.getCp(opponentToRam.cpId + 2);
            }

            // Drive to torrent cp
            pod->target = torrentCp;

            // Geometric array sum
            if (length(pod->pos, torrentCp) > length(pod->velocity) / (1 - DRAG)) {
                pod->thrust = MAX_THRUST;
            } else {
                pod->thrust = 0;
            }
            cerr << "Dist till cp: " << length(pod->pos, torrentCp) << endl;
            if (length(pod->pos, torrentCp) < CHECKPOINT_RADIUS * 2) {
                cerr << "Lets go ram!" << endl;
                isRamming = true;
            }
        } else {
            // Ram
            Vector2D target = Vector2D(opponentToRam.pos.x + opponentToRam.velocity.x * 2,
                                       opponentToRam.pos.y + opponentToRam.velocity.y * 2);
            pod->target = target;
            pod->thrust = MAX_THRUST;
            if (abs(normalize_angle(pod->angle - angle(pod->pos, target))) > 45) {
                pod->thrust = 0;
            }

            cerr << "angle " << pod->angle << " vs " << angle(pod->pos, target) << endl;
            cerr << "Dist till collision" << length(pod->pos, opponentToRam.pos) << endl;

            if (length(pod->pos, opponentToRam.pos) < POD_RADIUS * 2 * 1.8) {
                cerr << "SHIELD!" << endl;
                pod->shieldCoolDown = SHIELD_COOL_DOWN;
                isRamming = false;
            }
        }
    }

    void planRam2(Ship2D *pod) {
        Ship2D opponentToRam = opponent1;
        Vector2D cp = track.getCp(opponentToRam.cpId);
        if (opponent2.cpId > opponentToRam.cpId ||
            (opponentToRam.cpId == opponent2.cpId && length(opponentToRam.pos, cp) > length(opponent2.pos, cp) * 1.2)) {
            opponentToRam = opponent2;
        }
        Vector2D target = Vector2D(opponentToRam.pos.x + opponentToRam.velocity.x * 5,
                                   opponentToRam.pos.y + opponentToRam.velocity.y * 5);
        cp = track.getCp(opponentToRam.cpId);
        if (length(pod->pos, cp) - length(target, cp) < 4000) {
            pod->target = target;
        } else {
            pod->target = track.getCp(opponentToRam.cpId + 1);
        }
        if (length(pod->pos, target) < POD_RADIUS * 2 * 1.2) {
            cerr << "SHIELD!" << endl;
            pod->shieldCoolDown = SHIELD_COOL_DOWN;
        }
    }
};

void write_pod(const Ship2D &pod);

/**
 * This code automatically collects game data in an infinite loop.
 * It uses the standard input to place data into the game variables such as x and y.
 * YOU DO NOT NEED TO MODIFY THE INITIALIZATION OF THE GAME VARIABLES.
 **/
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    AI ai = AI();

    int lapsCount = 0;
    int checkpointsCount = 0;
    cin >> lapsCount >> checkpointsCount;
    ai.track.lapsCount = lapsCount;
    ai.track.checkpointsCount = checkpointsCount;
    int cX = 0, cY = 0;
    for (int i = 0; i < checkpointsCount; i++) {
        cin >> cX >> cY;
        ai.track.addNewCheckpoint(Vector2D(cX, cY));
    }

    int x = 0, y = 0, vx = 0, vy = 0, angle = 0, cpId = 0;


    // game loop
    while (true) {
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.pod1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.pod2, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.opponent1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.opponent2, x, y, vx, vy, angle, cpId);

        // cerr << "DT" << dt<<endl;

        ai.plan();

        write_pod(ai.pod1);
        write_pod(ai.pod2);

    }
}

void write_pod(const Ship2D &pod) {
    cout << (int) pod.target.x << " " << (int) pod.target.y << " ";
    if (pod.thrust > MAX_THRUST) {
        cout << "BOOST" << endl;
    } else if (pod.shieldCoolDown == SHIELD_COOL_DOWN) {
        cout << "SHIELD" << endl;
    } else {
        cout << pod.thrust << endl;
    }
}

#pragma clang diagnostic pop