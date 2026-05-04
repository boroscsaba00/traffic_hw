#include "veins/modules/application/traci/TraCIDemoRSU11p.h"

#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"

using namespace veins;

Define_Module(veins::TraCIDemoRSU11p);

namespace {

const char* TLS_ID = "J";

const bool USE_DYNAMIC_TRAFFIC_LIGHT = false;
// true  = adaptive traffic light
// false = fixed 20 s traffic light

const double MIN_GREEN = 8.0;
const double MAX_GREEN = 25.0;
const double FIXED_GREEN = 20.0;
const double YELLOW_TIME = 3.0;
const double QUEUE_SPEED_LIMIT = 1.0;

int dirToGreenPhase(int dir)
{
    if (dir == 0) return 0; // north
    if (dir == 1) return 4; // south
    if (dir == 2) return 2; // east
    return 6;               // west
}

int dirToYellowPhase(int dir)
{
    return dirToGreenPhase(dir) + 1;
}

std::string dirName(int dir)
{
    if (dir == 0) return "North";
    if (dir == 1) return "South";
    if (dir == 2) return "East";
    return "West";
}

std::string incomingEdge(int dir)
{
    if (dir == 0) return "N_in";
    if (dir == 1) return "S_in";
    if (dir == 2) return "E_in";
    return "W_in";
}

std::string outgoingEdge(int dir)
{
    if (dir == 0) return "S_out";
    if (dir == 1) return "N_out";
    if (dir == 2) return "W_out";
    return "E_out";
}

int edgeToDir(const std::string& edge)
{
    if (edge == "N_in") return 0;
    if (edge == "S_in") return 1;
    if (edge == "E_in") return 2;
    if (edge == "W_in") return 3;
    return -1;
}

int fixedTargetDir(simtime_t now)
{
    int cycleTime = int(4 * FIXED_GREEN);
    int t = int(now.dbl()) % cycleTime;

    // anti-clockwise: north -> west -> south -> east
    if (t < FIXED_GREEN) return 0;           // north
    if (t < 2 * FIXED_GREEN) return 3;       // west
    if (t < 3 * FIXED_GREEN) return 1;       // south
    return 2;                                // east
}

}

void TraCIDemoRSU11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {
        currentPhaseDir = 0;
        pendingPhaseDir = 0;
        yellowActive = false;

        minGreen = MIN_GREEN;
        maxGreen = MAX_GREEN;
        lastSwitch = simTime();

        controlEvt = new cMessage("controlEvt");
        scheduleAt(simTime() + 1, controlEvt);
    }
}

void TraCIDemoRSU11p::onWSA(DemoServiceAdvertisment* wsa)
{
    // not used
}

void TraCIDemoRSU11p::onWSM(BaseFrame1609_4* frame)
{
    delete frame;
}

void TraCIDemoRSU11p::handleSelfMsg(cMessage* msg)
{
    if (msg == controlEvt) {
        TraCIScenarioManager* manager = TraCIScenarioManagerAccess().get();

        if (!manager) {
            EV_ERROR << "Could not find TraCIScenarioManager" << endl;
            scheduleAt(simTime() + 1, controlEvt);
            return;
        }

        TraCICommandInterface* traci = manager->getCommandInterface();

        if (!traci) {
            EV_ERROR << "Could not get TraCICommandInterface" << endl;
            scheduleAt(simTime() + 1, controlEvt);
            return;
        }

        int queue[4] = {0, 0, 0, 0};
        int demand[4] = {0, 0, 0, 0};

        const std::map<std::string, cModule*>& hosts = manager->getManagedHosts();

        for (std::map<std::string, cModule*>::const_iterator it = hosts.begin(); it != hosts.end(); ++it) {
            std::string vehId = it->first;
            cModule* host = it->second;
            if (!host) continue;

            TraCIMobility* mob = nullptr;

            try {
                mob = TraCIMobilityAccess().get(host);
            }
            catch (...) {
                continue;
            }

            if (!mob) continue;

            std::string roadId;

            try {
                roadId = mob->getRoadId();
            }
            catch (...) {
                continue;
            }

            int inDir = edgeToDir(roadId);

            if (inDir >= 0) {
                demand[inDir]++;
                lastIncomingDir[vehId] = inDir;

                if (mob->getSpeed() < QUEUE_SPEED_LIMIT) {
                    queue[inDir]++;

                    bool hasGreen = (!yellowActive && currentPhaseDir == inDir);

                    if (!hasGreen) {
                        redWaitingTime[inDir] += 1.0;
                        totalRedWaitingTime += 1.0;
                    }
                }
            }
            else {
                if (lastIncomingDir.find(vehId) != lastIncomingDir.end()) {
                    int oldDir = lastIncomingDir[vehId];

                    if (roadId == outgoingEdge(oldDir)) {
                        passed[oldDir]++;
                        totalPassed++;
                        lastIncomingDir.erase(vehId);
                    }
                }
            }
        }

        int combinedQueue = queue[0] + queue[1] + queue[2] + queue[3];

        for (int i = 0; i < 4; i++) {
            if (queue[i] > maxQueue[i]) maxQueue[i] = queue[i];
            queueSum[i] += queue[i];
        }

        if (combinedQueue > maxCombinedQueue) maxCombinedQueue = combinedQueue;
        combinedQueueSum += combinedQueue;
        sampleCount++;

        if (yellowActive) {
            if (simTime() - lastSwitch >= YELLOW_TIME) {
                int greenPhase = dirToGreenPhase(pendingPhaseDir);
                traci->trafficlight(TLS_ID).setPhaseIndex(greenPhase);

                currentPhaseDir = pendingPhaseDir;
                yellowActive = false;
                lastSwitch = simTime();

                EV_INFO << "Yellow finished, switched to green phase "
                        << greenPhase << endl;
            }

            scheduleAt(simTime() + 1, controlEvt);
            return;
        }

        int targetDir = currentPhaseDir;

        if (!USE_DYNAMIC_TRAFFIC_LIGHT) {
            targetDir = fixedTargetDir(simTime());

            EV_INFO << "FIXED TLS mode | currentDir=" << currentPhaseDir
                    << " targetDir=" << targetDir << endl;
        }
        else {
            int bestDir = 0;
            int bestDemand = demand[0];

            if (demand[1] > bestDemand) {
                bestDemand = demand[1];
                bestDir = 1;
            }

            if (demand[2] > bestDemand) {
                bestDemand = demand[2];
                bestDir = 2;
            }

            if (demand[3] > bestDemand) {
                bestDemand = demand[3];
                bestDir = 3;
            }

            targetDir = bestDir;

            EV_INFO << "DYNAMIC TLS mode | Demand N=" << demand[0]
                    << " S=" << demand[1]
                    << " E=" << demand[2]
                    << " W=" << demand[3]
                    << " | Queue N=" << queue[0]
                    << " S=" << queue[1]
                    << " E=" << queue[2]
                    << " W=" << queue[3]
                    << " currentDir=" << currentPhaseDir
                    << " targetDir=" << targetDir
                    << endl;
        }

        simtime_t greenElapsed = simTime() - lastSwitch;

        bool shouldSwitch = false;

        if (!USE_DYNAMIC_TRAFFIC_LIGHT) {
            if (targetDir != currentPhaseDir) {
                shouldSwitch = true;
            }
        }
        else {
            if (greenElapsed >= minGreen) {
                if (targetDir != currentPhaseDir || greenElapsed >= maxGreen) {
                    shouldSwitch = true;
                }
            }
        }

        if (shouldSwitch) {
            int yellowPhase = dirToYellowPhase(currentPhaseDir);
            traci->trafficlight(TLS_ID).setPhaseIndex(yellowPhase);

            pendingPhaseDir = targetDir;
            yellowActive = true;
            lastSwitch = simTime();

            EV_INFO << "Started yellow phase " << yellowPhase
                    << " before switching to direction "
                    << pendingPhaseDir << endl;
        }

        scheduleAt(simTime() + 1, controlEvt);
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg);
}

void TraCIDemoRSU11p::finish()
{
    EV_INFO << endl;
    EV_INFO << "================ TRAFFIC LIGHT PERFORMANCE RESULTS ================" << endl;

    EV_INFO << "Mode: "
            << (USE_DYNAMIC_TRAFFIC_LIGHT ? "DYNAMIC" : "FIXED")
            << endl;

    EV_INFO << "Total vehicles passed junction: " << totalPassed << endl;
    EV_INFO << "Total waiting time at red/yellow [s]: " << totalRedWaitingTime << endl;

    EV_INFO << "---------------- Per direction ----------------" << endl;

    for (int i = 0; i < 4; i++) {
        double avgQueue = 0.0;

        if (sampleCount > 0) {
            avgQueue = queueSum[i] / double(sampleCount);
        }

        EV_INFO << dirName(i)
                << " | passed=" << passed[i]
                << " | red/yellow waiting time [s]=" << redWaitingTime[i]
                << " | max queue=" << maxQueue[i]
                << " | avg queue=" << avgQueue
                << endl;
    }

    double avgCombinedQueue = 0.0;

    if (sampleCount > 0) {
        avgCombinedQueue = combinedQueueSum / double(sampleCount);
    }

    EV_INFO << "---------------- Combined ----------------" << endl;
    EV_INFO << "Combined max queue=" << maxCombinedQueue << endl;
    EV_INFO << "Combined avg queue=" << avgCombinedQueue << endl;

    EV_INFO << "===================================================================" << endl;

    if (controlEvt) {
        cancelAndDelete(controlEvt);
        controlEvt = nullptr;
    }

    DemoBaseApplLayer::finish();
}
