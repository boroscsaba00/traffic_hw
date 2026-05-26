#include "veins/modules/application/traci/TraCIDemoRSU11p.h"

#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"

#include <fstream>
#include <set>
#include <cstdlib>

using namespace veins;

Define_Module(veins::TraCIDemoRSU11p);

namespace {

const char* TLS_ID = "J";

const bool USE_DYNAMIC_TRAFFIC_LIGHT = false;
// true  = adaptive traffic light
// false = fixed 20 s traffic light

const char* EMISSIONS_FILE = "/home/veins/src/veins/examples/veins2/emissions.xml";

const double MIN_GREEN = 8.0;
const double MAX_GREEN = 25.0;
const double FIXED_GREEN = 20.0;
const double YELLOW_TIME = 3.0;
const double QUEUE_SPEED_LIMIT = 1.0;
const double EMISSION_STEP_LENGTH = 0.1; // seconds

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
    if (t < FIXED_GREEN) return 0;
    if (t < 2 * FIXED_GREEN) return 3;
    if (t < 3 * FIXED_GREEN) return 1;
    return 2;
}

double getXmlDoubleValue(const std::string& line, const std::string& key)
{
    std::string pattern = key + "=\"";
    size_t start = line.find(pattern);

    if (start == std::string::npos) return 0.0;

    start += pattern.length();
    size_t end = line.find("\"", start);

    if (end == std::string::npos) return 0.0;

    return atof(line.substr(start, end - start).c_str());
}

std::string getXmlStringValue(const std::string& line, const std::string& key)
{
    std::string pattern = key + "=\"";
    size_t start = line.find(pattern);

    if (start == std::string::npos) return "";

    start += pattern.length();
    size_t end = line.find("\"", start);

    if (end == std::string::npos) return "";

    return line.substr(start, end - start);
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

    EV_INFO << "Total red/yellow waiting time [s]: " << totalRedWaitingTime << endl;

    for (int i = 0; i < 4; i++) {
        EV_INFO << dirName(i)
                << " red/yellow waiting time [s]: "
                << redWaitingTime[i]
                << endl;
    }

    EV_INFO << "---------------- Queue lengths ----------------" << endl;

    for (int i = 0; i < 4; i++) {
        double avgQueue = 0.0;

        if (sampleCount > 0) {
            avgQueue = queueSum[i] / double(sampleCount);
        }

        EV_INFO << dirName(i)
                << " | passed=" << passed[i]
                << " | max queue=" << maxQueue[i]
                << " | avg queue=" << avgQueue
                << endl;
    }

    double avgCombinedQueue = 0.0;

    if (sampleCount > 0) {
        avgCombinedQueue = combinedQueueSum / double(sampleCount);
    }

    EV_INFO << "Combined max queue: " << maxCombinedQueue << endl;
    EV_INFO << "Combined avg queue: " << avgCombinedQueue << endl;

    EV_INFO << "---------------- CO2 emission and fuel consumption ----------------" << endl;

    std::ifstream file(EMISSIONS_FILE);

    double totalCO2_mg = 0.0;
    double totalFuel_ml = 0.0;
    double totalDistance_m = 0.0;

    std::set<std::string> emissionVehicles;

    if (!file.is_open()) {
        EV_INFO << "Could not open emissions.xml. Check this path:" << endl;
        EV_INFO << EMISSIONS_FILE << endl;
        EV_INFO << "Add this to simple.sumo.cfg:" << endl;
        EV_INFO << "<output><emission-output value=\"/home/veins/src/veins/examples/veins2/emissions.xml\"/></output>" << endl;
    }
    else {
        std::string line;

        while (std::getline(file, line)) {
            if (line.find("<vehicle") != std::string::npos) {
                std::string id = getXmlStringValue(line, "id");

                if (!id.empty()) {
                    emissionVehicles.insert(id);
                }

                double co2Rate_mg_s = getXmlDoubleValue(line, "CO2");
                double fuelRate_ml_s = getXmlDoubleValue(line, "fuel");
                double speed_m_s = getXmlDoubleValue(line, "speed");

                totalCO2_mg += co2Rate_mg_s * EMISSION_STEP_LENGTH;
                totalFuel_ml += fuelRate_ml_s * EMISSION_STEP_LENGTH;
                totalDistance_m += speed_m_s * EMISSION_STEP_LENGTH;
            }
        }

        int emissionVehicleCount = emissionVehicles.size();

        double totalCO2_g = totalCO2_mg / 1000.0;
        double totalCO2_kg = totalCO2_mg / 1000000.0;

        double totalFuel_l = totalFuel_ml / 1000.0;

        double totalDistance_km = totalDistance_m / 1000.0;

        EV_INFO << "Vehicles with emission data: " << emissionVehicleCount << endl;

        EV_INFO << "Total driven distance [km]: " << totalDistance_km << endl;

        EV_INFO << "Total CO2 emission [g]: " << totalCO2_g << endl;
        EV_INFO << "Total CO2 emission [kg]: " << totalCO2_kg << endl;

        EV_INFO << "Total fuel consumption [ml]: " << totalFuel_ml << endl;
        EV_INFO << "Total fuel consumption [l]: " << totalFuel_l << endl;

        if (emissionVehicleCount > 0) {
            EV_INFO << "Average CO2 emission per vehicle [g]: "
                    << totalCO2_g / emissionVehicleCount
                    << endl;

            EV_INFO << "Average fuel consumption per vehicle [ml]: "
                    << totalFuel_ml / emissionVehicleCount
                    << endl;

            EV_INFO << "Average fuel consumption per vehicle [l]: "
                    << totalFuel_l / emissionVehicleCount
                    << endl;
        }

        if (totalDistance_km > 0.0) {
            double co2_g_per_km = totalCO2_g / totalDistance_km;
            double fuel_l_per_100km = (totalFuel_l / totalDistance_km) * 100.0;

            EV_INFO << "Average CO2 emission [g/km]: "
                    << co2_g_per_km
                    << endl;

            EV_INFO << "Average fuel consumption [l/100km]: "
                    << fuel_l_per_100km
                    << endl;
        }
    }

    EV_INFO << "===================================================================" << endl;

    if (controlEvt) {
        cancelAndDelete(controlEvt);
        controlEvt = nullptr;
    }

    DemoBaseApplLayer::finish();
}
