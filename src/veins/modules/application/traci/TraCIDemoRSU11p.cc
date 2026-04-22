//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//

#include "veins/modules/application/traci/TraCIDemoRSU11p.h"

#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"

using namespace veins;

Define_Module(veins::TraCIDemoRSU11p);

namespace {
const char* TLS_ID = "J";

int dirToPhase(int dir)
{
    // dir meaning in code:
    // 0 = north
    // 1 = south
    // 2 = east
    // 3 = west

    if (dir == 0) return 0; // north green
    if (dir == 1) return 4; // south green
    if (dir == 2) return 2; // east green
    return 6;               // west green
}



int countVehiclesOnEdge(TraCIScenarioManager* manager, const std::string& edgeId)
{
    int count = 0;

    const std::map<std::string, cModule*>& hosts = manager->getManagedHosts();
    for (std::map<std::string, cModule*>::const_iterator it = hosts.begin(); it != hosts.end(); ++it) {
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

        try {
            if (mob->getRoadId() == edgeId) {
                count++;
            }
        }
        catch (...) {
            // mobility not initialized enough yet, ignore this host for now
        }
    }

    return count;
}
}

void TraCIDemoRSU11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {
        currentPhaseDir = 0;
        minGreen = 8;
        maxGreen = 25;
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

        int n = countVehiclesOnEdge(manager, "N_in");
        int s = countVehiclesOnEdge(manager, "S_in");
        int e = countVehiclesOnEdge(manager, "E_in");
        int w = countVehiclesOnEdge(manager, "W_in");

        int bestDir = 0;
        int bestDemand = n;

        if (s > bestDemand) {
            bestDemand = s;
            bestDir = 1;
        }
        if (e > bestDemand) {
            bestDemand = e;
            bestDir = 2;
        }
        if (w > bestDemand) {
            bestDemand = w;
            bestDir = 3;
        }

        simtime_t greenElapsed = simTime() - lastSwitch;

        EV_INFO << "Demand N=" << n
                << " S=" << s
                << " E=" << e
                << " W=" << w
                << " current=" << currentPhaseDir
                << " best=" << bestDir
                << endl;

        if (greenElapsed >= minGreen) {
            if (bestDir != currentPhaseDir || greenElapsed >= maxGreen) {
                int targetPhase = dirToPhase(bestDir);
                traci->trafficlight(TLS_ID).setPhaseIndex(targetPhase);
                currentPhaseDir = bestDir;
                lastSwitch = simTime();

                EV_INFO << "Switched TLS to phase " << targetPhase << endl;
            }
        }

        scheduleAt(simTime() + 1, controlEvt);
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg);
}

void TraCIDemoRSU11p::finish()
{
    if (controlEvt) {
        cancelAndDelete(controlEvt);
        controlEvt = nullptr;
    }
    DemoBaseApplLayer::finish();
}

