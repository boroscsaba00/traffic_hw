#pragma once

#include <map>
#include <string>

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

namespace veins {

class VEINS_API TraCIDemoRSU11p : public DemoBaseApplLayer {
protected:
    cMessage* controlEvt = nullptr;

    int currentPhaseDir = 0;
    int pendingPhaseDir = 0;
    bool yellowActive = false;

    simtime_t minGreen;
    simtime_t maxGreen;
    simtime_t lastSwitch;

    // statistics
    long passed[4] = {0, 0, 0, 0};
    double redWaitingTime[4] = {0, 0, 0, 0};
    int maxQueue[4] = {0, 0, 0, 0};
    double queueSum[4] = {0, 0, 0, 0};

    long totalPassed = 0;
    double totalRedWaitingTime = 0;
    int maxCombinedQueue = 0;
    double combinedQueueSum = 0;
    long sampleCount = 0;

    std::map<std::string, int> lastIncomingDir;

protected:
    void initialize(int stage) override;
    void finish() override;
    void handleSelfMsg(cMessage* msg) override;

    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
};

}
