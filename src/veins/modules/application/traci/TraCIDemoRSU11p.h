//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//

#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

namespace veins {

/**
 * RSU app with adaptive traffic light control
 */
class VEINS_API TraCIDemoRSU11p : public DemoBaseApplLayer {
protected:
    cMessage* controlEvt = nullptr;

    int currentPhaseDir = 0;   // 0=N, 1=S, 2=E, 3=W
    simtime_t minGreen;
    simtime_t maxGreen;
    simtime_t lastSwitch;

protected:
    void initialize(int stage) override;
    void finish() override;
    void handleSelfMsg(cMessage* msg) override;

    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
};

} // namespace veins

