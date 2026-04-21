#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

namespace adaptive {

class VehicleApp : public veins::DemoBaseApplLayer {
protected:
    cMessage* sendMsgEvt = nullptr;

protected:
    virtual void initialize(int stage) override;
    virtual void finish() override;
    virtual void handleSelfMsg(cMessage* msg) override;
    virtual void sendStatus();
};

}

