#include "adaptive/VehicleApp.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"

using namespace veins;

namespace adaptive {

Define_Module(VehicleApp);

void VehicleApp::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sendMsgEvt = new cMessage("sendMsgEvt");
        scheduleAt(simTime() + uniform(0, 0.2), sendMsgEvt);
    }
}

void VehicleApp::handleSelfMsg(cMessage* msg) {
    if (msg == sendMsgEvt) {
        sendStatus();
        scheduleAt(simTime() + 1.0, sendMsgEvt);
        return;
    }
    DemoBaseApplLayer::handleSelfMsg(msg);
}

void VehicleApp::sendStatus() {
    DemoSafetyMessage* wsm = new DemoSafetyMessage();
    populateWSM(wsm);
    wsm->setSenderAddress(myId);
    sendDown(wsm);
}

void VehicleApp::finish() {
    cancelAndDelete(sendMsgEvt);
    sendMsgEvt = nullptr;
    DemoBaseApplLayer::finish();
}

}

