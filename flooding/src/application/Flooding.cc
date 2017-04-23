/*
 MIT License

 Copyright (c) 2017 Valentine Nwachukwu <valdiz777@gmail.com>

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#include "Flooding.h"
using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t Flooding::parkingStateChangedSignal = simsignalwrap_t(
        TRACI_SIGNAL_PARKING_CHANGE_NAME);
bool contains(std::deque<int> * queue, int key);

Define_Module(Flooding);

void Flooding::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        //sentMessage = false;
        lastDroveAt = simTime();
        findHost()->subscribe(parkingStateChangedSignal, this);
        isParking = false;
        sendWhileParking = par("sendWhileParking").boolValue();
    }
}

void Flooding::onBeacon(WaveShortMessage* wsm) {
}

void Flooding::onData(WaveShortMessage* wsm) {
    // ignore messages that I sent out
    if (contains(&sentMessages, wsm->getSerial())) {
        return;
    }
    findHost()->getDisplayString().updateWith("r=16,green");

    if (mobility->getRoadId()[0] != ':')
        traciVehicle->changeRoute(wsm->getWsmData(), 9999);
    //if (!sentMessage) sendMessage(wsm->getWsmData());

    sendMessage(wsm->getWsmData(), -1, wsm->getSerial());
    sentMessages.push_back(wsm->getSerial());
}

void Flooding::sendMessage(std::string blockedRoadId, int recipient,
        int serial) {

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel,
            dataPriority, -1, serial);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}
void Flooding::receiveSignal(cComponent* source, simsignal_t signalID,
        cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}
void Flooding::handleParkingUpdate(cObject* obj) {
    isParking = mobility->getParkingState();
    if (sendWhileParking == false) {
        if (isParking == true) {
            (FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(
                    this->getParentModule()->getSubmodule("nic"));
        } else {
            Coord pos = mobility->getCurrentPosition();
            (FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(
                    this->getParentModule()->getSubmodule("nic"),
                    (ChannelAccess*) this->getParentModule()->getSubmodule(
                            "nic")->getSubmodule("phy80211p"), &pos);
        }
    }
}

// route messages manually, bypass wsm routing to on data automatically
void Flooding::handleLowerMsg(cMessage* msg) {

    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
    EV << "****handleLowerMsg" << endl;
    if (std::string(wsm->getName()) == "data") {
        ASSERT(wsm);
        onData(wsm);
    } else {
        DBG << "unknown message (" << wsm->getName() << ")  received\n";
    }
    delete (msg);
}

void Flooding::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10) {
            findHost()->getDisplayString().updateWith("r=16,red");
            if (!sentMessage) {
                int serial = rand() % 101;
                sendMessage(mobility->getRoadId(), -1, serial);
                sentMessage = true;
            }
        }
    } else {
        lastDroveAt = simTime();
    }
}
void Flooding::sendWSM(WaveShortMessage* wsm) {
    if (isParking && !sendWhileParking)
        return;
    sendDelayedDown(wsm, individualOffset);
}

bool contains(std::deque<int> * queue, int key) {
    return (find(queue->begin(), queue->end(), key) != queue->end()) ?
            true : false;
}
