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

#ifndef Flooding_H
#define Flooding_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

/**
 * Simple Flooding VANET simulation using 11p
 */
class Flooding: public BaseWaveApplLayer {
public:
    virtual void initialize(int stage);
    virtual void receiveSignal(cComponent* source, simsignal_t signalID,
            cObject* obj, cObject* details);
protected:
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;
    AnnotationManager* annotations;
    simtime_t lastDroveAt;
    bool sentMessage;
    bool isParking;
    bool sendWhileParking;
    std::deque<int> sentMessages;
    static const simsignalwrap_t parkingStateChangedSignal;
protected:
    virtual void onBeacon(WaveShortMessage* wsm);
    virtual void onData(WaveShortMessage* wsm);
    void sendMessage(std::string blockedRoadId, int recipient,
            int serial);
    virtual void handlePositionUpdate(cObject* obj);
    virtual void handleParkingUpdate(cObject* obj);

    virtual void handleLowerMsg(cMessage* msg);
    virtual void sendWSM(WaveShortMessage* wsm);
};
template<class InputIterator, class T>
static InputIterator find(InputIterator first, InputIterator last,
        const T& val) {
    while (first != last) {
        if (*first == val)
            return first;
        ++first;
    }
    return last;
}
#endif
