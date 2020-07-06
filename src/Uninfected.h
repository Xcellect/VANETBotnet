#ifndef TraCIDemo11p_H
#define TraCIDemo11p_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

/**
 * @brief
 * A tutorial demo for TraCI. When the car is stopped for longer than 10 seconds
 * it will send a message out to other cars containing the blocked road id.
 * Receiving cars will then trigger a reroute via TraCI.
 * When channel switching between SCH and CCH is enabled on the MAC, the message is
 * instead send out on a service channel following a WAVE Service Advertisement
 * on the CCH.
 *
 * @author Christoph Sommer : initial DemoApp
 * @author David Eckhoff : rewriting, moving functionality to BaseWaveApplLayer, adding WSA
 *
 */

class Uninfected : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
    protected:
        simtime_t lastDroveAt;
        bool sentMessage;
        int currentSubscribedServiceId;
    protected:
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);

        virtual void handleSelfMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);
};

#endif
