#include "Uninfected.h"
#include <string>
Define_Module(Uninfected);

void Uninfected::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
    carCount = 1;

    cInfo.creatorID = mobility->getNode()->getIndex();

    // read parameters

}

void Uninfected::onWSA(WaveServiceAdvertisment* wsa) {
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
        currentSubscribedServiceId = wsa->getPsid();
        if  (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService((Channels::ChannelNumber) wsa->getTargetChannel(), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void Uninfected::onWSM(WaveShortMessage* wsm) {
    findHost()->getDisplayString().updateWith("r=16,green");

    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
    if (!sentMessage) {
        sentMessage = true;
        //repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
    }
}

void Uninfected::onBSM(BasicSafetyMessage* bsm) {
    findHost()->getDisplayString().updateWith("r=16,purple");
    Coord cSpeed = bsm->getSenderSpeed();
    struct Uninfected::CongestionInfo con = stringToStruct(bsm->getWsmData());

    cInfo.edgeID = mobility->getRoadId();
    cInfo.timestamp = simTime();
    cInfo.avgSpeed = mobility->getSpeed();


    double speed = 0;
    double xfract, xint, yfract, yint;
    double x = abs(cSpeed.x);
    double y = abs(cSpeed.y);
    xfract = modf(x, &xint);
    yfract = modf(y, &yint);
    // Need edge information for this comparison
    if(con.edgeID.compare(cInfo.edgeID) == 0) {
        if(xint > 0) {
            speed = x;
        } else if(yint > 0) {
            speed = y;
        }
        carCount++;
        if(carCount == 3) {
            printf("we just cured cancer");
        }
    }
    cInfo.avgSpeed += speed;
    cInfo.avgSpeed = cInfo.avgSpeed/carCount;
}


void Uninfected::handleSelfMsg(cMessage* msg) {
    /*
    if (WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg)) {
        //send this message on the service channel until the counter is 3 or higher.
        //this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() +1);
        if (wsm->getSerial() >= 3) {
            //stop service advertisements
            stopService();
            delete(wsm);
        }
        else {
            scheduleAt(simTime()+1, wsm);
        }
    }
    else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
    */
}
/*
 B. Congestion Information Database

1) Creating and Storing Congestion Information: In order
to store and exchange congestion measurements, vehicles make
use of congestion info structs. Each struct consists of the
following fields:

a. Creator ID: The unique ID of the vehicle that created this
measurement.
b. Edge ID: The unique ID of the one-way road that this
measurement belongs to.
c. Average Speed: Average of the speed readings from all the
cars on the same road with the car that crates this measurement.
d. Timestamp: Time of the measurement�s creation to ensure
the freshness of measurements and prioritize most recent ones.

2) Maintainting CIDB: Let, m be this struct (entry), E be the set
of all edges. DB = {m1,m2,m3,...,mn}, 1 <= n <= |E|. 2 cases for
an entry:
a. after creating its own measurement, sampling own and
other cars' speeds on its road. after t time threshold, its own
measurement gets outdated in the DB. a car will prioritize its
own entry over fresher ones unless it is at least t older. makes
it kinda secure.
b. after obtaining measurements thru congestion responses it
decides what route to choose

3) Least congested route selection:
    a. reach the end of the road
    b. calculate its interesting roads
    c. send congestion request asking for these roads
    d. merge both in CIDB
    e. routing decision:
        i. compute trip time for each of k candidate routes
            a. edge weight: calculate edge divided by avg speed
            b. route weight: calculate sum of all edge weights
                in the route
            c. find the minimum route weight out of all candidates
        ii. choose one of the lowest
 */
void Uninfected::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    cInfo.edgeID = mobility->getRoadId();
    cInfo.timestamp = simTime();
    cInfo.avgSpeed = mobility->getSpeed();
    // Road length is 200. Make the threshold 180.

    if (traciVehicle->getLanePosition() > 180) {
        findHost()->getDisplayString().updateWith("r=16,red");
        // iterate through all vehicles in the same edge
        // need to get this info from BSM

    } else {
        findHost()->getDisplayString().updateWith("r=16,green");
        lastDroveAt = simTime();
    }
    // send bsm
    if(simTime() - lastDroveAt >= 1 && sentMessage == false) {
        findHost()->getDisplayString().updateWith("r=16,yellow");
        sentMessage = true;

        BasicSafetyMessage* bsm = new BasicSafetyMessage();
        populateWSM(bsm);
        struct Uninfected::CongestionInfo con = stringToStruct(cInfo.toString().c_str());
        bsm->setWsmData(con.toString().c_str());
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);

    }
    // cInfo.avgSpeed = 0;
    // carCount = 1;
}
// Converting string to struct
struct Uninfected::CongestionInfo Uninfected::stringToStruct(std::string str) {
    struct Uninfected::CongestionInfo con;
    std::string const delims{",="};
    size_t beg, pos = 0;
    int count = 0;
    while ((beg = str.find_first_not_of(delims, pos)) != std::string::npos) {
        count++;
        pos = str.find_first_of(delims, beg + 1);
        if(count%2==0) {
            if(count == 2) con.creatorID = std::stoi(str.substr(beg, pos - beg));
            if(count == 4) con.edgeID = str.substr(beg, pos - beg).c_str();
            if(count == 6) con.avgSpeed = std::stod(str.substr(beg, pos - beg));
            if(count == 8) con.timestamp =  std::stoi(str.substr(beg, pos - beg));
         }
    }
    return con;
}


