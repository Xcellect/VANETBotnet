#include "Uninfected.h"
#include <string>
Define_Module(Uninfected);

void Uninfected::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        calculated = false;

        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
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

// wsm response, exploitation happens here
void Uninfected::onWSM(WaveShortMessage* wsm) {
    findHost()->getDisplayString().updateWith("r=16,blue");
    /**************************/
    // accessing the array from a different car
    std::string query;
    std::list<std::string> list = Uninfected::stringToList(wsm->getWsmData());
    for(int i = 0; i < list.size(); i++)
        i != list.size()-1 ? query += Uninfected::getEdgeByIndex(list, i) +"," : query += Uninfected::getEdgeByIndex(list, i);

    /*************************
    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);
    if (!sentMessage) {
        sentMessage = true;
        // repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
    }
    */
}

void Uninfected::onBSM(BasicSafetyMessage* bsm) {
    findHost()->getDisplayString().updateWith("r=16,purple");
    struct Uninfected::CongestionInfo con = stringToStruct(bsm->getWsmData());
    // updating current car's cinfo
    updateSelf();
    updateDB(cInfo);
    // Need edge information for this comparison
    if(cidb.size() >= 3) {
        printf("smd");
    }
    if(con.edgeID.compare(cInfo.edgeID) == 0) {
    // if it already exists just update it
        updateDB(con);
    }
}


void Uninfected::handleSelfMsg(cMessage* msg) {
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
            scheduleAt(simTime()+2, wsm);
        }
    }
    else {
      //  BaseWaveApplLayer::handleSelfMsg(msg);
    }

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
    updateSelf();
    updateDB(cInfo);
    // Road length is 200. Make the threshold 180.
    // apparently cidb never increments here???
    if (traciVehicle->getLanePosition() > 180 && calculated == false) {
        findHost()->getDisplayString().updateWith("r=16,yellow");
        // calculate avg
        double avgSpeed = 0.0;
        std::map<int,CongestionInfo>::iterator it;
        for(it=cidb.begin(); it != cidb.end(); ++it)
            avgSpeed += it->second.avgSpeed;
        avgSpeed = avgSpeed/cidb.size();
        // update own struct avgSpeed bc it got measurements from other cars
        cInfo.avgSpeed = avgSpeed;
        calculated = true;
        // send wsm request, find a way to get the next edge IDs
        // creating the query string of edges in current vehicle
        std::string allLanes;
        std::list<std::string> lanes = traciVehicle->getPlannedRoadIds();
        int size = lanes.size();
        for(int i = 0; i < size; i++)
            i != size-1 ? allLanes += getEdgeByIndex(lanes, i) +"," : allLanes += getEdgeByIndex(lanes, i);
        sendCongestionRequest(allLanes);
    } else {
        findHost()->getDisplayString().updateWith("r=16,green");
        lastDroveAt = simTime();
    }
    sendBeacon(cInfo);
}
// wsm request
void Uninfected::sendCongestionRequest(std::string edgeIDs) {
    findHost()->getDisplayString().updateWith("r=16,red");
    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm);
    wsm->setWsmData(edgeIDs.c_str());
    sendDown(wsm);
}

// calculate minimum route time and choose that route
// rerouting happens when a different edge has higher avg speed
void Uninfected::reroute() {

}
// string to list
std::list<std::string> Uninfected::stringToList(std::string edges) {
    std::list<std::string> ls;
    std::string const delims{","};
    size_t beg, pos = 0;
    while ((beg = edges.find_first_not_of(delims, pos)) != std::string::npos) {
        pos = edges.find_first_of(delims, beg + 1);
        ls.push_back(edges.substr(beg, pos - beg).c_str());
    }
    return ls;
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


void Uninfected::updateDB(struct CongestionInfo cInfo) {
    std::pair<std::map<int, CongestionInfo>::iterator,bool> itr;
    // if it already exists just update it
    itr = cidb.insert(std::pair<int,CongestionInfo>(cInfo.creatorID,cInfo));
    if(itr.second == false) {
        cidb.erase(cInfo.creatorID);
        cidb.insert(std::pair<int,CongestionInfo>(cInfo.creatorID,cInfo));
    }
}

void Uninfected::updateSelf() {
    cInfo.edgeID = traciVehicle->getLaneId();
    cInfo.timestamp = simTime();
    cInfo.avgSpeed = mobility->getSpeed();
}

void Uninfected::sendBeacon(struct CongestionInfo cInfo) {
    BasicSafetyMessage* bsm = new BasicSafetyMessage();
    populateWSM(bsm);
    // struct Uninfected::CongestionInfo con = stringToStruct(cInfo.toString().c_str());
    bsm->setWsmData(cInfo.toString().c_str());
    sendDown(bsm);
    scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
}

std::string Uninfected::getEdgeByIndex(std::list<std::string> lanes, int index) {
    // TraCICommandInterface::Route rou = traci->route(traciVehicle->getRouteId());

    std::list<std::string>::iterator it;
    int count = 0;
    for (it = lanes.begin(); it != lanes.end(); ++it) {
        if(count == index) return *it;
        count++;
    }
    return 0;
}


