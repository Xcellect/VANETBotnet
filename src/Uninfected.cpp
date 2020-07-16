#include "Uninfected.h"
#include <string>
Define_Module(Uninfected);

void Uninfected::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

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
    // reacting to request
    if(wsm->getPsid() == 10) {
        // accessing the array from a different car
        std::string query;
        std::list<std::string> list = Uninfected::stringToList(wsm->getWsmData());
        // iterate through the list and check cidb (map) has edge info on this
        // check if this car has calculated the average speed of that edge
        // Uninfected::getEdgeByIndex(list, i)
        // if it does, send that edge's struct
        std::list<struct CongestionInfo> cidbResponse;
        const char* c1;
        const char* c2 = wsm->getWsmData();
        const char* c3;
        for(int i = 0; i < list.size(); i++)
            for(std::map<std::string,struct CongestionInfo>::iterator it=cidb.begin(); it!=cidb.end(); ++it) {
                c3 = it->second.edgeID.c_str();
                // ITERATE MAP LIKE A NORMAL PERSON
                if(Uninfected::getEdgeByIndex(list, i).compare(it->second.edgeID) == 0)
                    cidbResponse.push_back( it->second );
            }
        if(cidbResponse.size() != 0) sendCongestionResponse(cidbResponse);
    } else {    // reacting to response
        // great. now we're getting the response. time to get to decode and decide.
        // now we have info on only maybe one or two roads, and not all
        // parse it to list of structs and add each of them to your cidb

        std::list<struct Uninfected::CongestionInfo> cidbList = stringToListStruct(wsm->getWsmData());
        std::list<struct Uninfected::CongestionInfo>::iterator it;
        cInfo.avgSpeed = calculateCIDBAvg(cidb);    // got the current road avg here
        for (it = cidbList.begin(); it != cidbList.end(); ++it) {
            updateDB(*it);      // adding congestionResponse struct to current DB
        }
        /*  Afterwards, the routing
            decision becomes a set of arithmetic operations; the trip time
            for each of the k candidate routes will be computed, and the
            car will choose the one with the lowest trip time.
            Problem: it's assuming we have measurements for all the edges

            Make a list of routes (list of roads) and sort them according to shortest time
         */
        const char* c9;
        const char* c10;
        std::string allEdges, currRoute;
        for(std::map<std::string,struct CongestionInfo>::iterator it=cidb.begin(); it!=cidb.end(); ++it) {
            allEdges += it->second.edgeID + ", ";
        }
        c9 = allEdges.c_str();
        currRoute = listToString(traciVehicle->getPlannedRoadIds());
        c10 = currRoute.c_str();

        // sort the candidate routes based on shortest time
        // then reroute using the shortest route
        const char* currEdge = listToString(traci->getRouteIds()).c_str();
        // here we need to choose between roads. if there is no avg speed info, use 50% max speed
        // for missing roads
        reroute();
    }

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
    // add only if the car is on the same edge
    if(con.edgeID.compare(cInfo.edgeID) == 0) {
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
    if (traciVehicle->getLanePosition() > 180) {
        findHost()->getDisplayString().updateWith("r=16,yellow");
        // send BSM request, find a way to get the next edge IDs
        // creating the query string of edges in current vehicle
        // list to string
        sendCongestionRequest(listToString(traciVehicle->getPlannedRoadIds()));
        // then we move on to onWSM's 2nd part where we get congestionResponse
        reroute();
    }
    sendBeacon(cInfo);
    // wait till you get the congestion response. if you don't, choose whatever
    // has higher avg speed
}
double Uninfected::calculateCIDBAvg(std::map<std::string, struct CongestionInfo> cidb) {

    double avgSpeed = 0.0;
    std::map<std::string, struct CongestionInfo>::iterator it = cidb.begin();
    int unique = 1;
    for(; it != cidb.end(); ++it) {
        if(it->second.edgeID.compare(cInfo.edgeID) == 0) {
            avgSpeed += it->second.avgSpeed;
            unique++;
        }
    }
    return (avgSpeed + cInfo.avgSpeed)/unique;
}


// wsm request
void Uninfected::sendCongestionRequest(std::string edgeIDs) {
    findHost()->getDisplayString().updateWith("r=16,red");
    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm);
    // setting psid to 10 for request
    wsm->setPsid(10);
    wsm->setWsmData(edgeIDs.c_str());
    sendDown(wsm);
}
void Uninfected::sendCongestionResponse(std::list<struct CongestionInfo> cidbResponse) {
    // sort the db with relevant edges by timestamp
    // here we're just sending the cidbResponse as wsm
    // then based on these structs it will choose the route
    findHost()->getDisplayString().updateWith("r=16,orange");
    std::string str = listStructToString(cidbResponse);
    WaveShortMessage* wsm = new WaveShortMessage();
    populateWSM(wsm);
    // setting psid to 20 for response
    wsm->setPsid(20);
    wsm->setWsmData(str.c_str());
    sendDown(wsm);
}

std::string Uninfected::listStructToString(std::list<struct CongestionInfo> cidbResponse) {
    std::string str;
    for(int i = 0; i != cidbResponse.size(); ) {
        str += "{" + cidbResponse.front().toString() + "}";
        cidbResponse.pop_front();
    }
    return str;
}
std::list<struct Uninfected::CongestionInfo> Uninfected::stringToListStruct(std::string cidbStr) {
    // ok it works for multiple structs in a single string separated by braces
    std::list<struct CongestionInfo> listStruct;
    std::string const delims{"{}"};
    // just get the content inside the braces and send to the struct converter
    std::string whatever;
    size_t beg, pos = 0;
    const char* c;
    struct CongestionInfo con;
    while ((beg = cidbStr.find_first_not_of(delims, pos)) != std::string::npos) {
        pos = cidbStr.find_first_of(delims, beg + 1);
        whatever = cidbStr.substr(beg, pos - beg);
        c = whatever.c_str();
        con = stringToStruct(whatever);
        listStruct.push_back(con);
    }
    return listStruct;
}




// calculate minimum route time and choose that route
// rerouting happens when a different edge has higher avg speed
void Uninfected::reroute() {
    std::string currEdge = getEdgeByIndex(traciVehicle->getPlannedRoadIds(), traciVehicle->getLaneIndex());
    const char* curr = currEdge.c_str();
    std::string all = listToString(traciVehicle->getPlannedRoadIds());
    const char* a = all.c_str();
    //std::string laneIDs = listToString(traci->getLaneIds());


    std::list<std::string> edgeList = traci->getLaneIds();
    std::string adjacentEdges;
    int size = edgeList.size();
    std::string end = currEdge.substr(2, 4);

    const char* en = end.c_str();
    for(int i = 0; i < size; i++)
        if(getEdgeByIndex(edgeList, i).substr(0, 2).compare(currEdge.substr(2, 4)) == 0)
            adjacentEdges += getEdgeByIndex(edgeList, i) + ",";
    // cool now we got the adjacent edges
    const char* e = adjacentEdges.c_str();
    // now if we choose an edge based on higher avg speed

    printf(curr);


}
// list to string
std::string Uninfected::listToString(std::list<std::string> lanes) {
    std::string allLanes;
    int size = lanes.size();
    for(int i = 0; i < size; i++)
        i != size-1 ? allLanes += getEdgeByIndex(lanes, i) +"," : allLanes += getEdgeByIndex(lanes, i);
    return allLanes;
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


void Uninfected::updateDB(struct CongestionInfo con) {
    std::pair<std::map<std::string, struct CongestionInfo>::iterator,bool> itr;
    // if it already exists just update it
    itr = cidb.insert(std::pair<std::string, struct CongestionInfo>(con.edgeID,con));
    if(itr.second == false && con.creatorID != cInfo.creatorID) {
        cidb.erase(cInfo.edgeID);
        cidb.insert(std::pair<std::string, struct CongestionInfo>(con.edgeID,con));
    }
}

void Uninfected::updateSelf() {
    cInfo.edgeID = traciVehicle->getRoadId();
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

    std::list<std::string>::iterator it;
    int count = 0;
    for (it = lanes.begin(); it != lanes.end(); ++it) {
        if(count == index) return *it;
        count++;
    }
    return 0;
}


