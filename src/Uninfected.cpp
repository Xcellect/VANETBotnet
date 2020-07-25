#include "Uninfected.h"
#include <string>
Define_Module(Uninfected);

void Uninfected::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        chosenRoute = traciVehicle->getPlannedRoadIds();
    }
    changedRoute = false;
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
/* Bots still request
 * congestion information from other cars to choose the least
 * congested route for themselves; however, when they receive
 * a congestion request, they respond with bogus information as
 * the parked bot cars do.
 * */
void Uninfected::onWSM(WaveShortMessage* wsm) {
    findHost()->getDisplayString().updateWith("r=16,blue");
    /**************************/

    // reacting to request
    if(wsm->getPsid() == 10) {
        // accessing the array from a different car
        std::string query;
        std::list<std::string> roadList = roadListStringToRoadList(wsm->getWsmData());
        std::list<struct CongestionInfo> cidbResponse;


        for(int i = 0; i < roadList.size(); i++) {
                    // convert string roads to list of roads
            std::string edge = getElementByIndex(roadList, i);
            for(std::map<std::string,struct CongestionInfo>::iterator it=cidb.begin(); it!=cidb.end(); ++it) {
                std::string cidbEdge = it->second.edgeID;
                // ITERATE MAP LIKE A NORMAL PERSON
                if(edge.compare(cidbEdge) == 0 && !contains_struct(cidbResponse, edge))
                    cidbResponse.push_back(it->second);
                }
            }
        if(cidbResponse.size() != 0)
            sendCongestionResponse(cidbResponse);
    } else {    // reacting to response
        // great. now we're getting the response. time to get to decode and decide.
        // now we have info on only maybe one or two roads, and not all
        // parse it to list of structs and add each of them to your cidb

        std::list<struct Uninfected::CongestionInfo> conResponseCIDB = stringToListStruct(wsm->getWsmData());
        std::list<struct Uninfected::CongestionInfo>::iterator it;
        // apparently the speed averaging can be influenced by the fork road averages
        cInfo.avgSpeed = calculateCIDBAvg(cidb);    // got the current road avg here
        for (it = conResponseCIDB.begin(); it != conResponseCIDB.end(); ++it) {
            updateDB(*it);      // adding congestionResponse struct to current DB
        }
        /*  Afterwards, the routing
            decision becomes a set of arithmetic operations; the trip time
            for each of the k candidate routes will be computed, and the
            car will choose the one with the lowest trip time.
            Problem: it's assuming we have measurements for all the edges
            Sol: give unknown roads a default trip time
            Make a list of routes (list of roads) and sort them according to shortest time
         */
        if(traciVehicle->getRoadId().compare(prevRoad) == 0) {
            changedRoute = true;
        } else {
            changedRoute = false;
        }
        if(!changedRoute) reroute();
        prevRoad = traciVehicle->getRoadId();
    }
    /*
    if (!sentMessage) {
        sentMessage = true;
        // repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01,0.2), wsm->dup());
    }
     */
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
       // BaseWaveApplLayer::handleSelfMsg(msg);
    }
*/
}
bool Uninfected::contains_struct(std::list<struct CongestionInfo> cidbResponse, std::string query) {
    for (std::list<struct CongestionInfo>::iterator it=cidbResponse.begin(); it != cidbResponse.end(); ++it) {
        struct CongestionInfo con = *it;
        if(con.edgeID.compare(query) == 0) {
            return true;
        }
    }
    return false;
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



/*Exploiting Basic Safety Messages: Cars broadcast their
current speeds at a constant frequency as a part of the
mandatory “Basic Safety Messages” (BSM) standardized in
[8]. These speed exchanges are used by each car in the
congestion avoidance mechanism to calculate the total average
speeds on the roads that they travel on. These average speed
measurements are then used for travel time calculations and
least-congested route selection. The goal of our attack is
to redirect as many cars as possible to the targeted road,
eventually decreasing its average speed. A vehicle will not pick
a route that contains the targeted road if it hears overwhelming
reports of low speeds from the cars that actually went over
that road. As a result of the congestion avoidance mechanism,
the other victim cars that are considering using the targeted
road will start rerouting themselves, which will prevent the
congestion from becoming heavier.
To overcome this effect, bots will advertise high speeds
while they are traveling on the targeted road. In other words,
they will continue to beacon that they are moving fast on
the targeted road that is, in fact, heavily congested. This will
exploit speed averaging mechanisms of the cars on the targeted
road, causing their calculated averages to increase in order to
help the effect of our attack continue as long as possible.
 * */
void Uninfected::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    updateSelf();
    updateDB(cInfo);
    // Road length is 200. Make the threshold 180.
    // apparently cidb never increments here???
    if (traciVehicle->getLanePosition() > 160) {
        findHost()->getDisplayString().updateWith("r=16,yellow");
        std::string alledges = routeListToRoadListString(getRouteListByCarID(traci->getRouteIds(), cInfo.creatorID));
        sendCongestionRequest(alledges);

        // then we move on to onWSM's 2nd part where we get congestionResponse
        if(traciVehicle->getRoadId().compare(prevRoad) == 0) {
            changedRoute = true;
        } else {
            changedRoute = false;
        }
        if(!changedRoute) reroute();
        prevRoad = traciVehicle->getRoadId();
        // rerouting works here but not afer congestion request.
        // traciVehicle->changeRoute(getElementByIndex(getAdjacentEdges(), getAdjacentEdges().size()-1), 9999);

    }
    sendBeacon(cInfo);
    // wait till you get the congestion response. if you don't, choose whatever
    // has higher avg speed
}

std::list<std::string> Uninfected::roadListStringToRoadList(std::string routeStr) {
    // ok it works for multiple structs in a single string separated by braces
    std::list<std::string> route;
    std::string const delims{"{}"};
    // just get the content inside the braces and send to the struct converter
    std::string edgelist;
    size_t beg, pos = 0;
    const char* c = routeStr.c_str();
    while ((beg = routeStr.find_first_not_of(delims, pos)) != std::string::npos) {
        pos = routeStr.find_first_of(delims, beg + 1);
        edgelist = routeStr.substr(beg, pos - beg);
        c = edgelist.c_str();
        std::list<std::string> newList = stringToList(edgelist);
        for(std::list<std::string>::iterator it = newList.begin(); it != newList.end(); ++it) {
            route.push_back(*it);
        }
    }
    route.sort();
    route.unique();
    return route;
}

std::string Uninfected::routeListToRoadListString(std::list<std::string> routeList) {
    std::string allRoads;
    for(int i = 0; i < routeList.size(); i++)
        allRoads += "{" + listToString( traci->route(getElementByIndex(routeList, i)).getRoadIds() )+ "}";
    return allRoads;
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
    const char* c = edgeIDs.c_str();
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

std::list<std::string> Uninfected::getAdjacentEdges() {
    //std::string laneIDs = listToString(traci->getLaneIds());
    std::list<std::string> adjEdge;
    std::list<std::string> edgeList = traci->getLaneIds();
    int size = edgeList.size();
    std::string edgeByIndex;
    for(int i = 0; i < size; i++) {
        edgeByIndex = getElementByIndex(edgeList, i);
        if(edgeByIndex.substr(0, 2).compare(cInfo.edgeID.substr(2, 4)) == 0
                /* && edgeByIndex.substr(2, 2).compare(currEdge.substr(0, 2)) != 0*/ ) {
            adjEdge.push_back(edgeByIndex.substr(0, 4));
        }
    }
    return adjEdge;
}



// calculate minimum route time and choose that route
// rerouting happens when a different edge has higher avg speed
void Uninfected::reroute() {
    // std::string rList = listToString(traci->route(getElementByIndex(getRouteListByCarID(traci->getRouteIds(), 114), 2)).getRoadIds());
    // relative to current road
    // 1: iterate the routes
    // 2: iterate the roads
    // choose the one withs shortest time

    std::list<std::string> routeList = getRouteListByCarID(traci->getRouteIds(), cInfo.creatorID);
    double time;
    double minTime = 1000;
    std::list<double> speedlist;
    std::list<double> timeL;
    //std::string fastestRoute;
    for(int i = 0; i < routeList.size(); i++) {
        double edgeCount = 0;
        double avgSpeedSum = 0;
        std::list<std::string> roads = traci->route(getElementByIndex(routeList, i)).getRoadIds();
        for(int j = 0; j < roads.size(); j++) {
            std::string edgeByIndex = getElementByIndex(roads, j);
            for(std::map<std::string,struct CongestionInfo>::iterator it=cidb.begin(); it!=cidb.end(); ++it) {
                if(it->second.edgeID.compare(edgeByIndex) == 0) {
                    avgSpeedSum += it->second.avgSpeed;
                } else {
                    avgSpeedSum += traci->road(edgeByIndex).getMeanSpeed() * 0.000001;
                }
            }
            edgeCount++;
        }
        time = (edgeCount*200)/avgSpeedSum;
        if(time < minTime) {
            minTime = time;
            chosenRoute = roads;
        }
        speedlist.push_back(time);
    }
    // you're supposed to change road one at a time, not the entire route
    // current gap: rerouting.
    // 1. rank the candidate routes.
    // 2. whatever is faster gets chosen.
    // 3. check the chosen route's edges in the current edge's adj edges
    // 4. if exists follow that road and subsequent ones
    std::list<std::string> adjacentEdges = getAdjacentEdges();
    if(!chosenRoute.empty() && !adjacentEdges.empty()) {
        for(int i = 0; i < adjacentEdges.size(); i++) {
            for(int j = 0; j < chosenRoute.size(); j++) {
                if(getElementByIndex(adjacentEdges, i).compare(getElementByIndex(chosenRoute, j)) == 0
                        && mobility->getRoadId()[0] != ':') {
                    traciVehicle->newRoute(getElementByIndex(adjacentEdges, i));
                    return;
                }
            }
        }
    }


}
// list to string
std::string Uninfected::listToString(std::list<std::string> lanes) {
    std::string allLanes;
    int size = lanes.size();
    const char* c;
    for(int i = 0; i < size; i++)
        (i != (size-1)) ? allLanes += getElementByIndex(lanes, i) +"," : allLanes += getElementByIndex(lanes, i);
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

std::string Uninfected::getElementByIndex(std::list<std::string> lanes, int index) {
    std::list<std::string>::iterator it;
    int count = 0;
    for (it = lanes.begin(); it != lanes.end(); ++it) {
        if(count == index) return *it;
        count++;
    }
    return 0;
}


std::list<std::string> Uninfected::getRouteListByCarID(std::list<std::string> ls, int carID) {
    std::list<std::string>::iterator it;
    std::pair<int,int> product;
    std::list<std::string> routeList;
    std::string routeID;
    for (it = ls.begin(); it != ls.end(); ++it) {
        product = iterateRouteID(*it);
        if(product.first == carID && product.second != -1) {
            routeList.push_back("!"+ std::to_string(product.first) + "#" +std::to_string(product.second));
        }
    }
    routeList.sort();
    routeList.unique();
    return routeList;
}


std::pair<int,int> Uninfected::iterateRouteID(std::string routeID) {
    const char* c0= routeID.c_str();
    std::pair<int,int> product;
    std::string const delims{"!#"};
    size_t beg, pos = 0;
    int count = 0;
    if(routeID.size() > 3) {
        while ((beg = routeID.find_first_not_of(delims, pos)) != std::string::npos) {
            count++;
            pos = routeID.find_first_of(delims, beg + 1);
            std::string curr = routeID.substr(beg, pos - beg);
            bool valid = true;
            for(int i = 0; i < curr.size(); i++) {
                if(!isdigit(curr[0])){
                    valid = false;
                    break;
                }
            }
            if(valid) {
                if(count == 1) product.first = std::stoi(routeID.substr(beg, pos - beg));
                if(count == 2) product.second = std::stoi(routeID.substr(beg, pos - beg));
            } else {
                product.first = -1;
                product.second = -1;
                break;
            }
        }
    }
    return product;
}
