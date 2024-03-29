#ifndef TraCIDemo11p_H
#define TraCIDemo11p_H
#include <string>
#include <stdint.h>
#include <map>
#include <list>
#include <vector>
#include <math.h>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"


class Uninfected : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
    protected:
        struct CongestionInfo {
            int creatorID;
            std::string edgeID;
            double avgSpeed;
            simtime_t timestamp;
            std::string toString() {
                return "creatorID=" + std::to_string(creatorID) + ",edgeID=" + edgeID + ",avgSpeed=" + std::to_string(avgSpeed) + ",timestamp=" + timestamp.str();
            }
        };
        struct CongestionInfo cInfo;
        std::map<std::string, struct CongestionInfo> cidb;     // carID, cInfo struct
        // basically insert your cidb here
        std::list<std::map<std::string, struct CongestionInfo>> candidateRoutes;
        double avgEdgeSpeed;
        simtime_t lastDroveAt;
        // for route following algo
        std::list<std::string> chosenRoute;
        std::string prevRoad;
        bool changedRoute;
        bool hasChosen;
        bool sentMessage;
        int currentSubscribedServiceId;
    protected:
        virtual void onWSM(WaveShortMessage* wsm);
        virtual void onWSA(WaveServiceAdvertisment* wsa);
        virtual void onBSM(BasicSafetyMessage* bsm);

        virtual void handleSelfMsg(cMessage* msg);
        virtual void handlePositionUpdate(cObject* obj);
        virtual struct CongestionInfo stringToStruct(std::string str);
        virtual void updateDB(struct CongestionInfo cinfo);
        virtual void updateSelf();
        virtual void sendBeacon(struct CongestionInfo cInfo);
        virtual double calculateCIDBAvg(std::map<std::string, struct CongestionInfo> cidb);
        virtual std::string listStructToString(std::list<struct CongestionInfo> cidbResponse);
        virtual std::list<struct CongestionInfo> stringToListStruct(std::string cidbStr);
        virtual void sendCongestionRequest(std::string edgeIDs);
        virtual void sendCongestionResponse(std::list<struct CongestionInfo> cidbResponse);
        virtual std::string listToString(std::list<std::string> lanes);
        virtual std::list<std::string> stringToList(std::string edges);
        virtual void reroute();
        virtual void rerouteHelper(std::list<std::string> adjacentEdges);
        virtual void changeRoute();
        void cleanRoadList() ;
        virtual std::string getElementByIndex(std::list<std::string> ls, int index);
        virtual std::list<std::string> getRouteListByCarID(std::list<std::string> ls, int carID);
        virtual std::pair<int,int> iterateRouteID(std::string routeID);
        virtual std::list<std::string> getAdjacentEdges();
        //virtual std::list<std::string> getTargetAdjacentEdges();
        virtual std::list<std::string> roadListStringToRoadList(std::string routeStr);
        virtual std::string routeListToRoadListString(std::list<std::string> routeList);
        virtual bool contains_struct(std::list<struct CongestionInfo> cidbResponse, std::string query);
        //virtual bool isOnTarget();
};

#endif
