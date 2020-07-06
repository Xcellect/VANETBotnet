Step 1: Implement the Congestion avoidance protocol
----------------------------------------------------------------
A. Message Types and Information Exchange
Two types of messages are implemented in our congestion avoidance simulator: 

1) Congestion Request Messages: When a car approaches
the end of a road and can choose from two or more routes, it
will need congestion measurements from other cars in order
to make the choice that will minimize its remaining trip time.
Hence, it broadcasts a congestion request message to all nearby
cars in its communication range to obtain this information. The
content of this message is a list of roads of interest; these are
the roads that form the candidate paths to destination. The car
will choose the path based on the responses it gets.

2) Congestion Response Messages: A congestion response
message is sent only when a congestion request is received and
there are related entries in the congestion information database
of the receiving car, ensuring that no superfluous information
is transmitted. The response includes congestion information
about the roads of interest available at the receiver.

3. These are in addition to the Basic Safety Messages (BSM) that are regularly 
sent to nearby vehicles
----------------------------------------------------------------

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
d. Timestamp: Time of the measurement’s creation to ensure
the freshness of measurements and prioritize most recent ones.

2) Maintainting CIDB: Let, m be this struct (entry), E be the set 
of all edges. DB = {m1,m2,m3,...,mn}, 1 <= n <= |E|. 2 cases for
an entry: 
a. after creating its own measurement, sampling own and
other cars' speeds on its road. after t time threshold, its own
measurement gets outdated in the DB. a car will prioritize its
own entry over fresher ones unless it is at least t older. makes
it kinda secure.
b. after obtaining measurements thru congestion responses 

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