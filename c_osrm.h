#ifndef C_OSRM_C_OSRM_H
#define C_OSRM_C_OSRM_H


#ifdef __cplusplus
extern "C" {
#endif

#include "engine_config.h"

typedef struct waypoint
{
    long long nodes[2];
    char* hint;
    double distance;
    char* name;
    double location[2];
} waypoint_t;

typedef struct osrm_bearing
{
    short bearing;
    short range;

} osrm_bearing_t;

enum osrm_approaches
{
    UNRESTRICTED = 0,
    CURB = 1,
};

typedef struct nearest_result
{
    char* code;
    char* message;
    waypoint_t* waypoints;
    int number_of_waypoints;
} nearest_result_t;



typedef struct nearest_request
{
    double latitude;
    double longitude;
    unsigned number_of_results;
    double radius;
    osrm_bearing_t* bearing;
    char generate_hints;
    char* hint;
    enum osrm_approaches approach;
    char* excluded;

} nearest_request_t;


enum status
{
    Ok = 0,
    Error = 1
};

struct c_osrm;
typedef struct c_osrm c_osrm_t;

c_osrm_t *osrm_create(engine_config_t *config);
void osrm_destroy(c_osrm_t *osrm);

enum status osrm_nearest(c_osrm_t *c_osrm, nearest_request_t* request, nearest_result_t** result);


nearest_request_t* nearest_request_create(double latitude, double longitude);
void nearest_request_destroy(nearest_request_t *value);

void nearest_result_destroy(nearest_result_t *value);




#ifdef __cplusplus
}
#endif

#endif //C_OSRM_C_OSRM_H
