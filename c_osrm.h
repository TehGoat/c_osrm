#ifndef C_OSRM_C_OSRM_H
#define C_OSRM_C_OSRM_H


#ifdef __cplusplus
extern "C" {
#endif

#include "engine_config.h"

struct waypoint
{
    unsigned nodes[2];
    char* hint;
    double distance;
    char* name;
    double location[2];
};


typedef struct waypoint waypoint_t;

struct nearest_result
{
    char* code;
    char* message;
    waypoint_t* waypoints;
    int number_of_waypoints;
};

typedef struct nearest_result nearest_result_t;

enum status
{
    Ok = 0,
    Error = 1
};

struct c_osrm;
typedef struct c_osrm c_osrm_t;

c_osrm_t *osrm_create(engine_config_t *config);
void osrm_destroy(c_osrm_t *osrm);

enum status osrm_nearest(c_osrm_t *c_osrm, double lat, double lon, unsigned number_of_results, nearest_result_t** result);

void nearest_result_destroy(nearest_result_t *value);




#ifdef __cplusplus
}
#endif

#endif //C_OSRM_C_OSRM_H
