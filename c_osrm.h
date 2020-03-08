#ifndef C_OSRM_C_OSRM_H
#define C_OSRM_C_OSRM_H


#ifdef __cplusplus
extern "C" {
#endif

#include "parameters.h"

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

enum status osrm_table(c_osrm_t *c_osrm, table_request_t* request, table_result_t** result);

enum status osrm_route(c_osrm_t *c_osrm, route_request_t* request, route_result_t** result);

enum status osrm_match(c_osrm_t *c_osrm, match_request_t* request, match_result_t** result);

void nearest_result_destroy(nearest_result_t *value);
void table_result_destroy(table_result_t *value);
void route_result_destroy(route_result_t *value);
void match_result_destroy(match_result_t *value);




#ifdef __cplusplus
}
#endif

#endif //C_OSRM_C_OSRM_H
