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


nearest_request_t* nearest_request_create(double latitude, double longitude);
void nearest_request_destroy(nearest_request_t *value);
void general_options_destroy(general_options_t& value);

void nearest_result_destroy(nearest_result_t *value);




#ifdef __cplusplus
}
#endif

#endif //C_OSRM_C_OSRM_H
