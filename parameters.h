//
// Created by tehkoza on 2020-02-10.
//

#ifndef C_OSRM_PARAMETERS_H
#define C_OSRM_PARAMETERS_H

#include <cfloat>

#ifdef __cplusplus
extern "C" {
#endif

enum boolean {
    FALSE = 0,
    TRUE = 1
};

enum class Algorithm
{
    CH,
    CoreCH, // Deprecated, will be removed in v6.0
    MLD
};

typedef struct engine_config
{
    char* storage_config;
    int max_locations_trip = -1;
    int max_locations_viaroute = -1;
    int max_locations_distance_table = -1;
    int max_locations_map_matching = -1;
    double max_radius_map_matching = -1.0;
    int max_results_nearest = -1;
    int max_alternatives = 3; // set an arbitrary upper bound; can be adjusted by user
    bool use_shared_memory = true;
    char* memory_file;
    bool use_mmap = true;
    Algorithm algorithm = Algorithm::CH;
    char* verbosity;
    char* dataset_name;
} engine_config_t;

typedef struct nearest_waypoint
{
    long long nodes[2];
    char* hint;
    double distance;
    char* name;
    double location[2];
} nearest_waypoint_t;

typedef struct waypoint
{
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

enum osrm_fallback_coordinate
{
    INPUT = 0,
    SNAPPED = 1
};

enum osrm_annotations
{
    NONE = 0,
    DURATION = 1,
    DISTANCE = 2,
    ALL = 3
};

typedef struct coordinate{
    double latitude;
    double longitude;
} coordinate_t;

typedef struct general_option
{
    coordinate_t* coordinates;
    int number_of_coordinates;
    osrm_bearing_t* bearings;
    double* radiuses;
    enum boolean generate_hints;
    char* hints;
    enum osrm_approaches* approaches;
    char** exclude;
    int number_of_excludes;
}general_options_t;

typedef struct nearest_request
{
    general_options_t general_options;
    unsigned number_of_results;
} nearest_request_t;

typedef struct table_request
{
    general_options_t general_options;
    int* sources;
    int number_of_sources;
    int* destinations;
    int number_of_destinations;
    enum osrm_annotations annotations = osrm_annotations::DURATION;
    double fallback_speed = DBL_MAX;
    enum osrm_fallback_coordinate fallback_coordinate = osrm_fallback_coordinate::INPUT;
    double scale_factor = 1;
} table_request_t;


typedef struct nearest_result
{
    char* code;
    char* message;
    nearest_waypoint_t* waypoints;
    int number_of_waypoints;
} nearest_result_t;

typedef struct table_result
{
    char* code;
    char* message;
    double* durations;
    waypoint_t* sources;
    waypoint_t* destinations;
    int number_of_sources;
    int number_of_destinations;
} table_result_t;


#ifdef __cplusplus
}
#endif


#endif //C_OSRM_PARAMETERS_H
