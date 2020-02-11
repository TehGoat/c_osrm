//
// Created by tehkoza on 2020-02-10.
//

#ifndef C_OSRM_PARAMETERS_H
#define C_OSRM_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

enum boolean {
    FALSE = 0,
    TRUE = 1
};

typedef struct nearest_waypoint
{
    long long nodes[2];
    char* hint;
    double distance;
    char* name;
    double location[2];
} nearest_waypoint_t;

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
    DURATION = 0,
    DISTANCE = 1,
    DURATION_DISTANCE = 2
};

typedef struct coordinate{
    double latitude;
    double longitude;
} coordinate_t;

typedef struct general_option
{
    coordinate_t coordinates;
    int number_of_coordinates;
    osrm_bearing_t* bearings;
    double* radiuses;
    enum boolean generate_hints;
    char* hints;
    enum osrm_approaches* approaches;
    char* exclude;
    int number_of_excludes;
}general_options_t;

typedef struct nearest_request
{
    general_options_t general_options;
    unsigned number_of_results;
} nearest_request_t;

typedef struct table_request
{
    general_options_t* general_options;
    int* sources;
    int* destinations;
    enum osrm_annotations annotations;
    double fallback_speed;
    enum osrm_fallback_coordinate fallback_coordinate;
    double scale_factor;
} table_request_t;

typedef struct nearest_result
{
    char* code;
    char* message;
    nearest_waypoint_t* waypoints;
    int number_of_waypoints;
} nearest_result_t;


#ifdef __cplusplus
}
#endif


#endif //C_OSRM_PARAMETERS_H
