//
// Created by tehkoza on 2020-02-10.
//

#ifndef C_OSRM_PARAMETERS_H
#define C_OSRM_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <float.h>

enum boolean {
    FALSE,
    TRUE
};

enum Algorithm
{
    CH,
    CoreCH, // Deprecated, will be removed in v6.0
    MLD
};

enum GeometriesType
{
    Polyline,
    Polyline6,
    GeoJSON
};

enum OverviewType
{
    Simplified,
    Full,
    False
};

enum AnnotationsType
{
    None,
    Duration,
    Nodes,
    Distance,
    Weight,
    Datasources,
    Speed,
    All
};

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


enum continue_straight
{
    CONTINUE_STRAIGHT_NONE = 0,
    CONTINUE_STRAIGHT_TRUE = 1,
    CONTINUE_STRAIGHT_FALSE = 2,
};

typedef struct engine_config
{
    char* storage_config;
    int max_locations_trip;
    int max_locations_viaroute;
    int max_locations_distance_table;
    int max_locations_map_matching;
    double max_radius_map_matching ;
    int max_results_nearest;
    int max_alternatives; // set an arbitrary upper bound; can be adjusted by user
    enum boolean use_shared_memory;
    char* memory_file;
    enum boolean use_mmap;
    enum Algorithm algorithm;
    char* verbosity;
    char* dataset_name;
} engine_config_t;


typedef struct coordinate{
    double latitude;
    double longitude;
} coordinate_t;

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

typedef struct osrm_lane
{
    char** indications;
    enum boolean valid;
} osrm_lane_t;

typedef struct osrm_intersections
{
    coordinate_t location;
    int* bearings;
    int number_of_bearings;
    char** classes;
    int number_of_classes;
    enum boolean* entry;
    int number_of_entries;
    long long in;
    long long out;
    osrm_lane_t* lanes;
    int number_of_lanes;
} osrm_intersections_t;

typedef struct osrm_maneuver
{
    int bearing_before;
    int bearing_after;
    coordinate_t location;
    char* type;
    char* modifer;
} osrm_maneuver_t;

typedef struct osrm_step
{
    double distance;
    double duration;
    char* geometry;
    double weight;
    char* name;
    char* ref;
    char* pronunciation;
    //destinations?
    char* exits;
    char* mode;
    osrm_maneuver_t* maneuver;
    osrm_intersections_t* intersections;
    int number_of_intersections;
    char* rotary_name;
    char* rotary_pronunciation;
    char* driving_side;
} osrm_step_t;

typedef struct osrm_metadata
{
    char** datasource_names;
    int number_of_datasource_names;
} osrm_metadata_t;

typedef struct osrm_annotation
{
    double* duration;
    double* distance;
    double* speed;
    double* weight;
    long long* nodes;
    int* datasources;
    osrm_metadata_t* metadata;
} osrm_annotation_t;

typedef struct osrm_route_legs
{
    osrm_annotation_t* annotation;
    double duration;
    char* summary;
    double weight;
    double distance;
    osrm_step_t* steps;
    int number_of_steps;
} osrm_route_legs_t;

typedef struct osrm_route
{
    double duration;
    double distance;
    char* weight_name;
    double weight;
    char* geometry;
    osrm_route_legs_t* legs;
    int number_of_legs;
} osrm_route_t;

typedef struct osrm_bearing
{
    short bearing;
    short range;

} osrm_bearing_t;

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
} general_options_t;

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
    enum osrm_annotations annotations;
    double fallback_speed;
    enum osrm_fallback_coordinate fallback_coordinate;
    double scale_factor;
} table_request_t;

typedef struct route_request
{
    general_options_t general_options;
    enum boolean steps;
    enum boolean alternatives;
    unsigned number_of_alternatives;
    enum boolean annotations;
    enum AnnotationsType annotations_type;
    enum GeometriesType geometries;
    enum OverviewType overview;
    enum continue_straight continue_straight;
    unsigned long long* waypoints;
    int number_of_waypoints;

} route_request_t;


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

typedef struct route_result
{
    char* code;
    char* message;
    waypoint_t* waypoints;
    int number_of_waypoints;
    osrm_route_t* routes;
    int number_of_routes;
} route_result_t;


#ifdef __cplusplus
}
#endif


#endif //C_OSRM_PARAMETERS_H
