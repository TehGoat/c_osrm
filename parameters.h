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

enum Gap
{
    Split,
    ignore
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

enum trip_start
{
    START_ANY = 0,
    FIRST = 1,
};

enum trip_end
{
    END_ANY = 0,
    LAST = 1,
};

struct engine_config
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
} const engine_config_default = {0, -1, -1, -1, -1, -1.0, -1, 3, 
                            TRUE, 0,  TRUE, CH, 0, 0};

typedef struct engine_config engine_config_t;


struct coordinate{
    double latitude;
    double longitude;
}const  coordinate_default = {0,0};

typedef struct coordinate coordinate_t;

struct nearest_waypoint
{
    long long nodes[2];
    char* hint;
    double distance;
    char* name;
    double location[2];
} const nearest_waypoint_default = {{0,0}, 0, 0, 0, {0,0}};

typedef struct nearest_waypoint nearest_waypoint_t;

struct waypoint
{
    char* hint;
    double distance;
    char* name;
    double location[2];
} const waypoint_default = {0, 0, 0, {0,0}};

typedef struct waypoint waypoint_t;

struct match_waypoint
{
    char* hint;
    double distance;
    char* name;
    double location[2];
    int matchings_index;
    int waypoint_index;
    int alternatives_count;
} const match_waypoint_default = {0, 0, 0, {0,0}, 0, 0, 0};

typedef struct match_waypoint match_waypoint_t;

struct trip_waypoint
{
    char* hint;
    double distance;
    char* name;
    double location[2];
    int trips_index;
    int waypoint_index;
} const trip_waypoint_default = {0, 0, 0, {0,0}, 0, 0};

typedef struct trip_waypoint trip_waypoint_t;

struct osrm_lane
{
    char** indications;
    enum boolean valid;
} const osrm_lane_default = {0, FALSE};

typedef struct osrm_lane osrm_lane_t;

struct osrm_intersections
{
    coordinate_t location;
    int* bearings;
    int number_of_bearings;
    char** classes;
    int number_of_classes;
    enum boolean* entry;
    int number_of_entries;
    int in;
    int out;
    osrm_lane_t* lanes;
    int number_of_lanes;
} const osrm_intersections_default = {coordinate_default, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct osrm_intersections osrm_intersections_t;

struct osrm_maneuver
{
    int bearing_before;
    int bearing_after;
    coordinate_t location;
    char* type;
    char* modifer;
} const osrm_maneuver_default = {0, 0, coordinate_default, 0, 0};

typedef struct osrm_maneuver osrm_maneuver_t;

struct osrm_step
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
} const osrm_step_default = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct osrm_step osrm_step_t;

struct osrm_metadata
{
    char** datasource_names;
    int number_of_datasource_names;
} const osrm_metadata_default = {0, 0};

typedef struct osrm_metadata osrm_metadata_t;

struct osrm_annotation
{
    double* duration;
    double* distance;
    double* speed;
    double* weight;
    long* nodes;
    int* datasources;
    osrm_metadata_t* metadata;
    int number_of_coordinates;
} const osrm_annotation_default = {0, 0, 0, 0, 0, 0, 0};

typedef struct osrm_annotation osrm_annotation_t;

struct osrm_route_legs
{
    osrm_annotation_t* annotation;
    double duration;
    char* summary;
    double weight;
    double distance;
    osrm_step_t* steps;
    int number_of_steps;
} const osrm_route_legs_default = {0, 0, 0, 0, 0, 0, 0};

typedef struct osrm_route_legs osrm_route_legs_t;

struct osrm_route
{
    double duration;
    double distance;
    char* weight_name;
    double weight;
    char* geometry;
    osrm_route_legs_t* legs;
    int number_of_legs;
} const osrm_route_default = {0, 0, 0, 0, 0, 0, 0};

typedef struct osrm_route osrm_route_t;

struct match_osrm_route
{
    double duration;
    double distance;
    char* weight_name;
    double weight;
    char* geometry;
    osrm_route_legs_t* legs;
    int number_of_legs;
    float confidence;
} const match_osrm_route_default = {0, 0, 0, 0, 0, 0, 0};

typedef struct match_osrm_route match_osrm_route_t;

struct osrm_bearing
{
    short bearing;
    short range;

} const osrm_bearing_default = {0, 0};

typedef struct osrm_bearing osrm_bearing_t;

struct general_option
{
    coordinate_t* coordinates;
    int number_of_coordinates;
    osrm_bearing_t** bearings;
    double** radiuses;
    enum boolean generate_hints;
    enum boolean skip_waypoints;
    char** hints;
    enum osrm_approaches** approaches;
    char** exclude;
    int number_of_excludes;
} const general_options_default = {0, 0, 0, 0, TRUE, FALSE, 0, 0, 0, 0};

typedef struct general_option general_options_t;

struct nearest_request
{
    general_options_t general_options;
    int number_of_results;
} const nearest_request_default = {general_options_default, 1};

typedef struct nearest_request nearest_request_t;

struct table_request
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
} const table_request_default = {general_options_default, 0, 0, 0, 0, DURATION, 
                            __DBL_MAX__, INPUT, 1};

typedef struct table_request table_request_t;

struct route_request
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

} const route_request_default = {general_options_default, FALSE, FALSE, 0, FALSE, 
                        None, Polyline, Simplified, CONTINUE_STRAIGHT_NONE, 0, 0};

typedef struct route_request route_request_t;

struct match_request 
{
    general_options_t general_options;
    enum boolean steps;
    enum GeometriesType geometries;
    enum boolean annotations;
    enum AnnotationsType annotations_type;
    enum OverviewType overview;
    int* timestamps;
    enum Gap gaps;
    enum boolean tidy;
    int* waypoits;
    int number_of_waypoints;
}const match_request_default = {general_options_default, FALSE, Polyline, FALSE, 
                        None, Simplified, 0, Split, FALSE, 0, 0};

typedef struct match_request match_request_t;

struct trip_request 
{
    general_options_t general_options;
    enum boolean roundtrip;
    enum trip_start source;
    enum trip_end destination;
    enum boolean steps;
    enum boolean annotations;
    enum AnnotationsType annotations_type;
    enum GeometriesType geometries;
    enum OverviewType overview;
}const trip_request_default = {general_options_default, FALSE, START_ANY, END_ANY, FALSE, 
                                FALSE, None, Polyline, Simplified};

typedef struct trip_request trip_request_t;

struct tile_request 
{
    int x;
    int y;
    int z;
}const tile_request_default = {0, 0, 0};

typedef struct tile_request tile_request_t;


struct nearest_result
{
    char* code;
    char* message;
    nearest_waypoint_t* waypoints;
    int number_of_waypoints;
} const nearest_result_default = {0, 0, 0, 0};

typedef struct nearest_result nearest_result_t;

struct table_result
{
    char* code;
    char* message;
    double* durations;
    double* distances;
    waypoint_t* sources;
    waypoint_t* destinations;
    int number_of_sources;
    int number_of_destinations;
} const table_result_default = {0, 0, 0, 0, 0, 0, 0, 0};

typedef struct table_result table_result_t;

struct route_result
{
    char* code;
    char* message;
    waypoint_t* waypoints;
    int number_of_waypoints;
    osrm_route_t* routes;
    int number_of_routes;
} const route_result_default = {0, 0, 0, 0, 0, 0};

typedef struct route_result route_result_t;

struct match_result 
{
    char* code;
    char* message;
    match_waypoint_t* tracepoints;
    int number_of_tracepoints;
    match_osrm_route_t* matchings;
    int number_of_matchings;
} const match_result_default = {0, 0};

typedef struct match_result match_result_t;

struct trip_result
{
    char* code;
    char* message;
    trip_waypoint_t* waypoints;
    int number_of_waypoints;
    osrm_route_t* trips;
    int number_of_trips;
} const trip_result_default = {0, 0, 0, 0, 0, 0};

typedef struct trip_result trip_result_t;

struct tile_result
{
    char* result;
    int string_length;
} const tile_result_default = {};

typedef struct tile_result tile_result_t;



#ifdef __cplusplus
}
#endif


#endif //C_OSRM_PARAMETERS_H
