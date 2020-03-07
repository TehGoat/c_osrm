#include "c_osrm.h"
#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/route_parameters.hpp"

#include <string>
#include <iostream>
#include <engine/approach.hpp>
#include <storage/storage_config.hpp>


using namespace osrm;

struct c_osrm {
    void *obj;
};


char* get_string(std::string key, json::Object &json);
char* get_string_from_string(json::String &value);

void parse_route(route_result_t *return_result, const json::Array &routes);
void parse_route_leg(osrm_route_t &route, const json::Array &routes_legs);
void parse_annotation(osrm_route_legs_t &route, json::Object &annotation);
void parse_step(osrm_route_legs_t &route, json::Array &steps);
void parse_maneuver(osrm_step_t &step, json::Object &maneuver);
void parse_intersections(osrm_step_t &step, json::Array &intersections);
void parse_lanes(osrm_intersections_t &intersections, json::Array &lanes);

void destroy_lanes(osrm_lane_t  *lanes, int number_of_lanes);
void destroy_intersections(osrm_intersections_t *intersections, int number_of_intersections);
void destroy_maneuver(osrm_maneuver_t *maneuver);
void destroy_steps(osrm_step_t  *steps, int number_of_steps);
void destroy_annotation(osrm_annotation_t  *annotation);
void destroy_route_leg(osrm_route_legs_t *route_legs, int number_of_routes_legs);
void destroy_route(osrm_route_t *routes, int number_of_routes);

c_osrm_t *osrm_create(engine_config_t *config)
{
    c_osrm_t *osrm;
    OSRM *osrm_osrm;

    EngineConfig osrm_config;
    osrm_config.storage_config = storage::StorageConfig(boost::filesystem::path(config->storage_config));
    osrm_config.max_locations_trip = config->max_locations_trip;
    osrm_config.max_locations_viaroute = config->max_locations_viaroute;
    osrm_config.max_locations_distance_table = config->max_locations_distance_table;
    osrm_config.max_locations_map_matching = config->max_locations_map_matching;
    osrm_config.max_radius_map_matching = config->max_radius_map_matching;
    osrm_config.max_results_nearest = config->max_results_nearest;
    osrm_config.max_alternatives = config->max_alternatives;
    osrm_config.use_shared_memory = config->use_shared_memory;
    if(config->memory_file != NULL)
    {
        osrm_config.memory_file = config->memory_file;
    }
    osrm_config.use_mmap = config->use_mmap;
    if(config->verbosity != NULL)
    {
        osrm_config.verbosity = config->verbosity;
    }
    if(config->dataset_name != NULL)
    {
        osrm_config.dataset_name = config->dataset_name;
    }

    switch (config->algorithm)
    {

        case Algorithm::CH:
            osrm_config.algorithm = EngineConfig::Algorithm::CH;
            break;
        case Algorithm::CoreCH:
            osrm_config.algorithm = EngineConfig::Algorithm::CoreCH;
            break;
        case Algorithm::MLD:
            osrm_config.algorithm = EngineConfig::Algorithm::MLD;
            break;
    }


    osrm = (typeof(osrm))malloc(sizeof(*osrm));
    osrm_osrm = new OSRM(osrm_config);
    osrm->obj = osrm_osrm;

    return osrm;
}

void osrm_destroy(c_osrm_t *c_osrm)
{
    if(c_osrm == nullptr)
    {
        return;
    }

    delete static_cast<OSRM*>(c_osrm->obj);
    free(c_osrm);
}

enum status osrm_nearest(c_osrm_t *c_osrm, nearest_request_t* request, nearest_result_t** result)
{
    OSRM *osrm =static_cast<OSRM*>(c_osrm->obj);

    NearestParameters parameters;
    parameters.coordinates.emplace_back(
            util::FloatLongitude{request->general_options.coordinates[0].longitude},
            util::FloatLatitude{request->general_options.coordinates[0].latitude}
            );

    parameters.number_of_results = request->number_of_results;

    if(request->general_options.radiuses != NULL )
    {
        parameters.radiuses.emplace_back(*request->general_options.radiuses);
    }

    if(request->general_options.bearings != NULL)
    {
        engine::Bearing bearing{};
        bearing.bearing = request->general_options.bearings->bearing;
        bearing.range = request->general_options.bearings->range;
        parameters.bearings.emplace_back(bearing);
    }

    parameters.generate_hints = request->general_options.generate_hints == TRUE;

    parameters.skip_waypoints = request->general_options.skip_waypoints == TRUE;

    if(request->general_options.hints != NULL)
    {
        for(int i = 0; i < request->general_options.number_of_coordinates; i++)
        {
            parameters.hints.emplace_back(osrm::engine::Hint::FromBase64(request->general_options.hints[i]));
        }
    }

    if(request->general_options.approaches != NULL &&
        *request->general_options.approaches == CURB)
    {
        parameters.approaches.emplace_back(engine::Approach::CURB);
    }

    if(request->general_options.exclude != NULL)
    {
       for(int i = 0; i < request->general_options.number_of_excludes; i++)
       {
           parameters.exclude.emplace_back(request->general_options.exclude[i]);
       }
    }


    engine::api::ResultT osr_result = json::Object();

    const auto status = osrm->Nearest(parameters, osr_result);

    auto &json_result = osr_result.get<json::Object>();

    if(*result != NULL)
    {
        free(result);
    }

    nearest_result *return_result = NULL;
    return_result = (typeof(return_result))malloc(sizeof(*return_result));
    *return_result = nearest_result_default;
    
    return_result->code = get_string("code", json_result);

    if (status == Status::Ok)
    {
        const auto waypoints = json_result.values["waypoints"].get<json::Array>().values;

        if(waypoints.empty())
        {
            return status::Ok;
        }

        return_result->waypoints = static_cast<nearest_waypoint_t  *>(malloc(sizeof(nearest_waypoint_t) * waypoints.size()));
        return_result->number_of_waypoints = waypoints.size();

        for(int i = 0; i < waypoints.size(); i++)
        {
            auto waypoint = waypoints[i].get<json::Object>();
            const auto distance = waypoint.values["distance"].get<json::Number>().value;
            auto location = waypoint.values["location"].get<json::Array>().values;
            auto nodes = waypoint.values["nodes"].get<json::Array>().values;

            return_result->waypoints[i].nodes[0] = nodes[0].get<json::Number>().value;
            return_result->waypoints[i].nodes[1] = nodes[1].get<json::Number>().value;

            return_result->waypoints[i].hint = get_string("hint", waypoint);
            return_result->waypoints[i].name= get_string("name", waypoint);

            return_result->waypoints[i].distance = distance;

            return_result->waypoints[i].location[0] = location[0].get<json::Number>().value;
            return_result->waypoints[i].location[1] = location[1].get<json::Number>().value;

        }

        *result = return_result;

        return status::Ok;

    }
    else
    {
        return_result->message = get_string("message", json_result);

        *result = return_result;

        return status::Error;
    }
}

enum status osrm_table(c_osrm_t *c_osrm, table_request_t* request, table_result_t** result)
{
    OSRM *osrm =static_cast<OSRM*>(c_osrm->obj);

    TableParameters parameters;

    for(int i = 0; i < request->general_options.number_of_coordinates; i++)
    {
        parameters.coordinates.emplace_back(
                util::FloatLongitude{request->general_options.coordinates[i].longitude},
                util::FloatLatitude{request->general_options.coordinates[i].latitude}
                );
    }

    if(request->sources != NULL)
    {
        for(int i = 0; i < request->number_of_sources; i++)
        {
            parameters.sources.emplace_back(request->sources[i]);
        }
    }

    if(request->destinations != NULL)
    {
        for(int i = 0; i < request->number_of_destinations; i++)
        {
            parameters.destinations.emplace_back(request->destinations[i]);
        }
    }

    parameters.fallback_speed = request->fallback_speed;

    switch (request->fallback_coordinate)
    {

        case INPUT:
            parameters.fallback_coordinate_type = TableParameters::FallbackCoordinateType::Input;
            break;
        case SNAPPED:
            parameters.fallback_coordinate_type = TableParameters::FallbackCoordinateType::Snapped;
            break;
    }

    switch (request->annotations)
    {

        case NONE:
            parameters.annotations = TableParameters::AnnotationsType::None;
            break;
        case DURATION:
            parameters.annotations = TableParameters::AnnotationsType::Duration;
            break;
        case DISTANCE:
            parameters.annotations = TableParameters::AnnotationsType::Distance;
            break;
        case ALL:
            parameters.annotations = TableParameters::AnnotationsType::All;
            break;
    }

    parameters.scale_factor = request->scale_factor;


    if(request->general_options.radiuses != NULL )
    {
        parameters.radiuses.emplace_back(*request->general_options.radiuses);
    }

    if(request->general_options.bearings != NULL)
    {
        engine::Bearing bearing{};
        bearing.bearing = request->general_options.bearings->bearing;
        bearing.range = request->general_options.bearings->range;
        parameters.bearings.emplace_back(bearing);
    }

    parameters.generate_hints = request->general_options.generate_hints == TRUE;

    parameters.skip_waypoints = request->general_options.skip_waypoints == TRUE;

    if(request->general_options.hints != NULL)
    {
        for(int i = 0; i < request->general_options.number_of_coordinates; i++)
        {
            parameters.hints.emplace_back(osrm::engine::Hint::FromBase64(request->general_options.hints[i]));
        }
    }

    if(request->general_options.approaches != NULL &&
       *request->general_options.approaches == CURB)
    {
        parameters.approaches.emplace_back(engine::Approach::CURB);
    }

    if(request->general_options.exclude != NULL)
    {
       for(int i = 0; i < request->general_options.number_of_excludes; i++)
       {
           parameters.exclude.emplace_back(request->general_options.exclude[i]);
       }
    }

    engine::api::ResultT osr_result = json::Object();

    const auto status = osrm->Table(parameters, osr_result);

    auto &json_result = osr_result.get<json::Object>();

    if(*result != NULL)
    {
        free(result);
    }

    table_result_t *return_result = NULL;
    return_result = (typeof(return_result))malloc(sizeof(*return_result));
    *return_result = table_result_default;


    return_result->code = get_string("code", json_result);

    if (status == Status::Ok)
    {
        const auto sources = json_result.values["sources"].get<json::Array>().values;
        const auto destinations = json_result.values["destinations"].get<json::Array>().values;
        const auto durations = json_result.values["durations"].get<json::Array>().values;

        return_result->sources = static_cast<waypoint_t  *>(malloc(sizeof(waypoint_t) * sources.size()));
        return_result->number_of_sources = sources.size();

        for(int i = 0; i < sources.size(); i++)
        {
            auto source = sources[i].get<json::Object>();
            const auto distance = source.values["distance"].get<json::Number>().value;
            auto location = source.values["location"].get<json::Array>().values;

            return_result->sources[i].hint = get_string("hint", source);
            return_result->sources[i].name = get_string("name", source);

            return_result->sources[i].distance = distance;

            return_result->sources[i].location[0] = location[0].get<json::Number>().value;
            return_result->sources[i].location[1] = location[1].get<json::Number>().value;

        }

        return_result->destinations = static_cast<waypoint_t  *>(malloc(sizeof(waypoint_t) * destinations.size()));
        return_result->number_of_destinations = destinations.size();

        for(int i = 0; i < destinations.size(); i++)
        {
            auto destination = destinations[i].get<json::Object>();
            const auto distance = destination.values["distance"].get<json::Number>().value;
            auto location = destination.values["location"].get<json::Array>().values;

            return_result->destinations[i].hint = get_string("hint", destination);
            return_result->destinations[i].name = get_string("name", destination);

            return_result->destinations[i].distance = distance;

            return_result->destinations[i].location[0] = location[0].get<json::Number>().value;
            return_result->destinations[i].location[1] = location[1].get<json::Number>().value;

        }

        return_result->durations = static_cast<double*>(malloc(sizeof(double) * sources.size() * durations.size()));
        for(int i = 0; i < sources.size(); i++)
        {
            const auto durations_element = durations[i].get<json::Array>().values;

            for(int j = 0; j < durations_element.size(); j++)
            {
                return_result->durations[i * sources.size() +j] = durations_element[j].get<json::Number>().value;
            }
        }

        *result = return_result;

        return status::Ok;

    }
    else
    {
        return_result->message = get_string("message", json_result);

        *result = return_result;

        return status::Error;
    }

    return status::Error;
}


enum status osrm_route(c_osrm_t *c_osrm, route_request_t* request, route_result_t** result)
{
    OSRM *osrm =static_cast<OSRM*>(c_osrm->obj);

    RouteParameters parameters;

    for(int i = 0; i < request->general_options.number_of_coordinates; i++)
    {
        parameters.coordinates.emplace_back(
                util::FloatLongitude{request->general_options.coordinates[i].longitude},
                util::FloatLatitude{request->general_options.coordinates[i].latitude}
                );
    }
    
    parameters.steps = request->steps == boolean::TRUE;
    parameters.alternatives = request->alternatives == boolean::TRUE;
    parameters.number_of_alternatives = request->number_of_alternatives;
    parameters.annotations = request->annotations == boolean::TRUE;
    
    switch(request->annotations_type)
    {
        case AnnotationsType::None:
            parameters.annotations_type = RouteParameters::AnnotationsType::None;
        break;
        case AnnotationsType::Duration:
            parameters.annotations_type = RouteParameters::AnnotationsType::Duration;
        break;
        case AnnotationsType::Nodes:
            parameters.annotations_type = RouteParameters::AnnotationsType::Nodes;
        break;
        case AnnotationsType::Distance:
            parameters.annotations_type = RouteParameters::AnnotationsType::Distance;
        break;
        case AnnotationsType::Weight:
            parameters.annotations_type = RouteParameters::AnnotationsType::Weight;
        break;
        case AnnotationsType::Datasources:
            parameters.annotations_type = RouteParameters::AnnotationsType::Datasources;
        break;
        case AnnotationsType::Speed:
            parameters.annotations_type = RouteParameters::AnnotationsType::Speed;
        break;
        case AnnotationsType::All:
            parameters.annotations_type = RouteParameters::AnnotationsType::All;
        break;
    }

    switch(request->geometries)
    {
        case GeometriesType::Polyline:
            parameters.geometries = RouteParameters::GeometriesType::Polyline;
        break;
        case GeometriesType::Polyline6:
            parameters.geometries = RouteParameters::GeometriesType::Polyline6;
        break;
        case GeometriesType::GeoJSON:
            parameters.geometries = RouteParameters::GeometriesType::GeoJSON;
        break;
    }

    switch(request->overview)
    {
        case OverviewType::Simplified:
            parameters.overview = RouteParameters::OverviewType::Simplified;
        break;
        case OverviewType::Full:
            parameters.overview = RouteParameters::OverviewType::Full;
        break;
        case OverviewType::False:
            parameters.overview = RouteParameters::OverviewType::False;
        break;
    }

    switch(request->continue_straight)
    {
        case continue_straight::CONTINUE_STRAIGHT_NONE:
            parameters.continue_straight = NULL;
        break;
        case continue_straight::CONTINUE_STRAIGHT_TRUE:
            parameters.continue_straight = true;
        break;
        case continue_straight::CONTINUE_STRAIGHT_FALSE:
            parameters.continue_straight = false;
        break;
    }

    if(request->waypoints != NULL)
    {
        for(int i = 0; i < request->number_of_waypoints; i++)
        {
            parameters.waypoints.push_back(request->waypoints[i]);
        }
    }

    if(request->general_options.radiuses != NULL )
    {
        parameters.radiuses.emplace_back(*request->general_options.radiuses);
    }

    if(request->general_options.bearings != NULL)
    {
        engine::Bearing bearing{};
        bearing.bearing = request->general_options.bearings->bearing;
        bearing.range = request->general_options.bearings->range;
        parameters.bearings.emplace_back(bearing);
    }

    parameters.generate_hints = request->general_options.generate_hints == TRUE;

    parameters.skip_waypoints = request->general_options.skip_waypoints == TRUE;

    if(request->general_options.hints != NULL)
    {
        for(int i = 0; i < request->general_options.number_of_coordinates; i++)
        {
            parameters.hints.emplace_back(osrm::engine::Hint::FromBase64(request->general_options.hints[i]));
        }
    }

    if(request->general_options.approaches != NULL &&
       *request->general_options.approaches == CURB)
    {
        parameters.approaches.emplace_back(engine::Approach::CURB);
    }

    if(request->general_options.exclude != NULL)
    {
       for(int i = 0; i < request->general_options.number_of_excludes; i++)
       {
           parameters.exclude.emplace_back(request->general_options.exclude[i]);
       }
    }

    engine::api::ResultT osr_result = json::Object();


    const auto status = osrm->Route(parameters, osr_result);

    auto &json_result = osr_result.get<json::Object>();

    if(*result != NULL)
    {
        free(result);
    }

    route_result_t *return_result = NULL;
    return_result = (typeof(return_result))malloc(sizeof(*return_result));
    *return_result = route_result_default;

    return_result->code = get_string("code", json_result);

    if (status == Status::Ok)
    {
        if(json_result.values.find("waypoints") != json_result.values.end())
        {
            const auto waypoints = json_result.values["waypoints"].get<json::Array>().values;

            return_result->waypoints = static_cast<waypoint_t  *>(malloc(sizeof(waypoint_t) * waypoints.size()));
            return_result->number_of_waypoints = waypoints.size();

            for(int i = 0; i < waypoints.size(); i++)
            {
                auto waypoint = waypoints[i].get<json::Object>();
                return_result->waypoints[i].name = NULL;
                return_result->waypoints[i].hint = NULL;
                if(waypoint.values.find("name") != waypoint.values.end())
                {
                    return_result->waypoints[i].name = get_string("name", waypoint);
                }
                

                if(waypoint.values.find("hint") != waypoint.values.end())
                {
                    return_result->waypoints[i].hint = get_string("hint", waypoint);
                }

                if(waypoint.values.find("distance") != waypoint.values.end())
                {
                    return_result->waypoints[i].distance = waypoint.values["distance"].get<json::Number>().value;
                }

                if(waypoint.values.find("location") != waypoint.values.end())
                {
                    auto location = waypoint.values["location"].get<json::Array>().values;
                    return_result->waypoints[i].location[0] = location[0].get<json::Number>().value;
                    return_result->waypoints[i].location[1] = location[1].get<json::Number>().value;
                }
            }
        }
        


        if(json_result.values.find("routes") != json_result.values.end())
        {
            const json::Array routes = json_result.values["routes"].get<json::Array>();
            parse_route(return_result, routes);
        }

        *result = return_result;
        
        return status::Ok;
    }
    else
    {
        return_result->message = get_string("message", json_result);

        *result = return_result;

        return status::Error;
    }

    return status::Error;
}

void parse_route(route_result_t *return_result, const json::Array &routes)
{
    return_result->routes = static_cast<osrm_route_t  *>(malloc(sizeof(osrm_route_t) * routes.values.size()));
    return_result->number_of_routes = routes.values.size();
    for(int i = 0; i < routes.values.size(); i++)
    {
        auto route = routes.values[i].get<json::Object>();
        return_result->routes[i] = osrm_route_default;

        if(route.values.find("duration") != route.values.end())
        {
            return_result->routes[i].duration = route.values["duration"].get<json::Number>().value;
        }
        if(route.values.find("distance") != route.values.end())
        {
            return_result->routes[i].distance = route.values["distance"].get<json::Number>().value;
        }
        if(route.values.find("weight_name") != route.values.end())
        {
            return_result->routes[i].weight_name = get_string("weight_name", route);
        }
        if(route.values.find("weight") != route.values.end())
        {
            return_result->routes[i].distance = route.values["weight"].get<json::Number>().value;
        }
        if(route.values.find("geometry") != route.values.end())
        {
            return_result->routes[i].geometry = get_string("geometry", route);
        }
        if(route.values.find("legs") != route.values.end())
        {
            const json::Array routes_legs = route.values["legs"].get<json::Array>();
            parse_route_leg(return_result->routes[i], routes_legs);
        }
    }
}

void destroy_route(osrm_route_t *routes, int number_of_routes)
{
    if(routes == NULL)
    {
        return;
    }

    for(int i = 0; i < number_of_routes; i++)
    {
        if(routes[i].weight_name != NULL)
        {
            free(routes[i].weight_name);
        }
        if(routes[i].geometry != NULL)
        {
            free(routes[i].geometry);
        }
        if(routes[i].legs != NULL)
        {
            destroy_route_leg(routes[i].legs, routes[i].number_of_legs);
        }
    }

    free(routes);
}

void parse_route_leg(osrm_route_t &route, const json::Array &routes_legs)
{
    route.legs = static_cast<osrm_route_legs_t  *>(malloc(sizeof(osrm_route_legs_t) * routes_legs.values.size()));
    route.number_of_legs = routes_legs.values.size();
    for(int i = 0; i < routes_legs.values.size(); i++)
    {
        auto route_leg = routes_legs.values[i].get<json::Object>();
        route.legs[i] = osrm_route_legs_default;

        if(route_leg.values.find("annotation") != route_leg.values.end())
        {
            auto annotation = route_leg.values["annotation"].get<json::Object>();
            parse_annotation(route.legs[i], annotation);
        }
        if(route_leg.values.find("duration") != route_leg.values.end())
        {
            route.legs[i].duration = route_leg.values["duration"].get<json::Number>().value;
        }
        if(route_leg.values.find("summary") != route_leg.values.end())
        {
            route.legs[i].summary = get_string("summary", route_leg);
        }
        if(route_leg.values.find("weight") != route_leg.values.end())
        {
            route.legs[i].weight = route_leg.values["weight"].get<json::Number>().value;
        }
        if(route_leg.values.find("distance") != route_leg.values.end())
        {
            route.legs[i].distance = route_leg.values["distance"].get<json::Number>().value;
        }
        if(route_leg.values.find("steps") != route_leg.values.end())
        {
            auto steps = route_leg.values["steps"].get<json::Array>();
            parse_step(route.legs[i], steps);
        }
    }
}

void destroy_route_leg(osrm_route_legs_t *route_legs, int number_of_routes_legs)
{
    if(route_legs == NULL)
    {
        return;
    }

    for(int i = 0; i < number_of_routes_legs; i++)
    {
        if(route_legs[i].summary != NULL)
        {
            free(route_legs[i].summary);
        }
        if(route_legs[i].steps != NULL)
        {
            destroy_steps(route_legs[i].steps, route_legs[i].number_of_steps);
        }
        if(route_legs[i].annotation != NULL)
        {
            destroy_annotation(route_legs[i].annotation);
        }
    }

    free(route_legs);
}

void parse_annotation(osrm_route_legs_t &route, json::Object &annotation)
{
    route.annotation = static_cast<osrm_annotation_t  *>(malloc(sizeof(osrm_annotation_t)));
    *route.annotation = osrm_annotation_default;

    if(annotation.values.find("duration") != annotation.values.end())
    {
        const auto duration_array = annotation.values["duration"].get<json::Array>().values;
        route.annotation->duration = static_cast<double  *>(malloc(sizeof(double) * duration_array.size()));

        for(int i = 0; i < duration_array.size(); i++)
        {
            route.annotation->duration[i] = duration_array[i].get<json::Number>().value;
        }
    }
    if(annotation.values.find("distance") != annotation.values.end())
    {
        const auto distance_array = annotation.values["distance"].get<json::Array>().values;
        route.annotation->distance = static_cast<double  *>(malloc(sizeof(double) * distance_array.size()));
        route.annotation->number_of_coordinates = distance_array.size();
        
        for(int i = 0; i < distance_array.size(); i++)
        {
            route.annotation->distance[i] = distance_array[i].get<json::Number>().value;
        }
    }
    if(annotation.values.find("datasources") != annotation.values.end())
    {
        const auto datasources_array = annotation.values["datasources"].get<json::Array>().values;
        route.annotation->datasources = static_cast<int  *>(malloc(sizeof(int) * datasources_array.size()));
        for(int i = 0; i < datasources_array.size(); i++)
        {
            route.annotation->datasources[i] = datasources_array[i].get<json::Number>().value;
        }
    }
    if(annotation.values.find("nodes") != annotation.values.end())
    {
        const auto nodes_array = annotation.values["nodes"].get<json::Array>().values;
        route.annotation->datasources = static_cast<int  *>(malloc(sizeof(int) * nodes_array.size()));
        for(int i = 0; i < nodes_array.size(); i++)
        {
            route.annotation->nodes[i] = nodes_array[i].get<json::Number>().value;
        }
    }
    if(annotation.values.find("weight") != annotation.values.end())
    {
        const auto weight_array = annotation.values["weight"].get<json::Array>().values;
        route.annotation->weight = static_cast<double  *>(malloc(sizeof(double) * weight_array.size()));
        for(int i = 0; i < weight_array.size(); i++)
        {
            route.annotation->weight[i] = weight_array[i].get<json::Number>().value;
        }
    }
    if(annotation.values.find("speed") != annotation.values.end())
    {
        const auto speed_array = annotation.values["speed"].get<json::Array>().values;
        route.annotation->speed = static_cast<double  *>(malloc(sizeof(double) * speed_array.size()));
        for(int i = 0; i < speed_array.size(); i++)
        {
            route.annotation->speed[i] = speed_array[i].get<json::Number>().value;
        }
    }
    if(annotation.values.find("metadata") != annotation.values.end())
    {
        auto metadata = annotation.values["metadata"].get<json::Object>();
        route.annotation->metadata = static_cast<osrm_metadata_t *>(malloc(sizeof(osrm_metadata_t)));
        if(metadata.values.find("datasource_names") != metadata.values.end())
        {
            const auto datasource_names_array = metadata.values["datasource_names"].get<json::Array>().values;
            route.annotation->metadata->number_of_datasource_names = datasource_names_array.size();
            route.annotation->metadata->datasource_names = static_cast<char **>(malloc(sizeof(char*) * datasource_names_array.size()));
            for(int i = 0; i < datasource_names_array.size(); i++)
            {
                const auto datasource_name = datasource_names_array[i].get<json::String>().value;
                route.annotation->metadata->datasource_names[i] = (char*)malloc(sizeof(char) * (datasource_name.size() + 1));
                datasource_name.copy(route.annotation->metadata->datasource_names[i], datasource_name.size() + 1);
                route.annotation->metadata->datasource_names[i][datasource_name.size()] = '\0';
            }
        }
    }

}

void destroy_annotation(osrm_annotation_t  *annotation)
{
    if(annotation == NULL)
    {
        return;
    } 

    if(annotation->duration != NULL)
    {
        free(annotation->duration);
    }
    if(annotation->distance != NULL)
    {
        free(annotation->distance);
    }
    if(annotation->datasources != NULL)
    {
        free(annotation->datasources);
    }
    if(annotation->nodes != NULL)
    {
        free(annotation->nodes);
    }
    if(annotation->weight != NULL)
    {
        free(annotation->weight);
    }
    if(annotation->speed != NULL)
    {
        free(annotation->speed);
    }
    if(annotation->metadata != NULL)
    {
        if(annotation->metadata->datasource_names != NULL)
        {
            for(int i = 0; i < annotation->metadata->number_of_datasource_names; i++)
            {
                if(annotation->metadata->datasource_names[i] == NULL)
                {
                    continue;
                }

                free(annotation->metadata->datasource_names[i]);
            }
        }
        free(annotation->metadata);
    }
    

    free(annotation);
}

void parse_step(osrm_route_legs_t &route, json::Array &steps)
{
    route.steps = static_cast<osrm_step_t  *>(malloc(sizeof(osrm_step_t) * steps.values.size()));
    route.number_of_steps = steps.values.size();

    for(int i = 0; i < steps.values.size(); i++)
    {
        auto curret_json_step = steps.values[i].get<json::Object>();
        route.steps[i] = osrm_step_default;
        auto &current_step = route.steps[i];
        
        if(curret_json_step.values.find("distance") != curret_json_step.values.end())
        {
            current_step.distance = curret_json_step.values["distance"].get<json::Number>().value;
        }
        if(curret_json_step.values.find("duration") != curret_json_step.values.end())
        {
            current_step.duration = curret_json_step.values["duration"].get<json::Number>().value;
        }
        current_step.geometry = get_string("geometry", curret_json_step);
        if(curret_json_step.values.find("weight") != curret_json_step.values.end())
        {
            current_step.weight = curret_json_step.values["weight"].get<json::Number>().value;
        }
        current_step.name = get_string("name", curret_json_step);
        current_step.ref = get_string("ref", curret_json_step);
        current_step.pronunciation = get_string("pronunciation", curret_json_step);
        //destinations?
        current_step.exits = get_string("exits", curret_json_step);
        current_step.mode = get_string("mode", curret_json_step);
        if(curret_json_step.values.find("maneuver") != curret_json_step.values.end())
        {
            auto maneuver = curret_json_step.values["maneuver"].get<json::Object>();
            parse_maneuver(current_step, maneuver);
        }
        if(curret_json_step.values.find("intersections") != curret_json_step.values.end())
        {
            auto intersections = curret_json_step.values["intersections"].get<json::Array>();
            parse_intersections(current_step, intersections);
        }
        current_step.rotary_name = get_string("rotary_name", curret_json_step);
        current_step.rotary_pronunciation = get_string("rotary_pronunciation", curret_json_step);
        current_step.driving_side = get_string("driving_side", curret_json_step);
    }
}

void destroy_steps(osrm_step_t  *steps, int number_of_steps)
{
    if(steps == NULL)
    {
        return;
    } 

    for(int i = 0; i < number_of_steps; i++)
    {
        if(steps[i].geometry != NULL)
        {
            free(steps[i].geometry);
        }
        if(steps[i].name != NULL)
        {
            free(steps[i].name);
        }
        if(steps[i].ref != NULL)
        {
            free(steps[i].ref);
        }
        if(steps[i].pronunciation != NULL)
        {
            free(steps[i].pronunciation);
        }
        if(steps[i].exits != NULL)
        {
            free(steps[i].exits);
        }
        if(steps[i].mode != NULL)
        {
            free(steps[i].mode);
        }
        if(steps[i].rotary_name != NULL)
        {
            free(steps[i].rotary_name);
        }
        if(steps[i].rotary_pronunciation != NULL)
        {
            free(steps[i].rotary_pronunciation);
        }
        if(steps[i].driving_side != NULL)
        {
            free(steps[i].driving_side);
        }
        if(steps[i].maneuver != NULL)
        {
            destroy_maneuver(steps[i].maneuver);
        }
        if(steps[i].intersections != NULL)
        {
            destroy_intersections(steps[i].intersections, steps[i].number_of_intersections);
        }
    }


    free(steps);
}

void parse_maneuver(osrm_step_t &step, json::Object &maneuver)
{
    step.maneuver = static_cast<osrm_maneuver_t  *>(malloc(sizeof(osrm_maneuver_t)));
    *step.maneuver = osrm_maneuver_default;

    if(maneuver.values.find("bearing_before") != maneuver.values.end())
    {
        step.maneuver->bearing_before = maneuver.values["bearing_before"].get<json::Number>().value;
    }
    if(maneuver.values.find("bearing_after") != maneuver.values.end())
    {
        step.maneuver->bearing_after = maneuver.values["bearing_after"].get<json::Number>().value;
    }
    if(maneuver.values.find("location") != maneuver.values.end())
    {
        auto location_array = maneuver.values["location"].get<json::Array>().values;
        step.maneuver->location.longitude = location_array[0].get<json::Number>().value;
        step.maneuver->location.latitude = location_array[1].get<json::Number>().value;
    }
    step.maneuver->type = get_string("type", maneuver);
    step.maneuver->modifer = get_string("modifer", maneuver);
}

void destroy_maneuver(osrm_maneuver_t *maneuver)
{
    if(maneuver == NULL)
    {
        return;
    } 

    if(maneuver->type != NULL)
    {
        free(maneuver->type);
    }
    if(maneuver->modifer != NULL)
    {
        free(maneuver->modifer);
    }

    free(maneuver);
}

void parse_intersections(osrm_step_t &step, json::Array &intersections)
{
    step.intersections = static_cast<osrm_intersections_t  *>(malloc(sizeof(osrm_intersections_t) * intersections.values.size()));
    step.number_of_intersections = intersections.values.size();
    for(int i = 0; i < intersections.values.size(); i++)
    {
        auto intersection = intersections.values[i].get<json::Object>();
        step.intersections[i] = osrm_intersections_default;

        if(intersection.values.find("location") != intersection.values.end())
        {
            auto location = intersection.values["location"].get<json::Array>().values;
            step.intersections[i].location.longitude = location[0].get<json::Number>().value;
            step.intersections[i].location.latitude = location[1].get<json::Number>().value;
        }
        if(intersection.values.find("in") != intersection.values.end())
        {
            step.intersections[i].in = intersection.values["in"].get<json::Number>().value;
        }
        if(intersection.values.find("out") != intersection.values.end())
        {
            step.intersections[i].out = intersection.values["out"].get<json::Number>().value;
        }
        if(intersection.values.find("bearings") != intersection.values.end())
        {
            auto bearings = intersection.values["bearings"].get<json::Array>();
            step.intersections[i].bearings = static_cast<int  *>(malloc(sizeof(int) * bearings.values.size()));
            step.intersections[i].number_of_bearings = bearings.values.size();
            for(int j = 0; j < bearings.values.size(); j++)
            {
                step.intersections[i].bearings[j] = bearings.values[j].get<json::Number>().value;
            }
        }
        if(intersection.values.find("classes") != intersection.values.end())
        {
            auto classes = intersection.values["classes"].get<json::Array>();
            step.intersections[i].classes = static_cast<char  **>(malloc(sizeof(char*) * classes.values.size()));
            step.intersections[i].number_of_classes = classes.values.size();
            for(int j = 0; j < classes.values.size(); j++)
            {
                step.intersections[i].classes[j] = get_string_from_string(classes.values[j].get<json::String>());
            }
        }
        if(intersection.values.find("lanes") != intersection.values.end())
        {
            auto lanes = intersection.values["lanes"].get<json::Array>();
            parse_lanes(step.intersections[i], lanes);
        }
    }
}

void destroy_intersections(osrm_intersections_t *intersections, int number_of_intersections)
{
    if(intersections == NULL)
    {
        return;
    }

    for(int i = 0; i < number_of_intersections; i++)
    {
        if(intersections[i].classes != NULL)
        {
            for(int j = 0; j < intersections[i].number_of_classes; j++)
            {
                if(intersections[i].classes[j] != NULL)
                {
                    free(intersections[i].classes[j]);
                }
            }
            free(intersections[i].classes);
        }
        if(intersections[i].bearings != NULL)
        {
            free(intersections[i].bearings);
        }

        destroy_lanes(intersections[i].lanes, intersections[i].number_of_lanes);
    }

    free(intersections);
}

void parse_lanes(osrm_intersections_t &intersections, json::Array &lanes)
{
    intersections.lanes = static_cast<osrm_lane_t  *>(malloc(sizeof(osrm_lane_t) * lanes.values.size()));
    intersections.number_of_lanes = lanes.values.size();
    for(int i = 0; i < lanes.values.size(); i++)
    {
        auto lane = lanes.values[i].get<json::Object>();
        intersections.lanes[i] = osrm_lane_default;

        if(lane.values.find("indications") != lane.values.end())
        {
            auto indications = lane.values["indications"].get<json::Array>();
            intersections.lanes[i].indications = static_cast<char **>(malloc(sizeof(char *) * indications.values.size()));
            for(int j = 0; j < indications.values.size(); j++)
            {
                intersections.lanes[i].indications[j] = get_string_from_string(indications.values[j].get<json::String>());
            }
        }
        if(lane.values.find("valid") != lane.values.end())
        {
            if(lane.values["valid"].is<json::True>())
            {
                intersections.lanes[i].valid = boolean::TRUE;
            } 
        }
    }
}

void destroy_lanes(osrm_lane_t  *lanes, int number_of_lanes)
{
    if(lanes == NULL)
    {
        return;
    }

    for(int i = 0; i < number_of_lanes; i++)
    {
        if(lanes[i].indications != NULL)
        {
            free(lanes[i].indications);
        }
    }

    free(lanes);
}

char* get_string(std::string key, json::Object &json)
{
    char* return_value;
    if(json.values.find(key) != json.values.end())
    {
        const auto value = json.values[key].get<json::String>().value;
        return_value = (char*)malloc(sizeof(char) * (value.size() + 1));
        value.copy(return_value, value.size() + 1);
        return_value[value.size()] = '\0';
    }

    return return_value;
}

char* get_string_from_string(json::String &value)
{
    char* return_value = (char*)malloc(sizeof(char) * (value.value.size() + 1));
    value.value.copy(return_value, value.value.size() + 1);
    return_value[value.value.size()] = '\0';

    return return_value;
}

void nearest_result_destroy(nearest_result_t *value)
{
    if(value == nullptr)
    {
        return;
    }

    if(value->code != nullptr)
    {
        free(value->code);
    }

    if(value->message != nullptr)
    {
        free(value->message);
    }

    if(value->waypoints != nullptr)
    {
        for(int i = 0; i < value->number_of_waypoints; i++)
        {
            free(value->waypoints[i].hint);
            free(value->waypoints[i].name);
        }

        free(value->waypoints);
    }

    free(value);
}

void table_result_destroy(table_result_t *value)
{
    if(value == NULL)
    {
        return;
    }

    if(value->code != NULL)
    {
        free(value->code);
    }

    if(value->message != NULL)
    {
        free(value->message);
    }

    if(value->durations != NULL)
    {
        free(value->durations);
    }
    
    if(value->sources != NULL)
    {
        for(int i = 0; i < value->number_of_sources; i++)
        {
            free(value->sources[i].hint);
            free(value->sources[i].name);
        }

        free(value->sources);
    }

    if(value->destinations != NULL)
    {
        for(int i = 0; i < value->number_of_destinations; i++)
        {
            free(value->destinations[i].hint);
            free(value->destinations[i].name);
        }

        free(value->destinations);
    }

    free(value);
}

void route_result_destroy(route_result_t *value)
{
    if(value == NULL)
    {
        return;
    }


    if(value->code != NULL)
    {
        free(value->code);
    }
    if(value->message != NULL)
    {
        free(value->message);
    }
    if(value->waypoints != NULL)
    {
        for(int i = 0; i < value->number_of_waypoints; i++)
        {
            if(value->waypoints[i].hint != NULL)
            {
                free(value->waypoints[i].hint);
            }
            if(value->waypoints[i].name != NULL)
            {
                free(value->waypoints[i].name);
            }
        }

        free(value->waypoints);
    }
    if(value->routes != NULL)
    {
        destroy_route(value->routes, value->number_of_routes);
    }

    free(value);
}