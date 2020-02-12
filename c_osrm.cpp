#include "c_osrm.h"
#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"
#include "osrm/nearest_parameters.hpp"

#include <iostream>
#include <engine/approach.hpp>
#include <storage/storage_config.hpp>


using namespace osrm;

struct c_osrm {
    void *obj;
};

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

    printf("%i", request->number_of_results);

    NearestParameters parameters;
    parameters.coordinates.emplace_back(util::FloatLongitude{request->general_options.coordinates.longitude}, util::FloatLatitude{request->general_options.coordinates.latitude});
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

    if(request->general_options.generate_hints == FALSE)
    {
        parameters.generate_hints = false;
    }

    if(request->general_options.hints != NULL)
    {
        parameters.exclude.emplace_back(request->general_options.hints);
    }

    if(request->general_options.approaches != NULL &&
        *request->general_options.approaches == CURB)
    {
        parameters.approaches.emplace_back(engine::Approach::CURB);
    }

    if(request->general_options.exclude != NULL)
    {
//        for(int i = 0; i < request->general_options.number_of_excludes; i++)
//        {
//            parameters.exclude.emplace_back(request->general_options.exclude[i]);
//        }
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

    const auto code = json_result.values["code"].get<json::String>().value;
    return_result->code = (char*)malloc(sizeof(char) * (code.size() + 1));
    code.copy(return_result->code, code.size() + 1);
    return_result->code[code.size()] = '\0';
    if (status == Status::Ok)
    {
        return_result->message = NULL;
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
            const auto name = waypoint.values["name"].get<json::String>().value;
            const auto hint = waypoint.values["hint"].get<json::String>().value;
            const auto distance = waypoint.values["distance"].get<json::Number>().value;
            auto location = waypoint.values["location"].get<json::Array>().values;
            auto nodes = waypoint.values["nodes"].get<json::Array>().values;

            return_result->waypoints[i].nodes[0] = nodes[0].get<json::Number>().value;
            return_result->waypoints[i].nodes[1] = nodes[1].get<json::Number>().value;

            return_result->waypoints[i].hint= (char*)malloc(sizeof(char) * (hint.size() + 1));
            hint.copy(return_result->waypoints[i].hint, hint.size() + 1);
            return_result->waypoints[i].hint[hint.size()] = '\0';

            return_result->waypoints[i].distance = distance;

            return_result->waypoints[i].name= (char*)malloc(sizeof(char) * (name.size() + 1));
            name.copy(return_result->waypoints[i].name, name.size() + 1);
            return_result->waypoints[i].name[name.size()] = '\0';

            return_result->waypoints[i].location[0] = location[0].get<json::Number>().value;
            return_result->waypoints[i].location[1] = location[1].get<json::Number>().value;


            *result = return_result;

        }

        return status::Ok;

    }
    else
    {
        const auto message = json_result.values["message"].get<json::String>().value;

        return_result->message = (char*)malloc(sizeof(char) * (message.size() + 1));
        message.copy(return_result->message, message.size() + 1);
        return_result->message[message.size()] = '\0';

        *result = return_result;

        return status::Error;
    }
}

nearest_request_t* nearest_request_create(double latitude, double longitude)
{
    nearest_request_t* request = static_cast<nearest_request_t *>(malloc(sizeof(nearest_request_t)));
    request->general_options.number_of_coordinates = 1;
    request->general_options.coordinates.latitude = latitude;
    request->general_options.coordinates.longitude = longitude;
    request->general_options.generate_hints = TRUE;
    request->general_options.bearings = NULL;
    request->general_options.hints = NULL;
    request->general_options.exclude = NULL;
    request->general_options.number_of_excludes = 0;
    request->general_options.radiuses = NULL;
    request->general_options.approaches = NULL;
    request->number_of_results = 1;

    return request;
}

void nearest_request_destroy(nearest_request_t *value)
{
    if(value == nullptr)
    {
        return;
    }

    general_options_destroy(value->general_options);

    free(value);
}

void general_options_destroy(general_options_t& value)
{
//    for(int i = 0; i < value.number_of_coordinates; i++)
//    {
//        if(value.bearings != nullptr && value.bearings[i] != nullptr)
//        {
//            free(value.bearings[i]);
//        }
//
//        if(value.hints != nullptr && value.hints[i] != nullptr)
//        {
//            free(value.hints[i]);
//        }
//
//        if(value.coordinates != nullptr && value.coordinates[i] != nullptr)
//        {
//            free(value.coordinates[i]);
//        }
//
//        if(value.radiuses != nullptr && value.radiuses[i] != nullptr)
//        {
//            free(value.radiuses[i]);
//        }
//
//        if(value.approaches != nullptr && value.approaches[i] != nullptr)
//        {
//            free(value.approaches[i]);
//        }
//    }

//    if(value.bearings != nullptr)
//    {
//        free(value.bearings);
//    }
//
//    if(value.hints != nullptr)
//    {
//        free(value.hints);
//    }
////    if(value.coordinates != nullptr)
////    {
////        free(value.coordinates);
////    }
//
//    if(value.radiuses != nullptr)
//    {
//        free(value.radiuses);
//    }
//
//    if(value.approaches != nullptr)
//    {
//        free(value.approaches);
//    }
//
//
//    if(value.exclude != nullptr)
//    {
////        for(int i = 0; i < value.number_of_excludes; i++)
////        {
////            free(value.exclude[i]);
////        }
//        free(value.exclude);
//    }

}

void nearest_result_destroy(nearest_result_t *value)
{
    if(value == nullptr)
    {
        return;
    }

    if(value->message != nullptr)
    {
        free(value->message);
    }

    if(value->code != nullptr)
    {
        free(value->code);
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


