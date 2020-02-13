#include "c_osrm.h"
#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/table_parameters.hpp"

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

        }

        *result = return_result;

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

enum status osrm_table(c_osrm_t *c_osrm, table_request_t* request, table_result_t** result)
{
    OSRM *osrm =static_cast<OSRM*>(c_osrm->obj);

    TableParameters parameters;

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

    const auto status = osrm->Table(parameters, osr_result);

    auto &json_result = osr_result.get<json::Object>();

    if(*result != NULL)
    {
        free(result);
    }

    table_result_t *return_result = NULL;
    return_result = (typeof(return_result))malloc(sizeof(*return_result));

    const auto code = json_result.values["code"].get<json::String>().value;
    return_result->code = (char*)malloc(sizeof(char) * (code.size() + 1));
    code.copy(return_result->code, code.size() + 1);
    return_result->code[code.size()] = '\0';

    if (status == Status::Ok)
    {
        return_result->message = NULL;

        const auto sources = json_result.values["sources"].get<json::Array>().values;
        const auto destinations = json_result.values["destinations"].get<json::Array>().values;
        const auto durations = json_result.values["durations"].get<json::Array>().values;

        return_result->sources = static_cast<waypoint_t  *>(malloc(sizeof(waypoint_t) * sources.size()));
        return_result->number_of_sources = sources.size();

        for(int i = 0; i < sources.size(); i++)
        {
            auto source = sources[i].get<json::Object>();
            const auto name = source.values["name"].get<json::String>().value;
            const auto hint = source.values["hint"].get<json::String>().value;
            const auto distance = source.values["distance"].get<json::Number>().value;
            auto location = source.values["location"].get<json::Array>().values;

            return_result->sources[i].hint= (char*)malloc(sizeof(char) * (hint.size() + 1));
            hint.copy(return_result->sources[i].hint, hint.size() + 1);
            return_result->sources[i].hint[hint.size()] = '\0';

            return_result->sources[i].distance = distance;

            return_result->sources[i].name= (char*)malloc(sizeof(char) * (name.size() + 1));
            name.copy(return_result->sources[i].name, name.size() + 1);
            return_result->sources[i].name[name.size()] = '\0';

            return_result->sources[i].location[0] = location[0].get<json::Number>().value;
            return_result->sources[i].location[1] = location[1].get<json::Number>().value;

        }

        return_result->destinations = static_cast<waypoint_t  *>(malloc(sizeof(waypoint_t) * destinations.size()));
        return_result->number_of_destinations = destinations.size();

        for(int i = 0; i < destinations.size(); i++)
        {
            auto destination = destinations[i].get<json::Object>();
            const auto name = destination.values["name"].get<json::String>().value;
            const auto hint = destination.values["hint"].get<json::String>().value;
            const auto distance = destination.values["distance"].get<json::Number>().value;
            auto location = destination.values["location"].get<json::Array>().values;

            return_result->destinations[i].hint= (char*)malloc(sizeof(char) * (hint.size() + 1));
            hint.copy(return_result->destinations[i].hint, hint.size() + 1);
            return_result->destinations[i].hint[hint.size()] = '\0';

            return_result->destinations[i].distance = distance;

            return_result->destinations[i].name= (char*)malloc(sizeof(char) * (name.size() + 1));
            name.copy(return_result->destinations[i].name, name.size() + 1);
            return_result->destinations[i].name[name.size()] = '\0';

            return_result->destinations[i].location[0] = location[0].get<json::Number>().value;
            return_result->destinations[i].location[1] = location[1].get<json::Number>().value;

        }

        return_result->durations = static_cast<double**>(malloc(sizeof(double) * sources.size()));
        for(int i = 0; i < sources.size(); i++)
        {
            const auto durations_element = durations[i].get<json::Array>().values;

            return_result->durations[i] = static_cast<double*>(malloc(sizeof(double) * durations_element.size()));

            for(int j = 0; j < durations_element.size(); j++)
            {
                return_result->durations[i][j] = durations_element[j].get<json::Number>().value;
            }
        }

        *result = return_result;

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

    return status::Error;
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
        for(int i = 0; i < value->number_of_sources; i++)
        {
            free(value->durations[i]);
        }

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

        free(value->sources);
    }

    free(value);
}