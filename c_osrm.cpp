#include "c_osrm.h"
#include "osrm/engine_config.hpp"
#include "osrm/osrm.hpp"
#include "osrm/nearest_parameters.hpp"

#include <iostream>


using namespace osrm;

struct c_osrm {
    void *obj;
};


c_osrm_t *osrm_create(engine_config_t *config)
{
    c_osrm_t *osrm;
    OSRM *osrm_osrm;

    osrm = (typeof(osrm))malloc(sizeof(*osrm));
    osrm_osrm = new OSRM(*static_cast<EngineConfig*>(config->obj));
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

enum status osrm_nearest(c_osrm_t *c_osrm, double lat, double lon, unsigned number_of_results, nearest_result_t** result)
{
    OSRM *osrm =static_cast<OSRM*>(c_osrm->obj);

    NearestParameters parameters;
    parameters.coordinates.emplace_back(util::FloatLongitude{lon}, util::FloatLatitude{lat});
    parameters.number_of_results = number_of_results;

    engine::api::ResultT osr_result = json::Object();

    const auto status = osrm->Nearest(parameters, osr_result);

    auto &json_result = osr_result.get<json::Object>();

    if(*result != nullptr)
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
        const auto waypoints = json_result.values["waypoints"].get<json::Array>().values;

        if(waypoints.empty())
        {
            return status::Ok;
        }

        return_result->waypoints = static_cast<waypoint_t *>(malloc(sizeof(waypoint_t) * waypoints.size()));
        return_result->number_of_waypoints = waypoints.size();
        printf("5");
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


