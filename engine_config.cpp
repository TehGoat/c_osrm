//
// Created by tehkoza on 2020-02-05.
//

#include "engine_config.h"
#include "osrm/engine_config.hpp"

using namespace osrm;



engine_config_t * engine_config_create()
{
    engine_config_t *config;
    EngineConfig *osrm_config;

    config = (typeof(config))malloc(sizeof(*config));
    osrm_config = new EngineConfig();
    config->obj = osrm_config;

    return config;
}

void engine_config_destroy(engine_config_t *config)
{
    if(config == nullptr)
    {
        return;
    }

    delete static_cast<EngineConfig*>(config->obj);
    free(config);
}

int engine_config_is_valid(engine_config_t *config)
{
    if(config == nullptr)
    {
        return 0;
    }

    return static_cast<EngineConfig*>(config->obj)->IsValid() ? 1 : 0;
}

void set_storage_config(engine_config_t *config, char* path)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->storage_config = storage::StorageConfig(path);
}

char* get_storage_config(engine_config_t *config)
{
    if(config == nullptr)
    {
        return nullptr;
    }

    std::string s =  static_cast<EngineConfig *>(config->obj)->storage_config.base_path.string();

    char *cstr = new char[s.size() + 1];
    s.copy(cstr, s.size() + 1);
    cstr[s.size()] = '\0';
}

void set_max_locations_trip(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->max_locations_trip = value;
}

int get_max_locations_trip(engine_config_t *config)
{
    if(config == nullptr)
    {
        return -1;
    }

    return static_cast<EngineConfig *>(config->obj)->max_locations_trip;
}

void set_max_locations_viaroute(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->max_locations_viaroute = value;
}

int get_max_locations_viaroute(engine_config_t *config)
{
    if(config == nullptr)
    {
        return -1;
    }

    return static_cast<EngineConfig *>(config->obj)->max_locations_viaroute;
}

void set_max_locations_distance_table(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->max_locations_distance_table = value;
}

int get_max_locations_distance_table(engine_config_t *config)
{
    if(config == nullptr)
    {
        return -1;
    }

    return static_cast<EngineConfig *>(config->obj)->max_locations_distance_table;
}

void set_max_locations_map_matching(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->max_locations_map_matching = value;
}

int get_max_locations_map_matching(engine_config_t *config)
{
    if(config == nullptr)
    {
        return -1;
    }

    return static_cast<EngineConfig *>(config->obj)->max_locations_map_matching;
}

void set_max_radius_map_matching(engine_config_t *config, double value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->max_radius_map_matching = value;

}

double get_max_radius_map_matching(engine_config_t *config)
{
    if(config == nullptr)
    {
        return -1.0;
    }

    return static_cast<EngineConfig *>(config->obj)->max_radius_map_matching;

}

void set_max_results_nearest(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->max_results_nearest = value;
}

int get_max_results_nearest(engine_config_t *config)
{
    if(config == nullptr)
    {
        return -1;
    }

    return static_cast<EngineConfig *>(config->obj)->max_results_nearest;
}

void set_max_alternatives(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->max_alternatives = value;

}

int get_max_alternatives(engine_config_t *config)
{
    if(config == nullptr)
    {
        return 3;
    }

    return static_cast<EngineConfig *>(config->obj)->max_alternatives;
}

void set_use_shared_memory(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->use_shared_memory = value != 0;
}

int get_use_shared_memory(engine_config_t *config)
{
    if(config == nullptr)
    {
        return 1;
    }

    return static_cast<EngineConfig *>(config->obj)->use_shared_memory ? 1 : 0;
}

void set_memory_file(engine_config_t *config, char* path)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->memory_file = path;

}

char* get_memory_file(engine_config_t *config)
{
    if(config == nullptr)
    {
        return nullptr;
    }

    std::string s =  static_cast<EngineConfig *>(config->obj)->memory_file.string();

    char *cstr = new char[s.size() + 1];
    s.copy(cstr, s.size() + 1);
    cstr[s.size()] = '\0';

    return cstr;

}

void set_use_mmap(engine_config_t *config, int value)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->use_mmap = value != 0;

}

int get_use_mmap(engine_config_t *config)
{
    if(config == nullptr)
    {
        return 1;
    }

    return static_cast<EngineConfig *>(config->obj)->use_mmap ? 1 : 0;
}

void set_algorithm(engine_config_t *config, algorithm value)
{
    if(config == nullptr)
    {
        return;
    }

    auto *osrm_config = static_cast<EngineConfig *>(config->obj);

    switch (value)
    {

        case algorithm::CH:
            osrm_config->algorithm = EngineConfig::Algorithm::CH;
            break;
        case algorithm::CoreCH:
            osrm_config->algorithm = EngineConfig::Algorithm::CoreCH;
            break;
        case algorithm::MLD:
            osrm_config->algorithm = EngineConfig::Algorithm::MLD;
            break;
    }
}

algorithm get_algorithm(engine_config_t *config)
{
    if(config == nullptr)
    {
        return algorithm::CH;
    }

    switch (static_cast<EngineConfig *>(config->obj)->algorithm)
    {

        case EngineConfig::Algorithm::CH:
            return algorithm::CH;
        case EngineConfig::Algorithm::CoreCH:
            return algorithm::CoreCH;
        case EngineConfig::Algorithm::MLD:
            return algorithm::MLD;
        default:
            return algorithm::CH;
    }
}

void set_verbosity(engine_config_t *config, char* path)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->verbosity = path;
}

char* get_verbosity(engine_config_t *config)
{
    if(config == nullptr)
    {
        return nullptr;
    }

    std::string s = static_cast<EngineConfig *>(config->obj)->verbosity;

    char *cstr = new char[s.size() + 1];
    s.copy(cstr, s.size() + 1);
    cstr[s.size()] = '\0';

    return cstr;
}

void set_dataset_name(engine_config_t *config, char* path)
{
    if(config == nullptr)
    {
        return;
    }

    static_cast<EngineConfig *>(config->obj)->dataset_name = path;
}

char* get_dataset_name(engine_config_t *config)
{
    if(config == nullptr)
    {
        return nullptr;
    }

    std::string s = static_cast<EngineConfig *>(config->obj)->dataset_name;

    char *cstr = new char[s.size() + 1];
    s.copy(cstr, s.size() + 1);
    cstr[s.size()] = '\0';

    return cstr;
}

