//
// Created by tehkoza on 2020-02-05.
//

#ifndef C_OSRM_ENGINE_CONFIG_H
#define C_OSRM_ENGINE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

enum algorithm
{
    CH = 0,
    CoreCH = 1, // Deprecated, will be removed in v6.0
    MLD = 2
};

struct engine_config {
    void *obj;
};
typedef struct engine_config engine_config_t;

engine_config_t *engine_config_create();
void engine_config_destroy(engine_config_t *config);

int engine_config_is_valid(engine_config_t *config);

void set_storage_config(engine_config_t *config, char* path);
char* get_storage_config(engine_config_t *config);

void set_max_locations_trip(engine_config_t *config, int value);
int get_max_locations_trip(engine_config_t *config);

void set_max_locations_viaroute(engine_config_t *config, int value);
int get_max_locations_viaroute(engine_config_t *config);

void set_max_locations_distance_table(engine_config_t *config, int value);
int get_max_locations_distance_table(engine_config_t *config);

void set_max_locations_map_matching(engine_config_t *config, int value);
int get_max_locations_map_matching(engine_config_t *config);

void set_max_radius_map_matching(engine_config_t *config, double value);
double get_max_radius_map_matching(engine_config_t *config);

void set_max_results_nearest(engine_config_t *config, int value);
int get_max_results_nearest(engine_config_t *config);

void set_max_alternatives(engine_config_t *config, int value);
int get_max_alternatives(engine_config_t *config);

void set_use_shared_memory(engine_config_t *config, int value);
int get_use_shared_memory(engine_config_t *config);

void set_memory_file(engine_config_t *config, char* path);
char* get_memory_file(engine_config_t *config);

void set_use_mmap(engine_config_t *config, int value);
int get_use_mmap(engine_config_t *config);

void set_algorithm(engine_config_t *config, enum algorithm value);
enum algorithm get_algorithm(engine_config_t *config);

void set_verbosity(engine_config_t *config, char* path);
char* get_verbosity(engine_config_t *config);

void set_dataset_name(engine_config_t *config, char* path);
char* get_dataset_name(engine_config_t *config);

#ifdef __cplusplus
}
#endif


#endif //C_OSRM_ENGINE_CONFIG_H
