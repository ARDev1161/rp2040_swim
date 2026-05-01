#include "target_db.h"

#include <string.h>

static const stm8_target_t g_targets[] = {
    {
        .name = "stm8s003f3",
        .flash_start = 0x008000u,
        .flash_size = 8192u,
        .ram_start = 0x000000u,
        .ram_size = 1024u,
        .eeprom_start = 0x004000u,
        .eeprom_size = 128u,
        .option_start = 0x004800u,
        .option_size = 64u,
        .block_size = 64u,
    },
    {
        .name = "stm8s103f3",
        .flash_start = 0x008000u,
        .flash_size = 8192u,
        .ram_start = 0x000000u,
        .ram_size = 1024u,
        .eeprom_start = 0x004000u,
        .eeprom_size = 640u,
        .option_start = 0x004800u,
        .option_size = 64u,
        .block_size = 64u,
    },
};

const stm8_target_t *target_db_find(const char *name) {
    for (size_t i = 0; i < sizeof(g_targets) / sizeof(g_targets[0]); i++) {
        if (strcmp(g_targets[i].name, name) == 0) {
            return &g_targets[i];
        }
    }
    return NULL;
}

const stm8_target_t *target_db_all(size_t *count) {
    *count = sizeof(g_targets) / sizeof(g_targets[0]);
    return g_targets;
}
