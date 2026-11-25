/*
 * hal_direct.c -- ISP direct mode (IVDC) as the ISP driver was loaded
 */

#include <stdio.h>
#include <stdlib.h>

#include "hal_direct.h"

int hal_direct_sensors_from_file(const char *path, int *count)
{
    if (!count)
        return RSS_ERR_INVAL;

    FILE *f = fopen(path, "r");
    if (!f)
        return RSS_ERR_NOTSUP;

    char buf[16] = "";
    if (!fgets(buf, sizeof(buf), f))
        buf[0] = '\0';
    fclose(f);

    char *end;
    long v = strtol(buf, &end, 10);
    if (end == buf || (*end != '\0' && *end != '\n') || v < 0)
        return RSS_ERR_IO;

    *count = (int)v;
    return RSS_OK;
}
