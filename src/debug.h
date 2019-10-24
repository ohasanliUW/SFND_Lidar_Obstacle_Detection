
#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <cstdio>

#define LOG(format, ...) \
    fprintf(stdout, format "\n", ## __VA_ARGS__)
#define ERROR(format, ...) \
    fprintf(stderr, format "\n", ## __VA_ARGS__)

#endif
