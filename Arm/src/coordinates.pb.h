/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_COORDINATES_PB_H_INCLUDED
#define PB_COORDINATES_PB_H_INCLUDED
#include "utils/pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _Coordinates {
    double x;
    double y;
    double z;
} Coordinates;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define Coordinates_init_default                 {0, 0, 0}
#define Coordinates_init_zero                    {0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define Coordinates_x_tag                        1
#define Coordinates_y_tag                        2
#define Coordinates_z_tag                        3

/* Struct field encoding specification for nanopb */
#define Coordinates_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    x,                 1) \
X(a, STATIC,   SINGULAR, FLOAT,    y,                 2) \
X(a, STATIC,   SINGULAR, FLOAT,    z,                 3)
#define Coordinates_CALLBACK NULL
#define Coordinates_DEFAULT NULL

extern const pb_msgdesc_t Coordinates_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Coordinates_fields &Coordinates_msg

/* Maximum encoded size of messages (where known) */
#define Coordinates_size                         15

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
