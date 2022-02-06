#ifndef _CMPRIKAPI_H_
#define _CMPRIKAPI_H_

/**
 * <note> data struct by api </note>
 * <author> tangliang </author>
*/

/** EXTERN LIB SECTION BEGIN **/
#ifdef __cplusplus
extern "C" {
#endif
/**
 * <description>get_apiArgs</description>
 */
#include <stdint.h>
typedef struct tagget_apiArgs_struct
{
	float *target_joint;		    /* VAR_INPUT */	
	float *target_speed;			/* VAR_INPUT */	
	float *target_torque;		    /* VAR_INPUT */	
	float get_apiArgs;				/* VAR_OUTPUT */	
} get_apiArgs_struct;

void get_apiArgs(get_apiArgs_struct *p);

/**
 * <description>set_apiArgs</description>
 */
typedef struct tagset_apiArgs_struct
{
	float *actual_joint;		    /* VAR_INPUT */	
	float *actual_speed;			/* VAR_INPUT */	
	float *actual_torque;		    /* VAR_INPUT */	
	float set_apiArgs;				/* VAR_OUTPUT */	
} set_apiArgs_struct;

void set_apiArgs(set_apiArgs_struct *p);

#ifdef __cplusplus
}
#endif

/** EXTERN LIB SECTION END**/

#endif