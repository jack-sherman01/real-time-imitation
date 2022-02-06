#ifndef _CMPRIKITF_H_
#define _CMPRIKITF_H_


/** EXTERN LIB SECTION BEGIN **/
#ifdef __cplusplus
extern "C" {
#endif
/**
 * <description>get_args</description>
 */
typedef struct tagget_args_struct
{
	int32_t *mode;					/* VAR_INPUT */	
	int32_t *cmd_word;				/* VAR_INPUT */	
	int32_t *target_position;		/* VAR_INPUT */	
	int32_t *target_speed;			/* VAR_INPUT */	
	int32_t *target_torque;		/* VAR_INPUT */	
	int16_t get_args;				/* VAR_OUTPUT */	
} get_args_struct;

void get_args(get_args_struct *p);

/**
 * <description>set_args</description>
 */
typedef struct tagset_args_struct
{
	int32_t *mode_dispaly;			/* VAR_INPUT */	
	int32_t *status;				/* VAR_INPUT */	
	int32_t *actual_position;		/* VAR_INPUT */	
	int32_t *actual_speed;			/* VAR_INPUT */	
	int32_t *actual_torque;		    /* VAR_INPUT */	
	int16_t set_args;				/* VAR_OUTPUT */	
} set_args_struct;

void set_args(set_args_struct *p);

#ifdef __cplusplus
}
#endif

/** EXTERN LIB SECTION END**/

#endif