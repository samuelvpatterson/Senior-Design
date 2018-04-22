#include "fixed_point.h"
#include "linear_case.h"

/*
 * In the event of strictly linear, we can directly calculate the necessary 
 * DAC_Out value since we know the range and resolution.
 * USING JULIAN'S LIBRARY
 */
uint32_t Linear_Find_Dac_Out(float dac_max_v, float dac_min_v, float target_v, uint32_t dac_num_bits)
{
	uint32_t _dac_max_v = convert_float_to_fp(dac_max_v);
	uint32_t _dac_min_v = convert_float_to_fp(dac_min_v);
	uint32_t _target_v = convert_float_to_fp(target_v);
	dac_num_bits = convert_int_to_fp(1 << dac_num_bits);
	
	uint32_t resolution = div_fp(sub_fp(_dac_max_v, _dac_min_v), dac_num_bits);
	return convert_fp_to_int_rn(div_fp(sub_fp(_target_v, _dac_min_v), resolution));
}

/*
 * Binary search for n bit DAC if monotonically increasing
 */

// uint32_t BS_Find_Dac_Out(uint32_t *buf, uint32_t low, uint32_t high, uint32_t target)
// {
// 	uint32_t mid, voltage;

// 	if (low >= high)
// 	{
// 		mid = (high - low) / 2;

// 		DAC_Out(buf[mid]);
// 		voltage = ADC_In();

// 		if (abs(target - voltage) <= error)
// 			return buf[mid];

// 		if (target > buf[mid])
// 			return BS_Find_Dac_Out(buf, mid, high, target);
// 		else
// 			return BS_Find_Dac_Out(buf, low, mid, target);
// 	}
// 	return -1;
// }
