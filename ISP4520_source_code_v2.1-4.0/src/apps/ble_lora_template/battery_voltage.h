#ifndef BATTERY_VOLTAGE_H__
#define BATTERY_VOLTAGE_H__
 
#include  <stdint.h>
 
/**@brief Function for initializing the battery voltage module.
 */
void battery_voltage_init(void);
 
/**@brief Function for reading the battery voltage.
 *
 * @param[out]   p_vbatt       Pointer to the battery voltage value.
 */
void battery_voltage_get(uint16_t * p_vbatt);
 
#endif // BATTERY_VOLTAGE_H__