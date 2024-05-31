
#ifndef __RELAY_PERIPHERAL_H__
#define __RELAY_PERIPHERAL_H__

#include "gr_includes.h"
#include "rscs.h"




sdk_err_t user_relay_peripheral_init(void);

void user_relay_peripheral_hrs_measurement_send(uint16_t heart_rate, bool is_energy_updated);
void user_relay_peripheral_rscs_measurement_send(rscs_meas_val_t *p_meas);
#endif

