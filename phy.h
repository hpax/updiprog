#ifndef PHY_H
#define PHY_H

#include <stdint.h>
#include <stdbool.h>

#define PHY_BAUDRATE      (115200)

struct com_params;

bool PHY_Init(const struct com_params *com);
bool PHY_DoBreak(const struct com_params *com);
bool PHY_Send(uint8_t *data, uint8_t len);
bool PHY_Receive(uint8_t *data, uint16_t len);
void PHY_Close(void);

#endif
