#ifndef COM_H
#define COM_H

#include <stdint.h>
#include <stdbool.h>

struct com_params {
    const char *port;
    unsigned long baudrate;
    bool have_parity;
    bool two_stopbits;
    bool rts;
    bool dtr;
};

bool COM_Open(const struct com_params *com);
bool COM_Config(const struct com_params *com);
int COM_Write(const uint8_t *data, uint16_t len);
int COM_Read(uint8_t *data, uint16_t len);
uint16_t COM_GetTransTime(uint16_t len);
void COM_Close(void);

#endif
