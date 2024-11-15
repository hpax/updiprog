#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "com.h"
#include "log.h"
#include "phy.h"
#include "updi.h"
#include "sleep.h"
#include "com.h"

static struct com_params phy_com;

/** \brief Initialize physical interface
 *
 * \param [in] port Port name as string
 * \param [in] baudrate Transmission baudrate
 * \param [in] onDTR True if using DTR for power
 * \return true if success
 *
 */
bool PHY_Init(const struct com_params *com)
{
  phy_com = *com;
  phy_com.two_stopbits = true;
  phy_com.have_parity  = true;
  return COM_Open(&phy_com);
}

/** \brief Sends a double break to reset the UPDI port
 *         BREAK is actually just a slower zero frame
 *         A double break is guaranteed to push the UPDI state
 *         machine into a known state, albeit rather brutally
 *
 * \param [in] port Port name as string
 * \return true if success
 *
 */
bool PHY_DoBreak(void)
{
  struct com_params break_com;
  uint8_t buf[] = {UPDI_BREAK, UPDI_BREAK};

  break_com = phy_com;
  break_com.baudrate     = 300;
  break_com.two_stopbits = false;
  break_com.have_parity  = false;

  LOG_Print(LOG_LEVEL_INFO, "Sending double break");

  // Re-init at a lower baudrate
  // At 300 bauds, the break character will pull the line low for 30ms
  // Which is slightly above the recommended 24.6ms
  // no parity, one stop bit
  if (!COM_Config(&break_com))
    return false;
  // Send two break characters, with 1 stop bit in between
  COM_Write(buf, sizeof(buf));
  // Wait for the double break end
  msleep(1000);  // wait for 1 second
  if (COM_Read(buf, 2) != 2)
    LOG_Print(LOG_LEVEL_WARNING, "No answer received");

  // Restore previous configuration and exit
  return COM_Config(&phy_com);
}

/** \brief Send data to physical interface
 *
 * \param [in] data Buffer with data
 * \param [in] len Length of data buffer
 * \return true if success
 *
 */
bool PHY_Send(uint8_t *data, uint8_t len)
{
  int rv1, rv2;
  rv1 = COM_Write(data, len);
  if (rv1 < 0)
    return false;
  rv2 = COM_Read(data, rv1);	/* Read echo due to half duplex */
  return rv1 == len && rv2 == len;
}

/** \brief Receive data from physical interface to data buffer
 *
 * \param [out] data Data buffer to write data in
 * \param [in] len Length of data to be received
 * \return true if success
 *
 */
bool PHY_Receive(uint8_t *data, uint16_t len)
{
  memset(data, 0, len);		/* This is not checked in too many places */
  return COM_Read(data, len) == len;
}

/** \brief Close physical interface
 *
 * \return Nothing
 *
 */
void PHY_Close(void)
{
  printf("PHY_Close\n");
  COM_Close();
}
