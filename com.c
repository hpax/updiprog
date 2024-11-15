#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "com.h"

static int fd;
static uint32_t COM_Baudrate = 115200;

#ifdef _WIN32
#include <windows.h>
#include <winbase.h>

/** \brief Open COM port with settings
 *
 * \param [in] port Port name as string
 * \param [in] baudrate Port baudrate
 * \param [in] have_parity true if parity should be switched on
 * \param [in] two_stopbits true if 2 stop bits used
 * \return true if succeed
 *
 */
bool COM_Open(char *port, uint32_t baudrate, bool have_parity, bool two_stopbits)
{
  printf("Opening %s at %u baud\n", port, baudrate);
  char str[64];
  HANDLE hSerial;

  snprintf(str, sizeof str, "\\\\.\\%s", port);
  hSerial = CreateFile(str, GENERIC_READ | GENERIC_WRITE, 0,
                              NULL, OPEN_EXISTING, 0, NULL);
  if (hSerial == INVALID_HANDLE_VALUE)
    return false;
  fd = _open_osfhandle((intptr_t)hSerial, 0);
  if (fd < 0)
      return false;

  DCB dcbSerialParams = { 0 }; // Initializing DCB structure
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  GetCommState(hSerial, &dcbSerialParams);
  dcbSerialParams.BaudRate        = baudrate;  // Setting BaudRate
  dcbSerialParams.fBinary         = TRUE;
  dcbSerialParams.fParity         = have_parity;
  dcbSerialParams.fOutxCtsFlow    = FALSE;
  dcbSerialParams.fOutxDsrFlow    = FALSE;
  dcbSerialParams.fDtrControl     = DTR_CONTROL_DISABLE;
  dcbSerialParams.fDsrSensitivity = FALSE;
  dcbSerialParams.fOutX           = FALSE;
  dcbSerialParams.fInX            = FALSE;
  dcbSerialParams.fErrorChar      = TRUE;
  dcbSerialParams.fNull		  = FALSE;
  dcbSerialParams.fRtsControl     = RTS_CONTROL_DISABLE;
  dcbSerialParams.fAbortOnError   = FALSE;
  dcbSerialParams.ByteSize        = 8;
  dcbSerialParams.Parity          = have_parity ? EVENPARITY : NOPARITY;
  dcbSerialParams.StopBits        = twostopbits ? TWOSTOPBITS : ONESTOPBIT;
  dcbSerialParams.ErrorChar       = 0x00;
  SetCommState(hSerial, &dcbSerialParams);
  COMMTIMEOUTS timeouts;
  DWORD multiplier = (100000 + baudrate - 1)/baudrate;
  timeouts.ReadIntervalTimeout = 20 * multiplier;
  timeouts.ReadTotalTimeoutMultiplier = 1 * multiplier;
  timeouts.ReadTotalTimeoutConstant = 100 * multiplier;
  timeouts.WriteTotalTimeoutMultiplier = 1;
  timeouts.WriteTotalTimeoutConstant = 1;
  SetCommTimeouts(hSerial, &timeouts);
  //COM_Bytes = 0;
  return true;
}

#elif defined(__unix__)

#define _BSD_SOURCE		/* For CRTSCTS */
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#ifdef TCSETS2
#include <asm/termbits.h>
#else
#include <termios.h>
#endif

#ifndef BOTHER
#define BOTHER B0		/* Non-fixed baud rates not supported */
#endif

/* If these are not supported on this platform, just skip them */
#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif
#ifndef LOBLK
#define LOBLK 0
#endif
#ifndef CRTSCTS
#define CRTSCTS 0
#endif

static speed_t termios_convert_speed(uint32_t baudrate)
{
    speed_t speed;

    /*
     * Assume termios is sane if and only if all baud rate constants
     * including B0 match their numeric values.
     */
    const bool termios_is_sane = true
#define SPEED(b,n) && (b == n)
#include "termios_speeds.h"
#undef SPEED
	;

    if (termios_is_sane) {
	speed = baudrate;
    } else {
	switch (baudrate) {
#define SPEED(b,n) case n : speed = b; break;
#include "termios_speeds.h"
#undef SPEED
	default : speed = BOTHER; break;
	}
    }
    return speed;
}

#ifdef TCSETS2
/*
 *
 * Note that <asm/termbits.h> conflicts with <termios.h>, so both
 * cannot be included at the same time. Thus, this is basically a
 * "mini-implementation" of termios.
 */

#define termios termios2
static int tcgetattr(int fd, struct termios *termios_p)
{
    return ioctl(fd, TCGETS2, termios_p);
}
#undef  TCSANOW
#undef  TCSADRAIN
#undef  TCSAFLUSH
#define TCSANOW		TCSETS2
#define TCSADRAIN	TCSETSW2
#define TCSAFLUSH	TCSETSF2
static int tcsetattr(int fd, int optional_actions,
		     const struct termios *termios_p)
{
    return ioctl(fd, optional_actions, termios_p);
}
static int cfsetbaud(struct termios *termios_p, unsigned long baudrate)
{
    speed_t speed = termios_convert_speed(baudrate);
    termios_p->c_ispeed = termios_p->c_ospeed = baudrate;
    if (speed == B0)
	speed = BOTHER;
    termios_p->c_cflag &= CBAUD | CIBAUD;
    termios_p->c_cflag |= speed | (speed << IBSHIFT);
    return 0;
}
static int tcflush(int fd, int arg)
{
    return ioctl(fd, TCFLSH, arg);
}
#else
static int cfsetbaud(struct termios *termios_p, unsigned long baudrate)
{
    speed_t speed = termios_convert_speed(baudrate);
    return baudrate != B0 &&
	cfsetispeed(termios_p, baudrate) &&
	cfsetospeed(termios_p, baudrate);
}
#endif

#ifdef TIOCMSET
static int set_modem_bits(int fd, bool dtr, bool rts)
{
    const int modem_bits =
	(dtr ? TIOCM_DTR : 0) |
	(rts ? TIOCM_RTS : 0);
    return ioctl(fd, TIOCMSET, &modem_bits);
}
#else
static int set_modem_bits(int fd, bool dtr, bool rts)
{
    (void)fd;
    (void)dtr;
    (void)rts;
    errno = ENOSYS;
    return -1;			/* Don't know how to set modem flags */
}
#endif

/** \brief Open COM port with settings
 *
 * \param [in] port Port name as string
 * \param [in] baudrate Port baudrate
 * \param [in] have_parity true if parity should be switched on
 * \param [in] two_stopbits true if 2 stop bits used
 * \return true if succeed
 *
 */
bool COM_Open(char *port, uint32_t baudrate, bool have_parity, bool two_stopbits)
{
  printf("Opening %s at %u baud\n", port, baudrate);
  fd = open(port, O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd <0)
    return false;
  struct termios SerialPortSettings;
  /* Get the current attributes of the Serial port */
  if (tcgetattr(fd, &SerialPortSettings))
      return false;
  /* Setting the Baud rate */
  if (cfsetbaud(&SerialPortSettings, baudrate))
      return false;

  /*
   * Disable output processing
   */
  SerialPortSettings.c_oflag &= ~OPOST;

  /*
   * Disable echo and other line processing
   */
  SerialPortSettings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  /*
   * Disable input processing
   */
  SerialPortSettings.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | PARMRK |
				  ISTRIP | INLCR | IGNCR | ICRNL |
				  IXON | IXOFF);
  /*
   * Disable parity, 1 stop bit, enable receiver, ignore modem signals,
   * disable RTS/CTS signalling, disable XON/XOFF, disable parity checking
   * and all character conversions.
   */
  SerialPortSettings.c_cflag &= ~(CSIZE | HUPCL | PARENB | CSTOPB | CRTSCTS);
  SerialPortSettings.c_cflag |= CREAD | CLOCAL | CS8;
  if (have_parity)
    SerialPortSettings.c_cflag |= PARENB;   /* Enable parity */
  if (two_stopbits)
    SerialPortSettings.c_cflag |= CSTOPB;   /* CSTOPB = 2 Stop bits */

  SerialPortSettings.c_cc[VMIN]  = 0;	// read doesn't block
  SerialPortSettings.c_cc[VTIME] = 1;	// 0.1 seconds read timeout
  tcsetattr(fd, TCSANOW, &SerialPortSettings);  /* Set the attributes to the termios structure*/

  tcflush(fd, TCIFLUSH);
  set_modem_bits(fd, false, false);

  return true;
}

#endif /* __unix__ */

/** \brief Write data to COM port
 *
 * \param [in] data Data buffer for writing
 * \param [in] len Length of data buffer
 * \return 0 if everything Ok
 *
 */
int COM_Write(uint8_t *data, uint16_t len)
{
    return write(fd, data, len) >= 0;
}

/** \brief Read data from COM port
 *
 * \param [out] data Data buffer to read data in
 * \param [in] len Length of data to read
 * \return number of received bytes as int, -1 on error
 *
 */
int COM_Read(uint8_t *data, uint16_t len)
{
    return read(fd, data, len);
}

/** \brief Calculate time for transmission with current baudrate
 *
 * \param [in] len Length of transmitted data
 * \return time in milliseconds as uint16_t
 *
 */
uint16_t COM_GetTransTime(uint16_t len)
{
    return (uint16_t)(len * 1000 * 11 / COM_Baudrate + 1);
}

/** \brief Close current COM port
 *
 * \return Nothing
 *
 */
void COM_Close(void)
{
  printf("Closing COM port\n");
  close(fd);
}
