#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "com.h"

static int fd;
static uint32_t COM_Baudrate = 115200;

#ifdef _WIN32
#include <windows.h>
#include <winbase.h>

static HANDLE hSerial;

/** \brief Configure a serial port
 *
 * \param [in] param Desired serial port parameters
 * \return true if succeed
 *
 */
bool COM_Config(const struct com_params *params)
{
  DCB dcbSerialParams = { 0 }; // Initializing DCB structure
  DWORD wait_events = EV_TXEMPTY;

  /* Wait for output to drain */
  WaitCommEvent(hSerial, &wait_events, NULL);

  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  GetCommState(hSerial, &dcbSerialParams);
  dcbSerialParams.BaudRate        = params->baudrate;  // Setting BaudRate
  dcbSerialParams.fBinary         = TRUE;
  dcbSerialParams.fParity         = params->have_parity;
  dcbSerialParams.fOutxCtsFlow    = FALSE;
  dcbSerialParams.fOutxDsrFlow    = FALSE;
  dcbSerialParams.fDtrControl     =
    params->dtr ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;
  dcbSerialParams.fDsrSensitivity = FALSE;
  dcbSerialParams.fOutX           = FALSE;
  dcbSerialParams.fInX            = FALSE;
  dcbSerialParams.fErrorChar      = TRUE;
  dcbSerialParams.fNull		  = FALSE;
  dcbSerialParams.fRtsControl     =
    params->rts ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;
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

/** \brief Configure a serial port
 *
 * \param [in] param Desired serial port parameters
 * \return true if succeed
 *
 */
bool COM_Config(const struct com_params *params)
{
  struct termios SerialPortSettings;
  /* Get the current attributes of the Serial port */
  if (tcgetattr(fd, &SerialPortSettings))
    return false;
  /* Setting the Baud rate */
  if (cfsetbaud(&SerialPortSettings, params->baudrate))
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
  if (params->have_parity)
    SerialPortSettings.c_cflag |= PARENB;   /* Enable parity */
  if (params->two_stopbits)
    SerialPortSettings.c_cflag |= CSTOPB;   /* CSTOPB = 2 Stop bits */

  SerialPortSettings.c_cc[VMIN]  = 0;	// read doesn't block
  SerialPortSettings.c_cc[VTIME] = 1;	// 0.1 seconds read timeout
  tcsetattr(fd, TCSAFLUSH, &SerialPortSettings);  /* Set the attributes to the termios structure*/
  set_modem_bits(fd, params->dtr, params->rts);

  return true;
}

#endif /* __unix__ */

/** \brief Open COM port with settings
 *
 * \param [in] params Port name and settings
 * \return true if succeed
 *
 */
bool COM_Open(const struct com_params *params)
{
  printf("Opening %s at %u baud\n", params->port, params->baudrate);
#ifdef _WIN32
  size_t port_len = strlen(params->port);
  char *str = malloc(port_len + 5);

  if (!str)
    return false;

  /* Add \\.\ device name prefix */
  memcpy(str, "\\\\.\\", 4);
  memcpy(str+4, params->port, port_len+1);
  hSerial = CreateFile(str, GENERIC_READ | GENERIC_WRITE, 0,
		       NULL, OPEN_EXISTING, 0, NULL);
  free(str);
  if (hSerial == INVALID_HANDLE_VALUE)
    return false;
  fd = _open_osfhandle((intptr_t)hSerial, 0);
#else
  fd = open(params->port, O_RDWR | O_NOCTTY | O_CLOEXEC);
#endif
  if (fd < 0)
    return false;
  return COM_Config(params);
}

/** \brief Write data to COM port
 *
 * \param [in] data Data buffer for writing
 * \param [in] len Length of data buffer
 * \return 0 if everything Ok
 *
 */
int COM_Write(const uint8_t *data, uint16_t len)
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
