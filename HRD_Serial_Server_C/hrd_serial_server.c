//
//  hrd_serial_server.c
//  hrd_serial_server
//
//  Created by Sivon Toledo.
//  Modified by Steve Baker.
//

#include <arpa/inet.h>
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
//#include <netdb.h>
//#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/socket.h>
//#include <sys/stat.h>
#include <sys/time.h>
//#include <sys/types.h>
//#include <sys/utsname.h>
#include <termios.h>
//#include <time.h>
#include <unistd.h>

#include "config.h"
#include "endian.h"
#include "hrd_serial_server.h"

dictionary_t config;
char* config_locations[] = {
  "/etc",
  "./",
  NULL
};

int main(int argc, char **argv)
{
  config =
    configInit("hrd_serial_server.conf",
               config_locations,
               argc,
               argv);

  int port = configGetInt(config, "port", 0, DEFAULT_PORT);
  printf("port %d\n",port);

  // Run the server
  int server_socket = server_socket_create(port);
  while (1)
    start_session(connection_socket_create(server_socket));
}

/********************************************************/
/* time limited I/O                                     */
/********************************************************/

int serialReceiveByteTimelimited(int fd, int msecs)
{
  uint8_t b;

  fd_set fd_read;
  FD_ZERO(&fd_read);
  FD_SET(fd,&fd_read);

  struct timeval timeout;
  timeout.tv_sec=0;
  timeout.tv_usec=1000*msecs;
  
  int res=select(FD_SETSIZE, &fd_read, 0, 0, &timeout);
  if (res == 0) {
    printf("timeout in select (> %dms)\n",msecs);
    errno = ETIMEDOUT;
    return -1;
  }
  if(res<0) {
    return -1;
    //if (errno == EINTR) return -1;
    //printf("Error in select %s.\n",strerror(errno));
    //exit(1);
  }
  if(res>0) {
    if (FD_ISSET(fd,&fd_read)) {
      ssize_t received = read(fd, &b, 1);
      if ( received == -1 ) {
        perror("read failed after select succeeded:");
        return -1;
      } else {
        return (int) b;
      }
    }
    assert(0); // this should never happen; fd must be set if res>0
  }
  return -1;
}

void connection_open(connection_t *conn)
{
  char port[64];
  strncpy(port,(conn->buffer)+12,63);
  port[63] = 0;
  // there's also some stuff in offset 51, so far seen 0xC0 0x03, probably some
  // argumnets of CreateFile
  printf("Connect command port=%s\n",port);
  char posix_port[80];
  strcpy(posix_port,"/dev/");
  strcat(posix_port,port+4);
  printf("  posix port=%s\n",posix_port);

  // let's pretend we opened the port; we invent a handle
  // (failure should return -1 as the handle).
  //conn->handle = 278; // need to actually open a serial port

  //if ((serial = open(port,O_RDWR | O_NOCTTY | O_SYNC)) == -1) {
  if ((conn->handle = open(posix_port, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
    printf("ERROR: failed to open %s\r\n", posix_port);
    perror("open() failed: ");
    strcpy((conn->buffer)+272, strerror(errno));
  } else {
    // clear buffers and switch to blocking mode
    tcflush(conn->handle, TCOFLUSH);
    tcflush(conn->handle, TCIFLUSH);
    fcntl(conn->handle, F_SETFL, fcntl(conn->handle, F_GETFL) & ~O_NONBLOCK);
  }

  //parse(buffer,total_length);

  uint32ToLittleEndian(conn->handle,(uint8_t *)(conn->buffer)+256);
}

void connection_close(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+256);
  char port[64];
  strncpy(port,(conn->buffer)+12,63);
  port[63] = 0;
  printf("close command, handle=%d (our handle is %d)\n", h, conn->handle);
  printf("  closing port %s\n",port);

  close(h);

  uint32ToLittleEndian(1, (uint8_t *)(conn->buffer)+260); // success code

  conn->done = 1;

  // XXX we need to notify higher layers somehow that we are done
  // Maybe we'll see in read() that the other side closed the connection.
}

void connection_misc(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+256);
  printf("Misc command, handle=%d (our handle is %d)\n",h,conn->handle);

  uint32ToLittleEndian(1,(uint8_t *)(conn->buffer)+260); // success code
}

void connection_waitmask(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+256);
  printf("Waitmask command, handle=%d (our handle is %d)\n",h,conn->handle);

  int event_bitmask = uint32FromLittleEndian((uint8_t *)(conn->buffer)+204);
  printf("  event mask=%08x\n",event_bitmask);

  uint32ToLittleEndian(1,(uint8_t *)(conn->buffer)+260); // success code
}

void connection_purge(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+256);
  printf("Purge command, handle=%d (our handle is %d)\n",h,conn->handle);

  int bitmask = uint32FromLittleEndian((uint8_t *)(conn->buffer)+200);
  printf("  bits=%08x\n",bitmask);

  // here we just flush the buffer that we can
  tcflush(h,TCOFLUSH);
  tcflush(h,TCIFLUSH);

  uint32ToLittleEndian(1,(uint8_t *)(conn->buffer)+260); // success code
}

void connection_parameters(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+256);
  printf("Set Comm Params command, handle=%d (our handle is %d)\n",h,conn->handle);

  int speed =  uint32FromLittleEndian((uint8_t *)(conn->buffer)+212);
  int bitfields = uint32FromLittleEndian((uint8_t *)(conn->buffer)+216);
  int nbits = (conn->buffer)[226];
  int parity= (conn->buffer)[227];
  int stop  = (conn->buffer)[228];
  int xonchar = (conn->buffer)[229];
  int xoffchar= (conn->buffer)[230];
  int errchar = (conn->buffer)[231];
  int eofchar = (conn->buffer)[232];
  int evtchar = (conn->buffer)[233];
  printf("Set parameters command, handle=%d (our handle is %d)\n",h,conn->handle);
  printf("  speed=%d nbits=%d parity=%d stop=%d\n",speed,nbits,parity,stop);
  printf("  xonch=%02x xoffch=%02x err=%02x eof=%02x evt=%02x\n",xonchar,xoffchar,errchar,eofchar,evtchar);
  printf("  bitfields=%08x \n",bitfields);

  //int fBinary           = (bitfields & 0x00000001);
  //int fParity           = (bitfields & 0x00000002) >> 1;
  //int fOutxCtsFlow      = (bitfields & 0x00000004) >> 2;
  //int fOutxDsrFlow      = (bitfields & 0x00000008) >> 3;
  int fDtrControl       = (bitfields & 0x00000030) >> 4;
  //int fDtrSensitivity   = (bitfields & 0x00000040) >> 6;
  //int fTXContinueOnXoff = (bitfields & 0x00000080) >> 7;
  //int fOutX             = (bitfields & 0x00000100) >> 8;
  //int fInX              = (bitfields & 0x00000200) >> 9;
  //int fErrorChar        = (bitfields & 0x00000400) >> 10;
  //int fNull             = (bitfields & 0x00000800) >> 11;
  int fRtsControl       = (bitfields & 0x00003000) >> 12;
  //int fAbortOnError     = (bitfields & 0x00004000) >> 14;
  printf("  bitfields=%08x \n",bitfields);
  // printed but not implemented yet...
  switch (fDtrControl) {
  case 0:
    printf("    DTR=disabled (off)\n");
    break;
  case 1:
    printf("    DTR=enabled (on)\n");
    break;
  case 2:
    printf("    DTR=handshake\n");
    break;
  default:
    printf("    DTR=error!?!\n");
    break;
  }
  switch (fRtsControl) {
  case 0:
    printf("    RTS=disabled (off)\n");
    break;
  case 1:
    printf("    RTS=enabled (on)\n");
    break;
  case 2:
    printf("    RTS=handshake\n");
    break;
  default:
    printf("    RTS=error!?!\n");
    break;
  }


  struct termios tty;
  //bzero(&tty, sizeof(tty));
  //printf("2\n");

  // get current options for the port.
  tcgetattr(h, &tty);
  cfmakeraw(&tty);

  speed_t speed_val;
  switch (speed) {
  case   4800:
    speed_val = B4800  ;
    break;
  case   9600:
    speed_val = B9600  ;
    break;
  case  19200:
    speed_val = B19200 ;
    break;
  case  38400:
    speed_val = B38400 ;
    break;
  case  57600:
    speed_val = B57600 ;
    break;
  case 115200:
    speed_val = B115200;
    break;
    //case   9600: tty.c_cflag |= B9600  ; break;
    //case  19200: tty.c_cflag |= B19200 ; break;
    //case  38400: tty.c_cflag |= B38400 ; break;
    //case  57600: tty.c_cflag |= B57600 ; break;
    //case 115200: tty.c_cflag |= B115200; break;
  default:
    printf("Can't set baud rate %d\n",speed);
    exit(1);
    break;
  }
  cfsetispeed(&tty, speed_val);
  cfsetospeed(&tty, speed_val);

  tty.c_cflag |= (CLOCAL | CREAD);

  switch (nbits) {
  case 8:
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    break;
  case 7:
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS7;
    break;
  default:
    printf("Can't set bits/byte to %d\n",nbits);
    exit(1);
    break;
  }

  if (stop == 2) {
    tty.c_cflag |=  CSTOPB;  // 2 stop bits
  } else {
    tty.c_cflag &= ~CSTOPB;  // only one stop bit
  }

  switch (parity) {
  case 0:
    tty.c_cflag &= ~(PARENB | PARODD);  // no parity
    break;
  case 1:
    tty.c_cflag |= (PARENB | PARODD);  // odd
    break;
  case 2:
    tty.c_cflag |=  PARENB;
    tty.c_cflag &= ~PARODD;  // even
    break;
  default:
    printf("Can't set parity to DCB value %d\n",parity);
    exit(1);
    break;
  }

  // no handshake
  tty.c_cflag &= ~CRTSCTS;
  tty.c_iflag &= ~IXON;

  //case RIG_HANDSHAKE_XONXOFF:
  //  options.c_cflag &= ~CRTSCTS;
  //  options.c_iflag |= IXON;		/* Enable Xon/Xoff software handshaking */
  // break;
  //case RIG_HANDSHAKE_HARDWARE:
  //  options.c_cflag |= CRTSCTS;		/* Enable Hardware handshaking */
  //  options.c_iflag &= ~IXON;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;


  //tty.c_iflag = IGNBRK | IGNPAR;
  // tty.c_oflag = 0;
  //tty.c_lflag = 0;

  //tty.c_cc[VMIN] = 0;
  //tty.c_cc[VTIME] = 1;


  //tty.c_cflag |= CLOCAL | CREAD;
#ifdef _DCDFLOW
  //tty.c_cflag &= ~CRTSCTS;
#endif
  //tty.c_cc[VMIN] = 1;
  //tty.c_cc[VTIME] = 5;

  //  tty.c_iflag |= IXON | IXOFF;
  //tty.c_iflag &= ~(IXON|IXOFF|IXANY); // no flow control
  //tty.c_cflag &= ~(PARENB | PARODD);  // no parity

  tcflush(h,TCOFLUSH);
  tcflush(h,TCIFLUSH);
  tcsetattr(h, TCSANOW, &tty);

  // TODO just say okay for now
  uint32ToLittleEndian(1,(uint8_t *)(conn->buffer)+260); // success code
}

void connection_timeouts(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+256);
  printf("Set Timeouts command, handle=%d (our handle is %d)\n",h,conn->handle);
  int ReadIntervalTimeout         = (int) uint32FromLittleEndian((uint8_t *)(conn->buffer)+236);
  int ReadTotalTimeoutMultiplier  = (int) uint32FromLittleEndian((uint8_t *)(conn->buffer)+240);
  int ReadTotalTimeoutConstant    = (int) uint32FromLittleEndian((uint8_t *)(conn->buffer)+244);
  int WriteTotalTimeoutMultiplier = (int) uint32FromLittleEndian((uint8_t *)(conn->buffer)+248);
  int WriteTotalTimeoutConstant   = (int) uint32FromLittleEndian((uint8_t *)(conn->buffer)+252);
  printf("Timeouts command, handle=%d (our handle is %d)\n",h,conn->handle);
  printf("  %d,%d,%d,%d,%d\n",ReadIntervalTimeout,ReadTotalTimeoutMultiplier,
         ReadTotalTimeoutConstant,WriteTotalTimeoutMultiplier,WriteTotalTimeoutConstant);

  conn->read_interval    = ReadIntervalTimeout;
  conn->read_constant    = ReadTotalTimeoutConstant;
  conn->read_multiplier  = ReadTotalTimeoutMultiplier;
  conn->write_constant   = ReadTotalTimeoutConstant;
  conn->write_multiplier = ReadTotalTimeoutMultiplier;

  uint32ToLittleEndian(1, (uint8_t *)(conn->buffer)+260); // success code
}

void connection_send(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+12);
  int n = uint32FromLittleEndian((uint8_t *)(conn->buffer)+16);
  printf("Send command, handle=%d (our handle is %d)\n",h,conn->handle);
  printf("  %d bytes\n",n);

  // send the data
  //p.send(buffer, 48, n);

  {
    int k;
    printf("  data to radio: ");
    for (k=48; k<48+n; k++) {
      printf("%02x ",(conn->buffer)[k]);
    }
    printf("\n");
  }
  persistent_write(h,(conn->buffer)+48,n);

  uint32ToLittleEndian(n, (uint8_t *)(conn->buffer)+20); // bytes written?
  uint32ToLittleEndian(n, (uint8_t *)(conn->buffer)+28); // bytes written?

  conn->length = 48; // chop the response
  uint32ToLittleEndian(conn->length, (uint8_t *)conn->buffer);
}

// time difference in milliseconds
uint32_t timediff(struct timeval *early, struct timeval *late)
{
  if (early->tv_sec == late->tv_sec) {
    return ((late->tv_usec - early->tv_usec) / 1000);
  }

  return (uint32_t)(((1000000 + late->tv_usec - early->tv_usec) / 1000)
                    + (late->tv_sec - early->tv_sec - 1) * 1000);
}

void connection_receive(connection_t *conn)
{
  int h = uint32FromLittleEndian((uint8_t *)(conn->buffer)+12);
  int n = uint32FromLittleEndian((uint8_t *)(conn->buffer)+16);
  printf("Receive command, handle=%d (our handle is %d)\n", h, conn->handle);
  printf("  %d bytes\n",n);

  conn->length = 52+n; // extend the response
  memset((conn->buffer)+20, 0, (conn->length)-20);

  // receive the data ...
  //p.receive(buffer, 52, n);
  int timeout = 0;
  {
    struct timeval start;
    struct timeval now;
    gettimeofday(&start,NULL);
    int k = 0;

    int msecs_total = (conn->read_constant) + n*(conn->read_multiplier);

    while (k < n) {
      gettimeofday(&now,NULL);
      int timeuntilnow = timediff(&start, &now);
      int timeleft = msecs_total - timeuntilnow;
      if (timeleft <=0) {
        printf("read timeout\n");
        timeout = 1;
        break;
      }
      if (timeleft > conn->read_interval) {
        timeleft = conn->read_interval;
      }
      int rc = serialReceiveByteTimelimited(h,timeleft);
      //if (rc < 0 && errno == ETIMEDOUT) {
      if (rc < 0) {
        timeout = 1;
        break;
      }
      (conn->buffer)[52+k] = (uint8_t) rc;
      printf("%02X ", (uint8_t)(conn->buffer)[52+k]);
      k++;
    }
  }


  //persistent_read(h,(conn->buffer)+52,n);
  //{
  //  int k;
  // printf("  data from radio: ");
  //  for (k=52; k<52+n; k++)
  //    printf("%02x ",(conn->buffer)[k]);
  //  printf("\n");
  // }

  if (timeout) {
    // return 1 zero yte
    conn->length = 52+1;
    (conn->buffer)[52] = 0;

    uint32ToLittleEndian(1,(uint8_t *)(conn->buffer)+24); // bytes written?
    uint32ToLittleEndian(1,(uint8_t *)(conn->buffer)+28); // ???
    uint32ToLittleEndian(0x102,(uint8_t *)(conn->buffer)+36); // time out return code
  } else {
    uint32ToLittleEndian(n,(uint8_t *)(conn->buffer)+24); // bytes written?
    uint32ToLittleEndian(1,(uint8_t *)(conn->buffer)+28); // ???
    uint32ToLittleEndian(n,(uint8_t *)(conn->buffer)+36); // bytes written?
  }

  uint32ToLittleEndian(conn->length,(uint8_t *)conn->buffer);
  // weird signature instead of HRD*
  (conn->buffer)[4] = 0x88;
  (conn->buffer)[5] = 0xff;
  (conn->buffer)[6] = 0xff;
  (conn->buffer)[7] = 0xff;
}


//------------------------------------------------------------------------------
// Purpose    : Create a server socket
// Parameters : port - Port to bind to
// Returns    : socket descriptor
//------------------------------------------------------------------------------
int server_socket_create(int port)
{
  int server_socket;
  if((server_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    perror("Unable to create socket");
    exit(1);
  }
  
  // Turn off bind address checking
  int on = 1;
  if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, 
                 (const char *)&on, sizeof(on)) == -1) {
    perror("Unable to set SO_REUSEADDR");
    exit(1);
  }
  
  // Make sure all data is transmitted before closing port
  struct linger linger = { 0 };
  linger.l_onoff = 1;
  linger.l_linger = 30;
  if (setsockopt(server_socket, SOL_SOCKET, SO_LINGER,
                 (const char *)&linger, sizeof(linger)) == -1) {
    perror("Unable to set SO_LINGER");
    exit(1);
  }
  
  // Bind to the server port
  // Accept connections at any IP address
  struct sockaddr_in addr;
  (void) memset(&addr, 0, sizeof(addr));
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(port); /* network-order */
  if (bind(server_socket, (struct sockaddr *) &addr, sizeof(addr)) == -1) {
    perror("Unable to bind to server port");
    exit(1);
  }
  
  // Listen for connections
  if (listen(server_socket, 5 /* back log */) == -1) {
    perror("Unable to listen for connections");
    exit(1);
  }
  
  return server_socket;
}

//------------------------------------------------------------------------------
// Purpose    : Create a connection socket
// Parameters : sd - server socket
// Returns    : socket descriptor
//------------------------------------------------------------------------------
int connection_socket_create(int server_socket)
{
  struct sockaddr_in client = { 0 };
  int client_len = sizeof(client);
  int connection_socket;
  (void) memset(&client, 0, sizeof(client));
  if((connection_socket = accept(server_socket, (struct sockaddr *)&client,
                                 (socklen_t *)&client_len)) == -1) {
    perror("Unable to create connection socket");
    exit(1);
  }
  
  if(getpeername(connection_socket, (struct sockaddr *)&client,
                 (socklen_t *)&client_len) == -1) {
    perror("Unable to get address of connection");
  } else {
    printf("Connection request from %s\n", inet_ntoa(client.sin_addr));
  }
  
  return connection_socket;
}

//------------------------------------------------------------------------------
// Purpose    : Start a new session on a socket
// Parameters : socket - An open socket
// Returns    : A connection structure
//------------------------------------------------------------------------------
connection_t *start_session(int socket)
{
  connection_t* conn = (connection_t*) malloc(sizeof(connection_t));
  assert (conn!=0);
  conn->buffer = malloc(8192);
  conn->s = socket;
  conn->handle = -1;
  
  conn->read_interval    = 20;
  conn->read_constant    = 20;
  conn->read_multiplier  = 20;
  conn->write_constant   = 20;
  conn->write_multiplier = 20;
  
  conn->done = 0;
  
  pthread_create(&(conn->thread), NULL, (void *)session, conn);
  
  return conn;
}

//------------------------------------------------------------------------------
// Purpose    : Handle a session
// Parameters : conn_v - Void pointer to the connection_t structure
// Returns    : NULL
//------------------------------------------------------------------------------
void *session(void* conn_v)
{
  connection_t *conn = (connection_t *)conn_v;
  
  while (1) {
    printf("reading command...");
    connection_read(conn);            // Get the command
    connection_process_command(conn); // Run the command
    
    if (conn->length == 0) { // If length == 0, there is no response
      continue;
    }
    
    // Respond to command
    size_t n = persistent_write(conn->s, conn->buffer, conn->length);
    assert( n == conn->length );
    
    // If we are finished, close the connection and exit
    if (conn->done) {
      printf("looks like we are done\n");
      free(conn->buffer);
      close(conn->handle);
      free(conn);
      return NULL;
    }
  }
}

//------------------------------------------------------------------------------
// Purpose    : Read a command from the connection
// Parameters : conn - Pointer to the connection_t structure
// Returns    : Number of bytes read
//------------------------------------------------------------------------------
ssize_t connection_read(connection_t* conn)
{
  
  // Read header and parse out length and command number
  ssize_t n = persistent_read(conn->s, conn->buffer, 12);
  assert( n == 12 );
  if ( strncmp((conn->buffer)+4, "HRD*", 4) ) {
    printf("Incoming data missing the HRD* signature\n");
    exit(1);
  }
  conn->length   = uint32FromLittleEndian((uint8_t *)(conn->buffer)+0);
  conn->command  = uint32FromLittleEndian((uint8_t *)(conn->buffer)+8);
  
  // Read command data
  n = persistent_read(conn->s, (conn->buffer)+12, (conn->length) - 12);
  if (n != (conn->length) - 12) {
    printf("read only %ld bytes, waiting for %d\n", n, (conn->length)-12);
    // NOTE: Doesn't actually wait
  }
  //assert(n == (conn->length) - 12);
  
  printf("Command %d length %d\n", conn->command, conn->length);
  
  return conn->length;
}

//------------------------------------------------------------------------------
// Purpose    : Process (run) a command
// Parameters : conn - Pointer to the connection_t structure
// Returns    : void
//------------------------------------------------------------------------------
void connection_process_command(connection_t* conn)
{
  printf("Processing command %d\n", conn->command);
  switch (conn->command) {
  case 0:
    connection_authenticate(conn);
    break;
  case 1:
    enumerate_serial_ports(conn);
    break;
  case 2:
    connection_open(conn);
    break;
  case 3:
    connection_misc(conn);
    break;
  case 4:
    connection_waitmask(conn);
    break;
  case 5:
    connection_purge(conn);
    break;
  case 6:
    connection_parameters(conn);
    break;
  case 7:
    connection_timeouts(conn);
    break;
  case 8:
    connection_receive(conn);
    break;
  case 10:
    connection_send(conn);
    break;
  case 11:
    connection_close(conn);
    break;
  case 12:
    printf("dumping command 12:\n");
    connection_dump(conn);
    printf("\n--\n");
    connection_send(conn);
    conn->length = 0; // no reply
    break;
  case 14:
    connection_logging(conn);
    break;
  default:
    printf("Unknown command code %d\n", conn->command);
    connection_dump(conn);
    exit(1);
    break;
  }
}

//------------------------------------------------------------------------------
// Purpose    : Build message to authenticate user
// Parameters : conn - Pointer to the connection_t structure
// Returns    : void
// Command #  : 0
// Input Msg  : header (0 - 11)
//            : user (16 - 79)
//            : password (80 - 143)
//            : release (144 - 207)
// Output Msg : header (0 - 11)
//            : success flag (12 - 15)
//            : user (16 - 79)
//            : password (80 - 143)
//            : welcome message (144 - 207)
//------------------------------------------------------------------------------
void connection_authenticate(connection_t* conn)
{
  // Parse out user, password, and release number
  char user[64];
  char password[64];
  char release[64];
  strncpy(user, (conn->buffer)+16, 63);
  user[63]     = 0;
  strncpy(password, (conn->buffer)+16+64, 63);
  password[63] = 0;
  strncpy(release, (conn->buffer)+16+2*64, 63);
  release[63]  = 0;
  
  // We currently authenticate everything
  printf("Authenticate command user=%s password=%s release=%s\n",
         user, password, release);
  
  // Build return message
  memset((conn->buffer)+16+2*64, 0, (conn->length)-(16+2*64));
  uint32ToLittleEndian(1, (uint8_t *)(conn->buffer)+12); // signal success; 0 for failure
  strcpy((conn->buffer)+16+2*64, "Welcome to Sivan Toledo's Serial Server");
}

//------------------------------------------------------------------------------
// Purpose    : Build a message to inform the client of available serial ports
// Parameters : conn - Pointer to the connection_t structure
// Returns    : void
// Command #  : 1
// Input Msg  : header (0 - 11)
// Output Msg : header (0 - 11)
//            : success flag (12 - 15)
//            : comma-delimited port list (16 - end)
//------------------------------------------------------------------------------
void enumerate_serial_ports(connection_t *conn)
{
  // Comma-delimited serial port list to return
  char ports[256];
  ports[0] = 0; // initialize to empty string
  printf("Enumerate command ports\n");
  
  // Look for serial ports in the /devs directory
  DIR* devs = opendir("/dev");
  if (devs==NULL) {
    printf("Weird: can't open /dev for listing\n");
  } else {
    struct dirent entry;
    struct dirent* p;
    int port_count=0;
    char port[256];
    do {
      readdir_r(devs, &entry, &p);
      if (!strncmp(entry.d_name,"ttyS", 4)
          || !strncmp(entry.d_name,"ttyUSB", 6)
          || !strncmp(entry.d_name,"tty.usbserial", 13)) {
        sprintf(port, "/dev/%s", entry.d_name);
        int dummy = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (dummy == -1) {
          printf("  %s exists, but cannot be opened, skipping\n", port);
        } else {
          close(dummy);
          if (port_count>0) {
            strcat(ports, ",");
          }
          strcat(ports, entry.d_name);
          port_count++;
        }
      }
    } while (p != NULL);
    closedir(devs);
  }
  
  // Build return message
  printf("  ports=%s\n",ports);
  memset((conn->buffer)+16, 0, (conn->length)-16);
  uint32ToLittleEndian(1, (uint8_t *)(conn->buffer)+12); // signal success; 0 for failure
  strcpy((conn->buffer)+16, ports);
  conn->length = 16 + (int)strlen(ports) + 1; // adjust size
  uint32ToLittleEndian(conn->length, (uint8_t *)conn->buffer); // and fix header accordingly
}

//------------------------------------------------------------------------------
// Purpose    : Log a message
// Parameters : conn - Pointer to the connection_t structure
// Returns    : void
// Command #  : 12
// Input Msg  : header (0 - 11)
// Output Msg : None
//------------------------------------------------------------------------------
void connection_logging(connection_t *conn)
{
  // Don't know what these numbers are
  int n1 = uint32FromLittleEndian((uint8_t *)(conn->buffer)+12);
  int n2 = uint32FromLittleEndian((uint8_t *)(conn->buffer)+16);
  
  // Port string
  char port[32];
  strncpy(port,(conn->buffer)+20,31);
  port[32] = 0;
  
  // Message string
  char msg[64];
  strncpy(port,(conn->buffer)+52,63);
  msg [64] = 0;
  
  // Log the message to the screen
  printf("Logging command (%d,%d,%s)\n", n1, n2, port);
  printf("  <%s>\n",msg);
  
  // don't send a response
  conn->length = 0;
}

//------------------------------------------------------------------------------
// Purpose    : Dump a command to the screen
// Parameters : conn - Pointer to the connection_t structure
// Returns    : void
//------------------------------------------------------------------------------
void connection_dump(connection_t *conn)
{
  printf("--- Dump Command, Length: %d---\n", conn->length);
  int k,q;
  q=0;
  for (k=0; k<conn->length; k++) {
    printf("%02X ", (uint8_t)(conn->buffer)[k]);
    q++;
    if (q==26) {
      q=0;
      printf("\n");
    }
  }
  printf("\n--- End Command ---\n");
}

//------------------------------------------------------------------------------
// Purpose    : Read bytes from a stream
// Parameters : fd    - File handle to read from
//            : buf   - Location which bytes should be written
//            : count - Number of bytes to read
// Returns    : Number of bytes read
//------------------------------------------------------------------------------
ssize_t persistent_read(int fd, void *buf, size_t count)
{
  ssize_t n = 0;
  while (n < count) {
    ssize_t m = read(fd, buf+n, count-n);
    if (m == -1) {
      return -1;
    }
    n = n+m;
  }
  return n;
}

//------------------------------------------------------------------------------
// Purpose    : Write bytes to a stream
// Parameters : fd    - File handle to write tp
//            : buf   - Location which bytes should be read from
//            : count - Number of bytes to write
// Returns    : Number of bytes written
//------------------------------------------------------------------------------
ssize_t persistent_write(int fd, void *buf, size_t count)
{
  ssize_t n = 0;
  while (n < count) {
    ssize_t m = write(fd, buf+n, count-n);
    if (m == -1) {
      return -1;
    }
    n = n+m;
  }
  return n;
}


