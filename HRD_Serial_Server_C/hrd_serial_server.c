/*
 * old
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <sys/utsname.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#include <errno.h>
#include <time.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>


#include <assert.h>

#include "endian.c"

/********************************************************/
/* time limited I/O                                     */
/********************************************************/

static int serialReceiveByteTimelimited(int fd, int msecs) {
  uint8_t b;
  int received;

  fd_set fd_read;
  FD_ZERO(&fd_read);
  //FD_SET(packetizerReadfd,&fd_read);
  FD_SET(fd,&fd_read);

  struct timeval timeout;
  timeout.tv_sec=0;
  timeout.tv_usec=1000*msecs; /* 25ms */

  int res=select(FD_SETSIZE,&fd_read,0,0,&timeout);
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
      received = read(fd, &b, 1);
      if ( received == -1 ) {
        perror("read failed after select succeeded:");
        return -1;
      } else {
        return (int) b;
      }
    }
    assert(0); // this should never happen; fd must be set if res>0
  }
}

/********************************************************/
/* persistent reads and writes                          */
/********************************************************/

ssize_t persistent_read(int fd, void *buf, size_t count) {
  size_t n = 0;
  
  while (n < count) {
    size_t m = read(fd, buf+n, count-n);
    if (m == -1) return -1;
    n = n+m;
  }
}

ssize_t persistent_write(int fd, void *buf, size_t count) {
  size_t n = 0;
  
  while (n < count) {
    size_t m = write(fd, buf+n, count-n);
    if (m == -1) return -1;
    n = n+m;
  }
}


int connection_socket_create(int sd) {
  struct sockaddr_in client = { 0 };
  int client_len = sizeof(client);
  int s;
  (void) memset(&client, 0, sizeof(client));
  s = accept(sd, (struct sockaddr *) &client, &client_len);
  if (s == -1) {
    perror("accept()");
    exit(1);
  }

  if (getpeername(s, (struct sockaddr *) &client, &client_len) == -1) {
    perror("getpeername()");
  } else {
    printf("Connection request from %s\n", inet_ntoa(client.sin_addr));
  }

  return s;
}


int server_socket_create(int port) {
  int sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sd == -1) {
      perror("socket()");
      exit(1);
  }

  /*
   * turn off bind address checking, and allow port numbers
   * to be reused - otherwise the TIME_WAIT phenomenon will
   * prevent binding to these address.port combinations for
   * (2 * MSL) seconds.
   */

  int on = 1;
  int status;
  if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR,
                 (const char *) &on, sizeof(on)) == -1) {
      perror("setsockopt(...,SO_REUSEADDR,...)");
  }

  /*
   * when connection is closed, there is a need to linger to ensure
   * all data is transmitted, so turn this on also
   */

  struct linger linger = { 0 };
  linger.l_onoff = 1;
  linger.l_linger = 30;
  if (setsockopt(sd, SOL_SOCKET, SO_LINGER,
                 (const char *) &linger, sizeof(linger)) == -1) {
    perror("setsockopt(...,SO_LINGER,...)");
  }

#if 0
  /*
   * find out who I am
   */

  struct utsname sysname = { 0 };
  if (uname(&sysname) != -1) {
    strncpy(buffer, sysname.nodename, length);
  } else {
    perror("uname");
    exit(1);
  }
  
  hostPtr = gethostbyname(hostname);
  if (NULL == hostPtr) {
      perror("gethostbyname()");
      exit(1);
  }

  (void) memset(&serverName, 0, sizeof(serverName));
  (void) memcpy(&serverName.sin_addr, hostPtr->h_addr,
		hostPtr->h_length);
#endif

  /*
   * to allow server be contactable on any of its
   * IP addresses, uncomment the following line of code:
   *
   * serverName.sin_addr.s_addr = htonl(INADDR_ANY);
   */

  struct sockaddr_in addr;
  (void) memset(&addr, 0, sizeof(addr));
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_family      = AF_INET;
  addr.sin_port        = htons(port); /* network-order */
  if (bind(sd, (struct sockaddr *) &addr, sizeof(addr)) == -1) {
      perror("bind()");
      exit(1);
  }

  if (listen(sd, 5 /* back log */) == -1) {
      perror("listen()");
      exit(1);
  }

  return sd;
}

/*****************************************************/
/* HRD connections                                   */
/*****************************************************/

typedef struct {
  //char* user = null;
  //public String pass = null;
  //public String port = null;
  int length, command;
  uint8_t *buffer;
  int s;
  
  int handle;

  int read_interval, read_constant, read_multiplier, write_constant, write_multiplier;

  int done; // are we done wiht this connection?

  pthread_t thread;
} connection_t;

void* connection_thread(void* conn_v);

connection_t* connection_create(int socket) {
  connection_t* conn = (connection_t*) malloc(sizeof(connection_t));
  assert (conn!=0);
  conn->buffer = (uint8_t*) malloc(8192);
  conn->s = socket;
  conn->handle = -1;

  conn->read_interval    = 20;
  conn->read_constant    = 20;
  conn->read_multiplier  = 20;
  conn->write_constant   = 20;
  conn->write_multiplier = 20;

  conn->done = 0;

  pthread_create(&(conn->thread),NULL,connection_thread,conn);

  return conn;
}

int connection_authenticate(connection_t* conn) {
  char user[64];
  char password[64];
  char release[64];
  strncpy(user,(conn->buffer)+16   ,63);      user[63]    =0;
  strncpy(password,(conn->buffer)+16+64,63);  password[63]=0;
  strncpy(release,(conn->buffer)+16+2*64,63); release[63] =0;

  printf("Authenticate command user=%s password=%s release=%s\n",user,password,release);


  memset((conn->buffer)+16+2*64, 0, (conn->length)-(16+2*64));
  uint32ToLittleEndian(1,(conn->buffer)+12); // signal success; 0 for failure
  strcpy((conn->buffer)+16+2*64,"Welcome to Sivan Toledo's Serial Server");
}
#include <dirent.h>
int connection_enumerate(connection_t* conn) {
  //char* ports = "COM1,COM21,ttyUSB0,ttyUSB1";
  char ports[256];
  ports[0] = 0; // initialize to empty string
  char port[256];
  printf("Enumerate command ports=%s\n",ports);

  DIR* devs = opendir("/dev");
  if (devs==0) {
    printf("Weird: can't open /dev for listing\n");
  }
  struct dirent entry;
  struct dirent* p;
  int n=0;
  do {
    int rc = readdir_r(devs, &entry, &p);
    if (!strncmp(entry.d_name,"ttyS",4) || !strncmp(entry.d_name,"ttyUSB",6)) {
      sprintf(port,"/dev/%s",entry.d_name);
      int dummy = open(port,O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (dummy == -1) {
	printf("  %s exists, but cannot be opened, skipping\n",port);
      } else {
	close(dummy);
	if (n>0) strcat(ports,",");
	strcat(ports,entry.d_name);
	n++;
      }
    }
    //printf("%s \n",entry.d_name);
  } while (p != NULL);
  closedir(devs);

  printf("  ports=%s\n",ports);

  memset((conn->buffer)+16, 0, (conn->length)-16);
  uint32ToLittleEndian(1,(conn->buffer)+12); // signal success; 0 for failure
  strcpy((conn->buffer)+16,ports);
  conn->length = 16 + strlen(ports) + 1; // adjust size
  uint32ToLittleEndian(conn->length,conn->buffer); // and fix header accordingly
}
int connection_open(connection_t* conn) {
  char port[64];
  strncpy(port,(conn->buffer)+12,63); port[63] = 0;
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
  if ((conn->handle = open(posix_port,O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
    printf("ERROR: failed to open %s\r\n",posix_port);
    perror("open() failed: ");
    strcpy((conn->buffer)+272,strerror(errno));
  } else {
    // clear buffers and switch to blocking mode
    tcflush(conn->handle,TCOFLUSH);
    tcflush(conn->handle,TCIFLUSH);
    fcntl(conn->handle,F_SETFL, fcntl(conn->handle,F_GETFL) & ~O_NONBLOCK);
  }

  //parse(buffer,total_length);

  uint32ToLittleEndian(conn->handle,(conn->buffer)+256);
}
int connection_close(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+256); 
  char port[64];
  strncpy(port,(conn->buffer)+12,63); port[63] = 0;
  printf("close command, handle=%d (our handle is %d)\n",h,conn->handle);
  printf("  closing port %s\n",port);

  close(h);

  uint32ToLittleEndian(1,(conn->buffer)+260); // success code

  conn->done = 1;

  // XXX we need to notify higher layers somehow that we are done
  // Maybe we'll see in read() that the other side closed the connection.
}
int connection_misc(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+256); 
  printf("Misc command, handle=%d (our handle is %d)\n",h,conn->handle);

  uint32ToLittleEndian(1,(conn->buffer)+260); // success code
}
int connection_waitmask(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+256); 
  printf("Waitmask command, handle=%d (our handle is %d)\n",h,conn->handle);

  int event_bitmask = uint32FromLittleEndian((conn->buffer)+204); 
  printf("  event mask=%08x\n",event_bitmask);

  uint32ToLittleEndian(1,(conn->buffer)+260); // success code
}
int connection_purge(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+256); 
  printf("Purge command, handle=%d (our handle is %d)\n",h,conn->handle);

  int bitmask = uint32FromLittleEndian((conn->buffer)+200); 
  printf("  bits=%08x\n",bitmask);

  // here we just flush the buffer that we can
  tcflush(h,TCOFLUSH);
  tcflush(h,TCIFLUSH);

  uint32ToLittleEndian(1,(conn->buffer)+260); // success code
}
int connection_parameters(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+256); 
  printf("Set Comm Params command, handle=%d (our handle is %d)\n",h,conn->handle);

  int speed =  uint32FromLittleEndian((conn->buffer)+212);
  int bitfields = uint32FromLittleEndian((conn->buffer)+216);
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

  int fBinary           = (bitfields & 0x00000001);
  int fParity           = (bitfields & 0x00000002) >> 1;
  int fOutxCtsFlow      = (bitfields & 0x00000004) >> 2;
  int fOutxDsrFlow      = (bitfields & 0x00000008) >> 3;
  int fDtrControl       = (bitfields & 0x00000030) >> 4;
  int fDtrSensitivity   = (bitfields & 0x00000040) >> 6;
  int fTXContinueOnXoff = (bitfields & 0x00000080) >> 7;
  int fOutX             = (bitfields & 0x00000100) >> 8;
  int fInX              = (bitfields & 0x00000200) >> 9;
  int fErrorChar        = (bitfields & 0x00000400) >> 10;
  int fNull             = (bitfields & 0x00000800) >> 11;
  int fRtsControl       = (bitfields & 0x00003000) >> 12;
  int fAbortOnError     = (bitfields & 0x00004000) >> 14;
  printf("  bitfields=%08x \n",bitfields);
  // printed but not implemented yet...
  switch (fDtrControl) {
  case 0:  printf("    DTR=disabled (off)\n"); break;
  case 1:  printf("    DTR=enabled (on)\n"); break;
  case 2:  printf("    DTR=handshake\n"); break;
  default: printf("    DTR=error!?!\n"); break;
  }
  switch (fRtsControl) {
  case 0:  printf("    RTS=disabled (off)\n"); break;
  case 1:  printf("    RTS=enabled (on)\n"); break;
  case 2:  printf("    RTS=handshake\n"); break;
  default: printf("    RTS=error!?!\n"); break;
  }


  struct termios tty;
  //bzero(&tty, sizeof(tty));
  //printf("2\n");
  
  // get current options for the port.
  tcgetattr(h, &tty);
  cfmakeraw(&tty);

  speed_t speed_val;
  switch (speed) {
  case   4800: speed_val = B4800  ; break;
  case   9600: speed_val = B9600  ; break;
  case  19200: speed_val = B19200 ; break;
  case  38400: speed_val = B38400 ; break;
  case  57600: speed_val = B57600 ; break;
  case 115200: speed_val = B115200; break;
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

  if (stop == 2) tty.c_cflag |=  CSTOPB;             // 2 stop bits
  else           tty.c_cflag &= ~CSTOPB;             // only one stop bit

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
  uint32ToLittleEndian(1,(conn->buffer)+260); // success code
}
int connection_timeouts(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+256); 
  printf("Set Timeouts command, handle=%d (our handle is %d)\n",h,conn->handle);
  int ReadIntervalTimeout         = (int) uint32FromLittleEndian((conn->buffer)+236);
  int ReadTotalTimeoutMultiplier  = (int) uint32FromLittleEndian((conn->buffer)+240);
  int ReadTotalTimeoutConstant    = (int) uint32FromLittleEndian((conn->buffer)+244);
  int WriteTotalTimeoutMultiplier = (int) uint32FromLittleEndian((conn->buffer)+248);
  int WriteTotalTimeoutConstant   = (int) uint32FromLittleEndian((conn->buffer)+252);
  printf("Timeouts command, handle=%d (our handle is %d)\n",h,conn->handle);
  printf("  %d,%d,%d,%d,%d\n",ReadIntervalTimeout,ReadTotalTimeoutMultiplier,
	 ReadTotalTimeoutConstant,WriteTotalTimeoutMultiplier,WriteTotalTimeoutConstant);

  conn->read_interval    = ReadIntervalTimeout;
  conn->read_constant    = ReadTotalTimeoutConstant;
  conn->read_multiplier  = ReadTotalTimeoutMultiplier;
  conn->write_constant   = ReadTotalTimeoutConstant;
  conn->write_multiplier = ReadTotalTimeoutMultiplier;

  uint32ToLittleEndian(1,(conn->buffer)+260); // success code
}
int connection_send(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+12); 
  int n = uint32FromLittleEndian((conn->buffer)+16);
  printf("Send command, handle=%d (our handle is %d)\n",h,conn->handle);
  printf("  %d bytes\n",n);

  // send the data
  //p.send(buffer, 48, n);

  {
    int k;
    printf("  data to radio: ");
    for (k=48; k<48+n; k++)
      printf("%02x ",(conn->buffer)[k]);
    printf("\n");
  }
  persistent_write(h,(conn->buffer)+48,n);

  uint32ToLittleEndian(n,(conn->buffer)+20); // bytes written?
  uint32ToLittleEndian(n,(conn->buffer)+28); // bytes written?

  conn->length = 48; // chop the response
  uint32ToLittleEndian(conn->length,conn->buffer);
}

// time difference in milliseconds
uint32_t timediff(struct timeval* early, struct timeval* late) {
  if (early->tv_sec == late->tv_sec)
    return ((late->tv_usec - early->tv_usec) / 1000);

  return ((1000000 + late->tv_usec - early->tv_usec) / 1000)
    + (late->tv_sec - early->tv_sec - 1) * 1000;
}

int connection_receive(connection_t* conn) {
  int h = uint32FromLittleEndian((conn->buffer)+12); 
  int n = uint32FromLittleEndian((conn->buffer)+16);
  printf("Receive command, handle=%d (our handle is %d)\n",h,conn->handle);
  printf("  %d bytes\n",n);

  conn->length = 52+n; // extend the response
  memset((conn->buffer)+20,0,(conn->length)-20);

  // receive the data ...
  //p.receive(buffer, 52, n);
  int timeout = 0;
  {
    struct timeval start;
    struct timeval now;
    gettimeofday(&start,NULL);
    int k = 0;
    int msecs;

    int msecs_total = (conn->read_constant) + n*(conn->read_multiplier);

    while (k < n) {
      gettimeofday(&now,NULL);
      int timeuntilnow = timediff(&start,&now);
      int timeleft = msecs_total - timeuntilnow;
      if (timeleft <=0) {
	printf("read timeout\n");
	timeout = 1;
	break;
      }
      if (timeleft > conn->read_interval) timeleft = conn->read_interval;
      int rc = serialReceiveByteTimelimited(h,timeleft);
      //if (rc < 0 && errno == ETIMEDOUT) {
      if (rc < 0) {
	timeout = 1;
	break;
      }
      (conn->buffer)[52+k] = (uint8_t) rc;
      printf("%02x ",(conn->buffer)[52+k]);
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

    uint32ToLittleEndian(1,(conn->buffer)+24); // bytes written?
    uint32ToLittleEndian(1,(conn->buffer)+28); // ???
    uint32ToLittleEndian(0x102,(conn->buffer)+36); // time out return code
  } else {
    uint32ToLittleEndian(n,(conn->buffer)+24); // bytes written?
    uint32ToLittleEndian(1,(conn->buffer)+28); // ???
    uint32ToLittleEndian(n,(conn->buffer)+36); // bytes written?
  }

  uint32ToLittleEndian(conn->length,conn->buffer);
  // weird signature instead of HRD*
  (conn->buffer)[4] = 0x88;
  (conn->buffer)[5] = 0xff;
  (conn->buffer)[6] = 0xff;
  (conn->buffer)[7] = 0xff;
}
int connection_logging(connection_t* conn) {
  int n1 = uint32FromLittleEndian((conn->buffer)+12);
  int n2 = uint32FromLittleEndian((conn->buffer)+16);
  /*
  char port[32];
  strncpy(port,(conn->buffer)+20,31); port[32] = 0;
  char msg[64];
  strncpy(port,(conn->buffer)+52,63); msg [64] = 0;

  printf("Logging command (%d,%d,%s)\n",n1,n2,port);
  printf("  <%s>\n",msg);
  */
  conn->length = 0; // don't send a response
}

int connection_process_command(connection_t* conn) {
  printf("Processing command %d\n",conn->command);
  switch (conn->command){
    case    0: connection_authenticate(conn); break;
    case    1: connection_enumerate(conn);    break;
    case    2: connection_open(conn);         break;
    case    3: connection_misc(conn);         break;
    case    4: connection_waitmask(conn);     break;
    case    5: connection_purge(conn);        break;
    case    6: connection_parameters(conn);   break;
    case    7: connection_timeouts(conn);     break;
    case 0x0c:
      {
	int k,q;
	q=0;
	printf("dumping command 12:\n");
	for (k=0; k<conn->length; k++) {
	  printf("%02x ",(conn->buffer)[k]);
	  q++;
	  if (q==15) {q=0; printf("\n");}
	}
	printf("\n--\n");
      }
      connection_send(conn);
      conn->length = 0; // no reply
      break;
    case 0x0a: connection_send(conn);         break;
    case    8: connection_receive(conn);      break;
    case 0x0b: connection_close(conn);        break;
    case 0x0e: connection_logging(conn);      break;
    default:
      printf("Unknown command code %d\n",conn->command);

      {
	int k,q;
	q=0;
	for (k=0; k<conn->length; k++) {
	  printf("%02x ",(conn->buffer)[k]);
	  q++;
	  if (q==15) {q=0; printf("\n");}
	}
	printf("\n");
      }

      exit(1);
      break;
  }
}

int connection_read(connection_t* conn) {
  int n;

  n = persistent_read(conn->s, conn->buffer, 12);
  assert( n == 12 );

  if ( strncmp((conn->buffer)+4, "HRD*", 4) ) {
    printf("Incoming data missing the HRD* signature\n");
    exit(1);
  }

  conn->length   = uint32FromLittleEndian((conn->buffer)+0);
  conn->command  = uint32FromLittleEndian((conn->buffer)+8);

  n = persistent_read(conn->s, (conn->buffer)+12, (conn->length) - 12);
  if (n != (conn->length) - 12) printf("read only %d bytes, waiting for %d\n",n,(conn->length)-12);
  assert( n == (conn->length) - 12 );

  printf("Command %d length %d\n", conn->command, conn->length);
}

//void connection_thread(int socket) {
void* connection_thread(void* conn_v) {
  connection_t* conn = (connection_t*) conn_v;
    //connection_t* conn = connection_create(socket);

  while (1) {
    printf("reading command...");
    connection_read(conn);
    connection_process_command(conn);

    if (conn->length == 0) continue;
      
    int n = persistent_write(conn->s, conn->buffer, conn->length);
    assert( n == conn->length );

    if (conn->done) {
      printf("looks like we are done\n");
      free(conn->buffer);
      close(conn->handle);
      free(conn);
      return;
    }

  }
}

#include "config.h"
dictionary_t config;
char* config_locations[] = {
  "/etc",
  "./",
  0
};


/*
int main(int argc, char** argv) {
  printf(">>> %s\n",configGetString(config, "vtrack", 'v', "vtrack-default"));
  printf(">>> %s\n",configGetString(config, "test", 'v', "test-default"));
  printf(">>> %s\n",configGetString(config, "noshort", 0, "noshort-default"));
  printf(">>> %s\n",configGetString(config, "missing", 0, "missing-default"));
  printf(">>> %s\n",configGetString(config, "short", 's', "short-default"));
}
*/

int main(int argc, char** argv) {

  printf("1\n");

  config =
    configInit("hrd_serial_server.conf",
	       config_locations,
	       argc,
	       argv);
  //dump(config);

  int port = configGetInt(config, "port", 0, 7805);
  //printf(">>> %s\n",configGetString(config, "port", 0, "7805"));
  printf("port %d\n",port);

  int ss = server_socket_create(port);
  while (1) {
    int c  = connection_socket_create(ss);
    connection_create(c);
  }
}


