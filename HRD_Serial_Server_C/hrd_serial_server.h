#ifndef GUARD_HRD_SERIAL_SERVER_H
#define GUARD_HRD_SERIAL_SERVER_H
//
//  hrd_serial_server.h
//  HRD_Serial_Server_C
//
//  Created by Sivon Toledo.
//  Modified by Steve Baker.
//

/*****************************************************/
/* HRD connections                                   */
/*****************************************************/

typedef struct {
  //char* user = null;
  //public String pass = null;
  //public String port = null;
  int length, command;
  char *buffer;
  int s;
  
  int handle;
  
  int read_interval, read_constant, read_multiplier, write_constant, write_multiplier;
  
  int done; // are we done with this connection?
  
  pthread_t thread;
} connection_t;

int serialReceiveByteTimelimited(int fd, int msecs);
ssize_t persistent_read(int fd, void *buf, size_t count);
size_t persistent_write(int fd, void *buf, size_t count);
int connection_socket_create(int sd);
int server_socket_create(int port);
connection_t* connection_create(int socket);
void connection_authenticate(connection_t* conn);
void connection_enumerate(connection_t *conn);
void connection_open(connection_t *conn);
void connection_close(connection_t *conn);
void connection_misc(connection_t *conn);
void connection_waitmask(connection_t *conn);
void connection_purge(connection_t *conn);
void connection_parameters(connection_t *conn);
void connection_timeouts(connection_t *conn);
void connection_send(connection_t *conn);
uint32_t timediff(struct timeval *early, struct timeval *late);
void connection_receive(connection_t *conn);
void connection_logging(connection_t *conn);
void connection_process_command(connection_t* conn);
ssize_t connection_read(connection_t* conn);
void *connection_thread(void *conn_v);

#endif
