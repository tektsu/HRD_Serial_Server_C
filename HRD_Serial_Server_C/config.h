#ifndef GUARD_CONFIG_H
#define GUARD_CONFIG_H
//
//  config.h
//  hrd_serial_server
//
//  Created by Sivon Toledo.
//  Modified by Steve Baker.
//

struct dictionary_item_st {
  char *identifier;
  char *value;
  struct dictionary_item_st *next;
  char  short_id;
};

struct dictionary_st {
  struct dictionary_item_st *items;
};

typedef struct dictionary_st *dictionary_t;

dictionary_t configInit(char *, char **, int, char **);
char* configGetString(dictionary_t dict, char* id, char short_id, char *dflt);
int configGetInt(dictionary_t dict, char *id, char short_id, int dflt);
int configGetBoolean(dictionary_t dict, char *id, char short_id, int dflt);

#endif
