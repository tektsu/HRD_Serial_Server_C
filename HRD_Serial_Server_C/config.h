
#ifndef CONFIG_H
#define CONFIG_H

struct dictionary_st;
typedef struct dictionary_st* dictionary_t;

dictionary_t configInit(char*, char**, int, char**);
char* configGetString(dictionary_t dict, char* id, char short_id, char* dflt);
int configGetInt(dictionary_t dict, char* id, char short_id, int dflt);
int configGetBoolean(dictionary_t dict, char* id, char short_id, int dflt);

#endif
