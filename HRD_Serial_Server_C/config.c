/*
 * Copyright Sivan Toledo, 2009
 *
 * This code is licenced under the GNU General Public License.
 * See http://www.gnu.org/licenses/gpl.html.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <assert.h>

#include "config.h"

/*** dictionary management ***/

struct dictionary_item_st {
  char* identifier;
  char* value;
  struct dictionary_item_st* next;
  char  short_id;
};

struct dictionary_st {
  struct dictionary_item_st* items;
};

static void update_item(dictionary_t dict, char* id, char* v) {
  struct dictionary_item_st* item = dict->items;

  while (item) {
    if (item->identifier && strcmp(item->identifier,id)==0) {
      if (item->value) free(item->value);
      if (v) {
        //printf("replacing <%s> with value <%s>\n",item->identifier,v);
        item->value = malloc(strlen(v)+1);
        assert(item->value);
        strcpy(item->value,v);
        return;
      } else
        item->value = 0;
    }
    item = item->next;
  }

  if (!item) {
    //printf("adding identifier <%s>=<%s>\n",id,v);
    item = (struct dictionary_item_st*) malloc(sizeof(struct dictionary_item_st));
    assert(item);
    item->short_id = 0;
    if (v) {
      item->value = malloc(strlen(v)+1);
      assert(item->value);
      strcpy(item->value,v);
    }
    item->identifier = malloc(strlen(id)+1);
    assert(item->identifier);
    strcpy(item->identifier,id);
    item->next = dict->items;
    dict->items = item;
  } else
    assert(0);
}

static void update_item_short(dictionary_t dict, char short_id, char* v) {
  struct dictionary_item_st* item = dict->items;

  //printf("update short %c\n",short_id);
  while (item) {
    if (item->short_id==short_id) {
      if (item->value) free(item->value);
      if (v) {
        //printf("replacing <-%c> with value <%s>\n",short_id,v);
        item->value = malloc(strlen(v)+1);
        assert(item->value);
        strcpy(item->value,v);
        return;
      } else
        item->value = 0;
    }
    item = item->next;
  }

  if (!item) {
    //printf("adding identifier <-%c>=<%s>\n",short_id,v);
    item = (struct dictionary_item_st*) malloc(sizeof(struct dictionary_item_st));
    assert(item);
    item->short_id = short_id;
    if (v) {
      item->value = malloc(strlen(v)+1);
      assert(item->value);
      strcpy(item->value,v);
    }
    item->identifier = 0;
    item->next = dict->items;
    dict->items = item;
  } else
    assert(0);
}

static void dump(dictionary_t dict) {
  struct dictionary_item_st* item = dict->items;

  while (item) {
    printf("%s=%s (short %c)\n",item->identifier,item->value,item->short_id?item->short_id:'?');
    item = item->next;
  }
}

  ///* process a client request to get an item, associating the long and short ids */

static struct dictionary_item_st* get_helper(dictionary_t dict, char* id, char short_id) {
  struct dictionary_item_st* item = dict->items;

  //printf("searching for --%s -%c\n",id,short_id?short_id:'?');

  while (item) {
    /* if we have already associated this short id with an id, return the item */
    if (item->identifier && strcmp(id,item->identifier)==0 && item->short_id==short_id)
      return item;
    /* but if we associated it with another id, there's a bug in the client code that queries the dict */
    if (item->identifier && strcmp(id,item->identifier)!=0 && short_id!=0 && item->short_id==short_id) {
      printf("an attempt to associate flag -%c with both --%s and --%s\n",short_id,item->identifier,id);
      return 0;
    }
    item = item->next;
  }

  //printf("  not already associated, looking for short id\n");
  /* not already associated; we try to associate */
  item = dict->items;
  while (item) {
    /* here is the short id */
    //printf("    testing <--%s> <-%c>\n",item->identifier,item->short_id);
    if (item->identifier==0 && item->short_id==short_id) {
      //printf("associating --%s with -%c\n",id,short_id);
      item->identifier = malloc(strlen(id)+1);
      assert(item->identifier);
      strcpy(item->identifier,id);
      return item;
    }
    item = item->next;
  }

  //printf("  not already associated, looking for full id\n");
  item = dict->items;
  while (item) {
    /* here is the short id */
    //printf("    testing <--%s> <-%c>\n",item->identifier,item->short_id);
    if (item->identifier && strcmp(id,item->identifier)==0) {
      //printf("associating --%s with -%c\n",id,short_id);
      item->short_id = short_id;
      return item;
    }
    item = item->next;
  }

  /* not found */
  return NULL;
}

char* configGetString(dictionary_t dict, char* id, char short_id, char* dflt) {
  //struct dictionary_item_st* item = dict->items;

  //while (item) {
  //  if (item->identifier && strcmp(id,item->identifier)==0) return item->value;
  //  item = item->next;
  //}

  struct dictionary_item_st* item = get_helper(dict, id, short_id);
  if (item) return item->value;

  return dflt;
}

int configGetInt(dictionary_t dict, char* id, char short_id, int dflt) {
  //struct dictionary_item_st* item = dict->items;
  int v;

  //while (item) {
  //  if (item->identifier && strcmp(id,item->identifier)==0) {
  //    if (sscanf(item->value,"%d",&v)==1) return v;
  //    else {
	//printf("config warning: value of %s should be an integer (not %s)\n",
	//       id,item->value);
//	return dflt;
 //     }
  //  }
   // item = item->next;
 // }

  struct dictionary_item_st* item = get_helper(dict, id, short_id);
  if (item) {
    if (sscanf(item->value,"%d",&v)==1) return v;
    else {
      printf("config warning: value of %s should be an integer (not %s)\n",
             id,item->value);
      return dflt;
    }
  }

  return dflt;
}

uint32_t configGetUInt32Hex(dictionary_t dict, char* id, char short_id, uint32_t dflt) {
  //struct dictionary_item_st* item = dict->items;
  uint32_t v;

  //while (item) {
  //  if (item->identifier && strcmp(id,item->identifier)==0) {
  //    if (sscanf(item->value,"%d",&v)==1) return v;
  //    else {
    //printf("config warning: value of %s should be an integer (not %s)\n",
    //       id,item->value);
//  return dflt;
 //     }
  //  }
   // item = item->next;
 // }

  struct dictionary_item_st* item = get_helper(dict, id, short_id);
  if (item) {
    if (sscanf(item->value,"0x%x",&v)==1) return v;
    else {
      printf("config warning: value of %s should be a hexadecimal value (not %s)\n",
             id,item->value);
      return dflt;
    }
  }

  return dflt;
}


int configGetBoolean(dictionary_t dict, char* id, char short_id, int dflt) {
  int v;

  struct dictionary_item_st* item = get_helper(dict, id, short_id);
  if (item) {
    if (!item->value) return 1; /* exists, although no value */
    if (!strcmp(item->value,"1")) return 1;
    if (!strcmp(item->value,"0")) return 0;
    if (!strcasecmp(item->value,"true"))  return 1;
    if (!strcasecmp(item->value,"false")) return 0;
    if (!strcasecmp(item->value,"yes"))   return 1;
    if (!strcasecmp(item->value,"no"))    return 0;
    printf("config warning: value of %s should be a boolean (not %s)\n",
           id,item->value);
  }

  return dflt;
}

/*** substituting environment variables (like $HOME) in paths ***/

static void env_substitute(char** env_variable, char** replacement) {
  char  varname[256];
  char* p = varname;
  while (isalpha(**env_variable)
	 || isdigit(**env_variable)
	 || **env_variable == '_') {
    *p = **env_variable;
    p++;
    (*env_variable)++;
  }
  *p = 0;

  char* t = getenv(varname);
  //printf("env variable %s\n",varname);
  if (t) {
    strcpy(*replacement, t);
    *replacement += strlen(t);
    //printf("translation %s\n",t);
  } //else printf("not found\n");
}

/*** configuration file parsing ***/

static void skip_whitespace(char** p, size_t* n) {
  while (*n > 0
	 &&
	 (**p == ' ' || **p == '\t' || **p == '\r' || **p == '\f' || **p == '\v')) {
    (*p) ++;
    (*n) --;
  }
}

static void skip_equals_sign(char** p, size_t* n) {
  while (*n > 0
     &&
     **p == '=') {
    (*p) ++;
    (*n) --;
  }
}

static void skip_to_eol(char** p, size_t* n) {
  while (*n > 0
	 &&
	 (**p != '\n')) {
    (*p) ++;
    (*n) --;
  }

  if (**p == '\n') {
    (*p) ++;
    (*n) --;
  }
}

static void get_identifier(char** p, size_t* n, char* id) {
  while (*n > 0
     &&
     **p != '=' &&**p != ' ' && **p != '\t' && **p != '\r' && **p != '\f' && **p != '\v' && **p != '\n') {
    *id = **p;
    id++;
    (*p) ++;
    (*n) --;
  }

  *id=0;
}

static void get_value(char** p, size_t* n, char* value) {
  if (**p=='"' || **p=='\'') {
    char quote = **p;
    (*p)++; /* skip the quotes */
    (*n)--;

    while (*n > 0
       &&
       **p != '\n' && **p != quote) {
      *value = **p;
      value++;
      (*p) ++;
      (*n) --;
    }

  } else {
    while (*n > 0
       &&
       **p != ' ' && **p != '\t' && **p != '\r' && **p != '\f' && **p != '\v' && **p != '\n') {
      *value = **p;
      value++;
      (*p) ++;
      (*n) --;
    }
  }

  *value=0;
}

static void read_config_file(dictionary_t dict, char* fname) {
  int f;
  struct stat stat;

  f = open(fname,O_RDONLY);
  if (f == -1) {
    printf("Failed to open %s: %s\n",fname,strerror(errno));
    return;
  }

  if (fstat(f,&stat) == -1) {
    printf("Failed to fstat %s: %s\n",fname,strerror(errno));
    close(f);
    return;
  }

  printf("size of %s is %u bytes\n",fname,stat.st_size);

  char* buffer;
  buffer = mmap(NULL, stat.st_size, PROT_READ , MAP_SHARED, f, 0);
  if (buffer == NULL) {
    printf("Failed to mmap %s: %s\n",fname,strerror(errno));
    close(f);
    return;
  }

  size_t remaining_size = stat.st_size;
  char*  next = buffer;
  char   identifier[256];
  char   value     [256];

  while (remaining_size > 0) {
    skip_whitespace(&next,&remaining_size);
    if (*next == '\n' || *next == '#') goto nextline;
    get_identifier(&next,&remaining_size,identifier);
    if (*next == '\n' || *next == '#') goto nextline;
    skip_whitespace(&next,&remaining_size);
    skip_equals_sign(&next,&remaining_size);
    skip_whitespace(&next,&remaining_size);
    get_value(&next,&remaining_size,value);
    //printf("processing a line... <%s>=<%s> remaining %u\n",identifier,value,remaining_size);
    update_item(dict,identifier,value);
    nextline:
      skip_to_eol    (&next,&remaining_size);
  }

  munmap(buffer, stat.st_size);
  close(f);
}

/*** processing argc, argv ***/

static void process_args(dictionary_t dict,
			 int argc,
			 char* argv[]) {
  int i;
  int argtype[argc];
  char* id;
  char* v;
  char  buf[256];

  argtype[0] = -1; /* argument 0 is the name of the program */

  for (i=1; i<argc; i++) {
    argtype[i] = 0;
    if (strncmp(argv[i],"--",2)==0) {
      argtype[i] = 1;
    } else if (argv[i][0] == '-') {
      argtype[i] = 2;
    } else if (strchr(argv[i],'=')) {
      argtype[i] = 3;
    }
  }

  for (i=1; i<argc; i++) {
    switch (argtype[i]) {
    case 0:
      break;
    case 1:
      id = argv[i] + 2;
      if (i<argc-1 && argtype[i+1]==0) {
        v = argv[i+1];
        i++;
      } else v = 0;
      update_item(dict,id,v);
      break;
    case 2:
      id = argv[i] + 1;
      if (i<argc-1 && argtype[i+1]==0) {
        v = argv[i+1];
        i++;
      } else v = 0;
      update_item_short(dict,*id,v);
      break;
    case 3:
      strcpy(buf,argv[i]);
      buf[ strchr(argv[i],'=') - argv[i] ] = 0;
      id = buf;
      v = argv[i] + (strchr(argv[i],'=') - argv[i] + 1);
      update_item(dict,id,v);
      break;
    default:
      assert(0);
    }
  }

}

/** initialization: process all the config files and the command-line arguments */

dictionary_t configInit(char* fname,
			char* locations[],
			int argc,
			char* argv[]) {
  dictionary_t dict = (dictionary_t) malloc(sizeof(struct dictionary_st));
  dict->items = 0;

  if (fname && locations) {
    int l;
    for (l = 0; locations[l]; l++) {
      char* location = locations[l];
      char  fullname[256];
      char* p = fullname;

      while (*location) {
	if (*location != '$') {
	  *p = *location;
	  p++;
	  location++;
	} else {
	  location++; /* skip the dollar sign */
	  env_substitute(&location, &p);
	}
      }
      if (*location == 0) *p = 0;
      if (*(p-1) != '/') {
	*p = '/';
	p++;
      }

      char* f = fname;
      while (*f) {
	*p = *f;
	p++;
	f++;
      }

      *p = 0;

      //printf("location=[%s]\n",fullname);
      read_config_file(dict,fullname);
    }
  }

  process_args(dict, argc, argv);

  return dict;
}

#ifdef UNIT_TEST
char* locations[] = {
  "/etc",
  "$HOME/.quickwifi/",
  "$XYZ/.xyz/",
  "./",
  0
};

int main(int argc, char** argv) {
  dictionary_t config =
    configInit("quickwifi.conf",
	       locations,
	       argc,
	       argv);
  dump(config);
  printf(">>> %s\n",configGetString(config, "vtrack", 'v', "vtrack-default"));
  printf(">>> %s\n",configGetString(config, "test", 'v', "test-default"));
  printf(">>> %s\n",configGetString(config, "noshort", 0, "noshort-default"));
  printf(">>> %s\n",configGetString(config, "missing", 0, "missing-default"));
  printf(">>> %s\n",configGetString(config, "short", 's', "short-default"));
}
#endif
