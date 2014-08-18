#ifndef _LAMA_INTERFACES_LAMA_UTILS_H_
#define _LAMA_INTERFACES_LAMA_UTILS_H_

#include <string>

#include <sqlite3.h>

int getDescriptorId (sqlite3* db, int object_id, std::string interface_name);

#endif // _LAMA_INTERFACES_LAMA_UTILS_H_
