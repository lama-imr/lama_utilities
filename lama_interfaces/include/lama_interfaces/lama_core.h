#ifndef _LAMA_INTERFACES_LAMA_CORE_H_
#define _LAMA_INTERFACES_LAMA_CORE_H_

#include <string>

class LamaCore
{
  public:

    LamaCore();
    ~LamaCore() {};

    bool assign(std::string vertex_name, int descriptor_id, std::string interface_name);
    bool assign(int vertex_id, int descriptor_id, std::string interface_name);
    int pushVertex(std::string vertex_name);
    std::string getVertexName(int vertex_id);
};

#endif // _LAMA_INTERFACES_LAMA_CORE_H_
