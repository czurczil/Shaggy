#ifndef DB_H_INCLUDED
#define DB_H_INCLUDED

#include <iostream>
#include <string>
#include <string.h>
#include "robot.h"

#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>

using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::open_document;
using namespace std;

class DB
{

public:

    int WriteState(RobotState *state);
    int WriteStateAsMap(RobotState *state);
    void ClearTerrainMap();
      string ReadCommand();
    int Open();
   int Close();


private:
    mongocxx::client *client = NULL;

};

#endif // DB_H_INCLUDED
