#include "DB.h"
#include <cstdint>
#include <iostream>
#include <vector>
//#include <time>
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

int DB::WriteState(RobotState *state)
{
mongocxx::database db = (*client)["kudlaty"];
mongocxx::collection coll = db["robotState"];

    auto builder = bsoncxx::builder::stream::document{};
//float angle = roundf(state->angle,1);

bsoncxx::document::value doc_value = builder
  <<"index" << (int) state->cycleNumber
  << "ax" << state->ax
  << "ay" << state->ay
  << "az" << state->az
  //<< "angle" << angle
  << "angle" << state->angle
  << "distFront" << state->distFront
  << "distBack" << state->distBack
  << "distLeft" << state->distLeft
  << "distRight" << state->distRight
  << "cycleMls" << state->cycleMillis
  << "whRPWM" << state->wheelRightPWM
  << "whLPWM" << state->wheelLeftPWM
  << "tstampSys" << (int)state->sysTimestamp
  << "tstampMls" << (int)state->millisTimestamp
  << "tstampMcs" << (int)state->microsTimestamp
  << bsoncxx::builder::stream::finalize;
//auto v = builder.view();
auto result = coll.insert_one(doc_value.view());

    return 0;
}

void DB::ClearTerrainMap()
{
mongocxx::database db = (*client)["kudlaty"];
mongocxx::collection collCmd = db["terrainMap"];
collCmd.drop();
}

int DB::WriteStateAsMap(RobotState *state)
{
mongocxx::database db = (*client)["kudlaty"];
mongocxx::collection coll = db["terrainMap"];

    auto builder = bsoncxx::builder::stream::document{};
int angle = (int)roundf(state->angle);
bsoncxx::document::value doc_value = builder

  << "angle" << angle
  << "distFront" << state->distFront
  << "distBack" << state->distBack
  << "distLeft" << state->distLeft
  << "distRight" << state->distRight
  << bsoncxx::builder::stream::finalize;
//auto v = builder.view();
 mongocxx::stdx::optional< mongocxx::result::replace_one> r = coll.replace_one(document{}<<"angle"<<angle<<finalize, doc_value.view());
if(r && r.value().matched_count() == 0)
{
auto resulti = coll.insert_one(doc_value.view());

}
    return 0;
}

string DB::ReadCommand()
{
    string result = "";
    mongocxx::database db = (*client)["kudlaty"];
    mongocxx::collection coll = db["robotCommand"];
    mongocxx::stdx::optional<bsoncxx::document::value>  valRes;
   // auto  tmp = coll.find_one_and_update(document {}<<"rd"<<0<<finalize, document{}<<"rd"<<1<<finalize);
    valRes = coll.find_one_and_update(document {}<<"rd"<<0<<finalize, document{}<<"rd"<<1<<finalize);
 //   valRes = std::move(tmp);
if(valRes) {
auto r = valRes.value().view()["cmd"];
mongocxx::stdx::string_view v = r.get_utf8().value;
result = v.to_string();
cout<<result<<endl;
//result= bsoncxx::to_string(r);
}
    return result;
}
int DB::Open()
{
mongocxx::uri uri("mongodb://localhost:27017");
client= new mongocxx::client(uri);
mongocxx::database db = (*client)["kudlaty"];
mongocxx::collection coll = db["robotState"];
coll.drop();
mongocxx::collection collCmd = db["robotCommand"];
collCmd.drop();

auto index_spec = document{}<<"index"<<-1<<finalize;
coll.create_index( std::move(index_spec));
    return 0;
}

int DB::Close()
{
    if(client != NULL) delete client ;
    client = NULL;
    return 0;
}
