#include <string>

#include <ros/ros.h>

#include <lama_interfaces/CLMIPolygon.h>
#include <lama_interfaces/lama_utils.h>
#include <geometry_msgs/Point32.h>

CLMIPolygon::CLMIPolygon(std::string map_file)
{
  map = map_file;
}

bool CLMIPolygon::getter(lama_interfaces::lmi_polygon_get::Request& req, lama_interfaces::lmi_polygon_get::Response& res)
{
  bool ret = false;
  sqlite3_stmt *statement;
  int rowCount = 0;
  int result = 0;
  if (req.id.descriptor_id < 0)
  { 
    req.id.descriptor_id = getDescriptorId(db,req.id.object_id, req.id.interface_name);
    ROS_DEBUG("POLYGON get descriptor %i", req.id.descriptor_id);
  }

  if (sqlite3_prepare_v2(db, "select x,y, sequence_number  from polygon_points where id = ? order by sequence_number asc ;", -1, &statement, 0) == SQLITE_OK)
  {
    sqlite3_bind_int(statement, 1, req.id.descriptor_id);

    while ((result = sqlite3_step(statement)) == SQLITE_ROW)
    {
      rowCount ++;
      geometry_msgs::Point32 p;
      p.x = sqlite3_column_double(statement, 0);
      p.y = sqlite3_column_double(statement, 1);
      res.descriptor.points.push_back(p);
      ROS_DEBUG("polygon %i [%f %f]", sqlite3_column_int(statement, 2), p.x, p.y);
    } 
    ret = rowCount;

  }
  else
  {
    ROS_ERROR("POLYGON cannot prepare sqlite3 statement");
    ret = false;
  }
  sqlite3_finalize(statement);
  return ret;
};

bool CLMIPolygon::setter(lama_interfaces::lmi_polygon_set::Request& req, lama_interfaces::lmi_polygon_set::Response& res)
{
  bool ret = false;
  sqlite3_stmt *statement;
  sqlite3_stmt *statement_max_id;
  int result = 0;
  int actual_id = 0;
  int err;

  //ros::param::param<int>("lama_actual_view", actual_id, 1);
  ROS_DEBUG("polygon setter");

  if ((err = sqlite3_prepare_v2(db, "select max(id) from polygon_points;", -1, &statement_max_id, 0)) == SQLITE_OK)
  {
    result = sqlite3_step(statement_max_id);
    if (result == SQLITE_ROW)
    {
      actual_id = sqlite3_column_int(statement_max_id, 0) + 1;
    }
    else
    {
      ROS_ERROR ("failed get max id  %i ", result);
    }
  }
  else
  {
    ROS_ERROR("cannot prepare sqlite3 statement select max: %i", err);
    ret  =  false;
  }
  sqlite3_finalize(statement_max_id);

  sqlite3_exec(db, "BEGIN IMMEDIATE TRANSACTION;", 0, 0, 0);
  if ((err = sqlite3_prepare_v2(db, "insert into polygon_points (id, sequence_number, x,y) values (?, ?, ?, ?);", -1, &statement, 0)) == SQLITE_OK)
  {
    for (uint i =0; i < req.descriptor.points.size(); i++)
    {
      sqlite3_bind_int(statement,1,actual_id);
      sqlite3_bind_int(statement,2,i);
      sqlite3_bind_double(statement,3,req.descriptor.points[i].x);
      sqlite3_bind_double(statement,4,req.descriptor.points[i].y);
      result = sqlite3_step(statement);
      if(result == SQLITE_DONE)
      { 
      }
      else
      {
        ROS_ERROR("insert %i point  fails %i ", i, result);
      }
      sqlite3_clear_bindings(statement);
      if (sqlite3_reset(statement)== SQLITE_OK)
      {
      }
      else
      {
        ROS_ERROR("problem with reset");
      };
    }
    if (result == SQLITE_DONE)
    {
      res.id.descriptor_id = actual_id;
      //sqlite3_exec(db, "COMMIT", 0, 0, 0);
      ROS_DEBUG("return true");
      ret = true;
    }
    else
    {
      ROS_ERROR("failed inserti  polygon  %i", result);
      ret = false;
    }
  }
  else
  {
    // sqlite3_exec(db, "ROLBACK", 0, 0, 0);
    ROS_ERROR("cannot prepare sqlite3 statement insert int: %i", err);
    ret = false;
  }

  sqlite3_finalize(statement);
  sqlite3_exec(db, "END;", 0, 0, 0);
  ROS_DEBUG("polygon setter finalize");
  return ret;
};


bool CLMIPolygon::connect()
{
  int rc;

  rc = sqlite3_open(map.c_str(), &db);
  if (rc)
  {
    ROS_ERROR("cannot open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db);
    return false;
  }
  sqlite3_busy_timeout(db, 1000);
  /*   rc = sqlite3_exec(db, "create table if not exists polygon ( id int, view int); ", 0, 0, 0);
       if( rc ){
       ROS_ERROR("Can't create table: %s\n", sqlite3_errmsg(db));
       sqlite3_close(db);
       return false;
       }
       */
  rc = sqlite3_exec(db, "create table if not exists polygon_points (id int, sequence_number int, x double, y double); ", 0, 0, 0);
  if (rc)
  {
    ROS_ERROR("cannot create table: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db);
    return false;
  }
  return true;
}
