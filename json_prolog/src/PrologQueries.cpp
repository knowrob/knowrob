#include <designators/Designator.h>
#include <designator_integration_msgs/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

#include <rs_queryanswering/RSControledAnalysisEngine.h>
#include <rs_queryanswering/RSProcessManager.h>

#include <ros/package.h>

#include <uima/api.hpp>
#include <SWI-cpp.h>

#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <memory>

using namespace designator_integration;

Designator *req_desig = NULL;

static RSProcessManager *pm;

uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock");


//class RSPMProxy
//{
//  ros::NodeHandle *nh_;
//  ros::ServiceClient client_;
//  RSProcessManager *pm_;
//  std::mutex lock_;
//  std::string ae_;
//  RSPMProxy(std::string ae):ae_(ae)
//  {
//    ros::init(ros::M_string(), std::string("RoboSherlock"));
//    nh_ = new ros::NodeHandle("~");
//    pm_ = new RSProcessManager(false, ".", true, false, nh_);
//  }

//  void run()
//  {

//  }

//  void queryRoboSherlock(Designator d)
//  {
//    designator_integration_msgs::DesignatorCommunication srv;
//    srv.request.request.designator = d.serializeToMessage();
//    {
//      if(client_.call(srv))
//      {
//          m->setUseIdentityResolution(false);
//          pm->init(pipelinePath, "cml");
//      }

//    }
//  }
//};

std::thread thread;

/***************************************************************************
 *                                  ADD DESIGNATOR
 * *************************************************************************/


PREDICATE(install_rs_prologrulescpp,0)
{
  return true;
}

PREDICATE(cpp_add_designator, 2)
{
  std::string desigType((char *)A1);
  Designator *desig = new Designator();
  if(desigType == "object")
  {
    desig->setType(Designator::ACTION);
  }
  std::cerr << "designator type: " << desig->type() << "\n";
  return A2 = static_cast<void *>(desig);
}


//string,Designator,Kvp
PREDICATE(cpp_init_kvp, 3)
{
  void *obj = A1;
  std::string type((char *)A2);
//  std::transform(type.begin(), type.end(), type.begin(), ::toupper);
  Designator *desig = (Designator *)obj;
  KeyValuePair *kvp = desig->addChild(type);
  return A3 = static_cast<void *>(kvp);
}

PREDICATE(cpp_add_kvp, 3)
{
  std::string key = (std::string)A1;
//  std::transform(key.begin(), key.end(), key.begin(), ::toupper);
  std::string value = (std::string)A2;
  void *obj = A3;
  Designator *desig = (Designator *)obj;
  KeyValuePair *kvp = new KeyValuePair(key, value);

  if(desig)
  {
    //std::cerr<<"Adding Kvp: ("<<key<<" : "<<value<<")\n";
    desig->addChild(kvp);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(cpp_print_desig, 1)
{
  void *obj = A1;
  Designator *desig = (Designator *)obj;
  if(desig)
  {
    desig->printDesignator();
    return TRUE;
  }
  else
  {
    std::cerr << "Desigantor object not initialized. Can not print" << std::endl;
    return FALSE;
  }
}

PREDICATE(cpp_init_desig, 1)
{
  if(!req_desig)
  {
    std::cerr << "Initializing designator: " << std::endl;
    req_desig = new designator_integration::Designator();
    designator_integration::KeyValuePair *some_shit =  new designator_integration::KeyValuePair("location", "on table");
    designator_integration::KeyValuePair *links = new designator_integration::KeyValuePair("location");
    links->addChild(some_shit);
    req_desig->addChild(links);
    return A1 = (void *)req_desig;
  }
  else
  {
    std::cerr << "Designator already initialized" << std::endl;
    return FALSE;
  }
}

PREDICATE(cpp_delete_desig, 1)
{
  void *obj = A1;
  Designator *desig = (Designator *)obj;
  delete desig;
  return TRUE;
}

/***************************************************************************
 *                  Manipulate RS instances/pipelines
 * *************************************************************************/

/**
 * @brief initialize the AnalysisEngine object
 */
PREDICATE(cpp_init_rs, 2)
{
  if(!pm)
  {
    ros::init(ros::M_string(), std::string("RoboSherlock"));
    ros::NodeHandle nh("~");
    dlopen("libpython2.7.so", RTLD_LAZY | RTLD_GLOBAL);
    std::string pipelineName((char *)A1);
    std::string pipelinePath;
    rs::common::getAEPaths(pipelineName, pipelinePath);
    std::vector<std::string> lowLvlPipeline;
    lowLvlPipeline.push_back("CollectionReader");
    std::string configPath =
        ros::package::getPath("rs_queryanswering").append(std::string("/config/config.yaml"));

    std::cerr<<"Path to config file: "<<configPath<<std::endl;

    if(!pipelinePath.empty())
    {
      bool waitForService = false;
      pm = new RSProcessManager(false, waitForService, false, nh);
//      pm->setLowLvlPipeline(lowLvlPipeline);
      pm->setUseIdentityResolution(false);
      pm->setUseJsonPrologInterface(true);
      pm->init(pipelinePath, configPath);
      thread = std::thread(&RSProcessManager::run, &(*pm));
      return A2 = (void *)pm;
    }
  }
  return FALSE;
}

PREDICATE(cpp_rs_pause, 1)
{
  if(pm)
  {
    pm->pause();
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(cpp_stop_rs, 1)
{
  if(pm)
  {
    pm->stop();
    delete pm;
    pm = NULL;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


/**
 * change AE file that is loaded, enables changing the context
 * e.g. from kitchen to cehmlab, where we need different parameterizations
 * */

PREDICATE(rs_render_view, 1)
{
  if(pm)
  {
    std::string objectName((char *)A1);
    pm->renderOffscreen(objectName);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

PREDICATE(change_context, 1)
{
  if(pm)
  {
    std::string pipelineName((char *)A1);
    std::string newPipelinePath;
    rs::common::getAEPaths(pipelineName, newPipelinePath);

    if(!newPipelinePath.empty())
    {
      std::string currentPipeline = pm->getEngineName();
      if(currentPipeline != newPipelinePath)
      {
        pm->resetAE(newPipelinePath);
        return TRUE;
      }
      else
      {
        outInfo("Already set to: " << newPipelinePath);
        return FALSE;
      }
    }
    else
    {
      return FALSE;
    }
  }
  else
  {
    return FALSE;
  }
}

/**
 * @brief run the process function once
 */
PREDICATE(cpp_process_once, 1)
{
  if(pm)
  {
    void *myobj = A1;
    Designator *desig  = (Designator *)myobj;
    std::vector<designator_integration::Designator> resp;
    pm->handleQuery(desig, resp);
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


PREDICATE(delete_desig, 1)
{
  if(req_desig)
  {
    delete req_desig;
    req_desig = NULL;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}


PREDICATE(write_list, 1)
{
  PlTail tail(A1);
  PlTerm e;

  while(tail.next(e))
  {
    std::cout << (char *)e << std::endl;
  }

  return TRUE;
}

