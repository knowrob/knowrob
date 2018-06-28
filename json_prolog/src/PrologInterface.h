#ifndef JSONPROLOGINTERFACE_H
#define JSONPROLOGINTERFACE_H

//boost
#include <boost/algorithm/string.hpp>

//ros
#include <ros/package.h>

//ros packages
#include <designators/Designator.h>

//robosherlock
#include <rs/utils/output.h>
#include <rs_queryanswering/KRDefinitions.h>

//SWI Prolog
#include <SWI-cpp.h>

//json_prolog interface
#include <json_prolog/prolog.h>

//STD
#include <memory>

//wrapper class for Prolog Engine based on SWI-C++
class PrologInterface
{

  typedef std::shared_ptr<PlEngine> PlEnginePtr;
  PlEnginePtr engine;
  bool useJsonProlog;

public:
  PrologInterface(bool json_prolog = false);
  ~PrologInterface()
  {
  }


  /*brief
   * initialize the necessary knowrob packages
   */
  void init();

  /*
   * in: vector of keys extracted from query
   * out: vector of annotator names forming the pipeline
   */
  bool planPipelineQuery(const std::vector<std::string> &keys,
                         std::vector<std::string> &pipeline)
  {
    if(!useJsonProlog)
    {
      PlTermv av(2);
      PlTail l(av[0]);
      for(auto key : keys)
      {
        l.append(key.c_str());
      }
      l.close();
      PlQuery q("build_single_pipeline_from_predicates", av);
      std::string prefix("http://knowrob.org/kb/rs_components.owl#");
      while(q.next_solution())
      {
        //      std::cerr<<(char*)av[1]<<std::endl;
        PlTail res(av[1]);//result is a list
        PlTerm e;//elements of that list
        while(res.next(e))
        {
          std::string element((char *)e);
          element.erase(0, prefix.length());
          pipeline.push_back(element);
        }
      }
      return true;
    }
    else
    {
      outInfo("Calling Json Prolog");
      json_prolog::Prolog pl;
      json_prolog::PrologQueryProxy bdgs = pl.query(buildPrologQueryFromKeys(keys));
      if(bdgs.begin() == bdgs.end())
      {
        outInfo("Can't find solution for pipeline planning");
        return false; // Indicate failure
      }
      for(auto bdg : bdgs)
      {
        pipeline = createPipelineFromPrologResult(bdg["A"].toString());
      }
      return true;
    }
  }

  /*brief
   * ask prolog if child is of type parent
   * */
  bool q_subClassOf(std::string child, std::string parent)
  {
    if(!useJsonProlog)
    {
      PlTermv av(2);
      av[0] =  rs_queryanswering::makeUri(rs_queryanswering::krNameMapping[child]).c_str();
      av[1] =  rs_queryanswering::makeUri(rs_queryanswering::krNameMapping[parent]).c_str();
      try
      {
        if(PlCall("owl_subclass_of", av))
        {
          outInfo(child << " is subclass of " << parent);
          return true;
        }
        else
        {
          outInfo(child << " is NOT subclass of " << parent);
          return false;
        }
      }
      catch(PlException &ex)
      {
        outError((char *)ex);
        return false;
      }
      return false;
    }
    else
    {
      std::stringstream prologQuery;
      prologQuery << "owl_subclass_of(" << rs_queryanswering::krNameMapping[child] << "," << rs_queryanswering::krNameMapping[parent] << ").";
      outInfo("Asking Query: " << prologQuery.str());
      json_prolog::Prolog pl;
      json_prolog::PrologQueryProxy bdgs = pl.query(prologQuery.str());

      if(bdgs.begin() != bdgs.end())
      {
        outInfo(rs_queryanswering::krNameMapping[child] << " IS " << rs_queryanswering::krNameMapping[parent]);
        return true;
      }
      return false;
    }
  }

  /*brief
   * check for a class property
   * */
  bool q_classProperty(std::string className, std::string property, std::string value)
  {
    if(!useJsonProlog)
    {
      PlTermv av(3);
      av[0] =  rs_queryanswering::makeUri(rs_queryanswering::krNameMapping[className]).c_str();
      av[1] =  rs_queryanswering::makeUri(property).c_str();
      av[2] =  rs_queryanswering::makeUri(value).c_str();
      try
      {
        if(PlCall("class_properties", av))
        {
          outInfo(className << " " << property << " " << value);
          return true;
        }
        else
        {
          outInfo(className << " has NO " << property << " " << value);
          return false;
        }
      }
      catch(PlException &ex)
      {
        std::cerr << (char *)ex << std::endl;
        return false;
      }
      return false;
    }
    else
    {
      outInfo("Calling Json Prolog");
    }
    return false;
  }


  std::string buildPrologQueryFromKeys(const std::vector<std::string> &keys);

  /*brief
   *extract the keys that serve as input for pipeline planning
   * in query as a designator
   * out vector of keys
   * */
  bool extractQueryKeysFromDesignator(designator_integration::Designator *desig,
                                      std::vector<std::string> &keys);

  /*brief
   * in desig: query as designator
   * out prologQuery: the Prolog Query as a string
   * returns true or false /success or fail
   * */
  bool buildPrologQueryFromDesignator(designator_integration::Designator *desig,
                                      std::string &prologQuery);

  /*brief
   * Create a vector of Annotator Names from the result of the knowrob_rs library.
   * This vector can be used as input for RSControledAnalysisEngine::setNextPipelineOrder
   */
  std::vector<std::string> createPipelineFromPrologResult(std::string result);

};

#endif //JSONPROLOGINTERFACE_H
