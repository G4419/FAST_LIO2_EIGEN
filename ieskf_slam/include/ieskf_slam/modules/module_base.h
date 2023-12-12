#ifndef MODULE_BASE_H
#define MODULE_BASE_H

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "colorful_terminal/colorful_terminal.hpp"
namespace IESKFSlam{
    class ModuleBase{
        private:
            YAML::Node config_node;
            std::string name;
            std::shared_ptr<ctl::table_out> table_out_ptr;
        protected:
            /**
            *    @param config_path:配置文件目录
            *    @param prefix:前缀
            *    @param module_name:模块名称
            *   
            */
           ModuleBase(const std::string &config_path, const std::string &prefix, const std::string &module_name = "default"){
                name = module_name;
                table_out_ptr = std::make_shared<ctl::table_out>(name);//"module_name"?
                if(config_path != ""){
                    try
                    {
                        config_node = YAML::LoadFile(config_path);
                    }
                    catch(YAML::Exception& e)
                    {
                        std::cerr <<"error:" <<  e.msg << '\n';
                    }

                    if(prefix!="" && config_node[prefix]) config_node = config_node[prefix];
                }
           }
           template<typename T>
           void readParam(const std::string &key, T &val, T default_val){
                if(config_node[key]){
                    val = config_node[key].as<T>();
                }else{
                    val = default_val;
                }
                table_out_ptr->add_item(key, VAR_NAME(val), val);
           }
           void print_table(){table_out_ptr->make_table_and_out();}


    };
}

#endif