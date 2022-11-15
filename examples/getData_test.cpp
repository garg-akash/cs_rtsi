/*
Author: Elite_akashgarg
CreateDate: 2022-11-09
LastEdited: 2022-11-14
Description: example script to test output subscription
*/
#include <cs_rtsi/rtsi.h>
#include <cs_rtsi/robot_state.h>
#include <cs_rtsi/rtsi_receive_interface.h>
#include <thread>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <boost/variant.hpp>

using rtsi_type_variant_ = boost::variant<bool, uint8_t, uint32_t, uint64_t, int32_t, double, std::vector<double>,
    											 std::vector<int32_t>>;

const std::string hostip = "192.168.133.129";
const std::string recipe_file = "../examples/recipe.json";

int main(int argc, char const *argv[])
{
	double frequency = 1;
	std::ifstream recipes(recipe_file, std::ifstream::binary);
	Json::Value o_recipes;
	recipes >> o_recipes;
	std::vector<std::string> variables;
	for(auto i : o_recipes["output"])
		variables.push_back(i.asString());

	bool verbose = true;
	RTSIReceiveInterface rtsi_receive(hostip, frequency, variables, verbose);
	int steps = 100;
	while(steps--)
	{
		std::cout << "\nStep : " << steps << "\n";
		for(auto v : variables)
		{
			if(rtsi_receive.robot_state_->state_types_.find(v) == rtsi_receive.robot_state_->state_types_.end())
			{
				std::cout << v << " is not a valid output subscription\n";
				continue;
			}
			rtsi_type_variant_ entry = rtsi_receive.robot_state_->state_types_[v];
			if (entry.type() == typeid(double))
			{		
				double parsed_data;
				if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
					std::cout << "\n" << v << " :\n" << parsed_data << "\n";
				else
					throw std::runtime_error("unable to get state data for " + v);
			}
			else if (entry.type() == typeid(std::vector<double>))
      {
      	std::vector<double> parsed_data;
      	if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
      	{
      		std::cout << "\n" << v << " :\n";
      		for(auto i : parsed_data)
      			std::cout << i << "\t";
      		std::cout << "\n";
      	}
      	else
					throw std::runtime_error("unable to get state data for " + v);
      }
	    else if (entry.type() == typeid(bool))
			{		
				bool parsed_data;
				if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
					std::cout << "\n" << v << " :\n" << parsed_data << "\n";
				else
					throw std::runtime_error("unable to get state data for " + v);
			}
			else if (entry.type() == typeid(int32_t))
			{		
				std::int32_t parsed_data;
				if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
					std::cout << "\n" << v << " :\n" << parsed_data << "\n";
				else
					throw std::runtime_error("unable to get state data for " + v);
			}
			else if (entry.type() == typeid(uint8_t))
			{		
				std::uint8_t parsed_data;
				if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
					std::cout << "\n" << v << " :\n" << +parsed_data << "\n";
				else
					throw std::runtime_error("unable to get state data for " + v);
			}
			else if (entry.type() == typeid(uint32_t))
			{		
				std::uint32_t parsed_data;
				if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
					std::cout << "\n" << v << " :\n" << parsed_data << "\n";
				else
					throw std::runtime_error("unable to get state data for " + v);
			}
	    else if (entry.type() == typeid(uint64_t))
			{		
				std::uint64_t parsed_data;
				if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
					std::cout << "\n" << v << " :\n" << parsed_data << "\n";
				else
					throw std::runtime_error("unable to get state data for " + v);
			}
			else if (entry.type() == typeid(std::vector<std::int32_t>))
      {
	    	std::vector<std::int32_t> parsed_data;
	    	if(rtsi_receive.robot_state_->getStateData(v, parsed_data))
	    	{
	    		std::cout << "\n" << v << " :\n";
	    		for(auto i : parsed_data)
	    			std::cout << i << "\t";
	    		std::cout << "\n";
	    	}
	    	else
					throw std::runtime_error("unable to get state data for " + v);
      }
      else
      	throw std::runtime_error("unknown type of the variable " + v);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	return 0;
}