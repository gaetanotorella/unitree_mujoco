#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;
    // joystick_type: "script" -> il joystick e' riprodotto da questo file YAML invece che letto
    // da /dev/input/js*. Vedi scripted_joystick.h e tools/run_protocol.py.
    std::string joystick_script;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            joystick_script = cfg["joystick_script"] ? cfg["joystick_script"].as<std::string>() : "";
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

// Pre-scansione di argv per --config/-c PRIMA di load_from_yaml.
// Non si puo' usare boost::program_options qui: helper() scrive direttamente dentro `config`
// tramite po::value<...>(&config.x), quindi deve girare DOPO il caricamento del file, altrimenti
// il file sovrascriverebbe i valori passati da riga di comando. Da cui questa scansione a mano.
//
// PERCHE' ESISTE: la procedura di test automatizzata deve poter usare un config con il joystick
// scriptato senza riscrivere quello di produzione, che e' lo stesso usato per le sessioni manuali.
inline std::string config_override(int argc, char** argv)
{
    for(int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if((a == "--config" || a == "-c") && i + 1 < argc) return argv[i + 1];
        if(a.rfind("--config=", 0) == 0) return a.substr(9);
    }
    return "";
}

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("config,c", po::value<std::string>(), "config.yaml alternativo; gia' letto prima di qui")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}