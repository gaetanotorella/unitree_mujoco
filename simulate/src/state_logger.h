// Logger ground-truth per il simulatore MuJoCo C++.
// Scrive lo stato VERO della base (posa + velocita') letto da mjData
// in $HOME/sim_logs/sim_<YYYY-MM-DD_HH-MM-SS>/ground_truth.csv.
// La stessa cartella viene poi usata dal controller di deploy per
// obs.csv / commands.csv / errors.csv (trova la sim_* piu' recente).
// Allineamento cross-processo tramite wall_t (orologio di sistema, stesso PC).
#pragma once

#include <mujoco/mujoco.h>
#include <fstream>
#include <filesystem>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstdlib>

namespace sim_logger
{

inline std::string g_run_dir; // cartella del run corrente (creata al primo log)

inline std::string make_timestamp()
{
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

inline double wall_seconds()
{
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

inline std::string base_log_dir()
{
    const char *home = std::getenv("HOME");
    return (home ? std::string(home) : std::string(".")) + "/sim_logs";
}

// Da chiamare a ogni passo fisico (con downsampling interno a ~100 Hz).
// qpos: [0..2]=x,y,z base, [3..6]=quat(w,x,y,z).
// qvel: [0..2]=vel lineare (world), [3..5]=vel angolare (body frame).
inline void log_ground_truth(const mjData *d)
{
    static std::ofstream out;
    static double last_t = -1e9;
    static bool init = false;

    if (!init)
    {
        init = true;
        g_run_dir = base_log_dir() + "/sim_" + make_timestamp();
        std::error_code ec;
        std::filesystem::create_directories(g_run_dir, ec);
        out.open(g_run_dir + "/ground_truth.csv");
        out << "wall_t,sim_t,base_x,base_y,base_z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz\n";
        std::cout << "[sim_logger] ground_truth -> " << g_run_dir << std::endl;
    }

    // downsampling a ~100 Hz (fisica a 200 Hz)
    if (d->time - last_t < 0.01 - 1e-9)
        return;
    last_t = d->time;

    out << std::setprecision(9)
        << wall_seconds() << "," << d->time
        << "," << d->qpos[0] << "," << d->qpos[1] << "," << d->qpos[2]
        << "," << d->qpos[3] << "," << d->qpos[4] << "," << d->qpos[5] << "," << d->qpos[6]
        << "," << d->qvel[0] << "," << d->qvel[1] << "," << d->qvel[2]
        << "," << d->qvel[3] << "," << d->qvel[4] << "," << d->qvel[5] << "\n";
    out.flush();
}

} // namespace sim_logger
