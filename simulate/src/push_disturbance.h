#pragma once
//
// Spinte MANUALI per il test di robustezza: forze e coppie applicate a un corpo del robot premendo
// un TASTO nella finestra di MuJoCo, mentre si guida il robot col joystick fisico. Ogni tasto e' un
// preset (corpo, forza/coppia, durata, punto, riferimento) definito in un file YAML indicato da
// `disturbance_script:` in config.yaml. Nessun legame con il banco automatico o i protocolli.
//
//   # disturbances.yaml
//   presets:
//     - {key: "1", body: torso_link, force: [100, 0, 0],  dur: 0.2}   # 20 N.s avanti
//     - {key: "2", body: torso_link, force: [-100, 0, 0], dur: 0.2}   # 20 N.s indietro
//     - {key: "3", body: torso_link, force: [0, 100, 0],  dur: 0.2}   # 20 N.s a sinistra
//     - {key: "4", body: torso_link, force: [0, -100, 0], dur: 0.2}   # 20 N.s a destra
//     - {key: "5", body: pelvis,     torque: [0, 0, 30],  dur: 0.2}   # coppia di imbardata
//     - {key: "6", body: torso_link, force: [150, 0, 0],  dur: 0.2, point: [0, 0, 0.3]}  # 30 cm sopra l'origine
//
//   force in N, torque in N.m, dur in secondi (0 = un solo passo fisico, 5 ms), point in metri nel
//   riferimento del corpo (default: baricentro; un punto diverso aggiunge la coppia equivalente
//   (P - CoM) x F). frame: "body" (DEFAULT: "avanti" e' l'avanti del robot, anche se ha girato) o
//   "world". I tasti sono caratteri singoli; 7, 8, 9 sono gia' della corda elastica.
//
// Meccanismo: mjData::xfrc_applied[6*body] = forza (3, mondo) + coppia (3, mondo) al baricentro.
// ATTENZIONE: Simulate::Sync() (simulate.cc, thread di rendering, ~60 Hz) fa mju_zero su TUTTO
// xfrc_applied a ogni frame, per ripulire le perturbazioni del mouse: qualunque cosa si voglia
// applicare va RISCRITTA in valore assoluto a ogni passo fisico, come fa la banda elastica. Se il
// corpo e' quello della banda, il valore appena scritto dalla banda (che agisce prima, nello
// stesso passo) viene conservato e la spinta gli si somma.
//
// Ogni spinta applicata e' loggata in disturbance_events.csv nella cartella del run (start/end,
// tempo di simulazione, corpo, forza, coppia, punto): tools/report_pushes.py la legge insieme a
// ground_truth.csv e obs.csv per misurare la risposta (Dv, picco di tilt, spostamento, assestamento).
//
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <map>
#include <mutex>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace pushes
{

class PushDisturbances
{
public:
    struct Event
    {
        double t0 = 0.0, t1 = 0.0;
        int body = -1;
        std::string body_name;
        std::array<double, 3> force{0, 0, 0}, torque{0, 0, 0}, point{0, 0, 0};
        bool body_frame = false, has_point = false;
        bool started = false, finished = false;
    };

    // Carica i preset da `path` (chiave `presets`). Ritorna false se manca o e' vuota.
    bool load(const std::string & path, const mjModel * m)
    {
        YAML::Node cfg;
        try { cfg = YAML::LoadFile(path); }
        catch (const std::exception & ex) {
            std::cerr << "Spinte: impossibile leggere " << path << ": " << ex.what() << std::endl;
            return false;
        }
        if (!cfg["presets"]) { std::cerr << "Spinte: nessuna chiave `presets` in " << path << std::endl; return false; }

        std::cout << "Spinte manuali da " << path << " -- tasti nella finestra MuJoCo:" << std::endl;
        for (const auto & n : cfg["presets"])
        {
            Event e;
            e.t1 = (n["dur"] ? n["dur"].as<double>() : 0.0);   // qui t1 = durata; t0 si fissa al trigger
            e.body_name = n["body"] ? n["body"].as<std::string>() : "torso_link";
            e.body = mj_name2id(m, mjOBJ_BODY, e.body_name.c_str());
            if (e.body < 0) {
                std::cerr << "Spinte: corpo '" << e.body_name << "' non esiste nel modello" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            if (n["force"])  for (int i = 0; i < 3; i++) e.force[i]  = n["force"][i].as<double>();
            if (n["torque"]) for (int i = 0; i < 3; i++) e.torque[i] = n["torque"][i].as<double>();
            if (n["point"])  { e.has_point = true; for (int i = 0; i < 3; i++) e.point[i] = n["point"][i].as<double>(); }
            e.body_frame = !(n["frame"] && n["frame"].as<std::string>() == "world");   // default: body
            const std::string key = n["key"].as<std::string>();
            if (key.size() != 1) { std::cerr << "Spinte: `key` deve essere un carattere singolo (" << key << ")" << std::endl; std::exit(EXIT_FAILURE); }
            presets_[std::toupper(key[0])] = e;
            std::cout << "   [" << key << "]  " << e.body_name
                      << "  F=(" << e.force[0] << "," << e.force[1] << "," << e.force[2] << ") N"
                      << "  T=(" << e.torque[0] << "," << e.torque[1] << "," << e.torque[2] << ") N.m"
                      << "  " << e.t1 << " s  " << (e.body_frame ? "body" : "world")
                      << "  -> impulso " << std::sqrt(e.force[0]*e.force[0]+e.force[1]*e.force[1]+e.force[2]*e.force[2]) * std::max(e.t1, 0.005) << " N.s"
                      << std::endl;
        }
        return !presets_.empty();
    }

    bool empty() const { return presets_.empty(); }

    // Da chiamare dal callback della tastiera: se `key` (GLFW, = codice ASCII maiuscolo) e' un
    // preset, programma la spinta A PARTIRE DA ORA (tempo di simulazione corrente).
    void trigger(int key, double now)
    {
        auto it = presets_.find(key);
        if (it == presets_.end()) return;
        Event e = it->second;
        e.t0 = now; e.t1 = now + it->second.t1;
        {   // trigger() gira nel thread della finestra, apply() in quello della fisica
            std::lock_guard<std::mutex> lk(mtx_);
            pending_.push_back(e);
        }
        std::cout << "[spinta] tasto " << char(key) << " a t=" << now << " s: " << e.body_name << std::endl;
    }

    // Da chiamare a OGNI passo fisico, prima di mj_step, DOPO la banda elastica.
    // band_body: corpo su cui agisce la banda (-1 se nessuno); band_on: se in questo passo la
    // banda ha appena scritto la sua forza su xfrc_applied[band_body].
    void apply(const mjModel * m, mjData * d, const std::string & log_dir, int band_body = -1, bool band_on = false)
    {
        std::map<int, std::array<double, 6>> acc_by_body;
        {   // prendi in carico le spinte richieste da tastiera
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto & e : pending_) events_.push_back(e);
            pending_.clear();
        }

        const double t = d->time;
        for (auto & e : events_)
        {
            const bool active = (t >= e.t0) && (t < e.t1 || (!e.started && e.t1 <= e.t0));  // dur 0 -> un passo
            if (!active) {
                if (e.started && !e.finished && t >= e.t1) { e.finished = true; log(e, "end", t, log_dir); }
                continue;
            }
            if (!e.started) { e.started = true; log(e, "start", t, log_dir); }
            if (e.t1 <= e.t0) e.finished = true;  // impulsivo: si spegne da solo al passo dopo

            // forza e coppia nel riferimento MONDO
            std::array<double, 3> F = e.force, T = e.torque;
            if (e.body_frame) { F = rot(d->xmat + 9 * e.body, e.force); T = rot(d->xmat + 9 * e.body, e.torque); }

            // punto di applicazione diverso dal baricentro -> coppia equivalente (P - CoM) x F
            if (e.has_point)
            {
                const std::array<double, 3> pl = rot(d->xmat + 9 * e.body, e.point);
                std::array<double, 3> r;
                for (int i = 0; i < 3; i++) r[i] = (d->xpos[3 * e.body + i] + pl[i]) - d->xipos[3 * e.body + i];
                T[0] += r[1] * F[2] - r[2] * F[1];
                T[1] += r[2] * F[0] - r[0] * F[2];
                T[2] += r[0] * F[1] - r[1] * F[0];
            }

            auto & acc = acc_by_body[e.body];
            for (int i = 0; i < 3; i++) { acc[i] += F[i]; acc[3 + i] += T[i]; }
        }

        // scrittura ASSOLUTA per ogni corpo toccato in questo passo
        for (auto & kv : acc_by_body)
        {
            mjtNum * x = d->xfrc_applied + 6 * kv.first;
            const bool keep_band = (kv.first == band_body) && band_on;
            for (int i = 0; i < 6; i++)
                if (!(keep_band && i < 3)) x[i] = 0.0;      // la forza della banda resta, il resto riparte da 0
            for (int i = 0; i < 6; i++) x[i] += kv.second[i];
        }
        // i corpi che avevano un disturbo al passo scorso e ora no: Sync() li azzera a ogni frame,
        // ma fra due frame ci sono piu' passi fisici -> azzeriamo noi subito, per non lasciare
        // la forza appesa un passo in piu'.
        for (int b : touched_)
            if (!acc_by_body.count(b) && b != band_body)
                for (int i = 0; i < 6; i++) d->xfrc_applied[6 * b + i] = 0.0;
        touched_.clear();
        for (auto & kv : acc_by_body) touched_.push_back(kv.first);
    }

private:
    static std::array<double, 3> rot(const mjtNum * R, const std::array<double, 3> & v)
    {
        return {R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
                R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
                R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
    }

    void log(const Event & e, const char * what, double t, const std::string & dir)
    {
        if (dir.empty()) return;
        if (!log_.is_open()) {
            log_.open(dir + "/disturbance_events.csv");
            log_ << "sim_t,event,body,fx,fy,fz,tx,ty,tz,frame,px,py,pz,t_planned,dur_planned\n";
        }
        log_ << t << "," << what << "," << e.body_name
             << "," << e.force[0] << "," << e.force[1] << "," << e.force[2]
             << "," << e.torque[0] << "," << e.torque[1] << "," << e.torque[2]
             << "," << (e.body_frame ? "body" : "world")
             << "," << e.point[0] << "," << e.point[1] << "," << e.point[2]
             << "," << e.t0 << "," << (e.t1 - e.t0) << "\n";
        log_.flush();
    }

    std::map<int, Event> presets_;
    std::vector<Event> events_, pending_;
    std::mutex mtx_;
    std::vector<int> touched_;
    std::ofstream log_;
};

inline PushDisturbances g_pushes;

}  // namespace pushes
