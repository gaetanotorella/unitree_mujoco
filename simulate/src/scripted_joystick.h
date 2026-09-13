#pragma once
//
// Joystick SCRIPTATO: riproduce un protocollo di test contro il TEMPO DI SIMULAZIONE.
//
// PERCHE' QUI E NON VIA DDS: il controller di deploy legge il joystick da
// `lowstate->joystick`, cioe' dal campo wireless_remote del messaggio LowState, che e' pubblicato
// dal simulatore. Il topic WirelessController esiste ma il controller non lo usa. Quindi l'unico
// punto di iniezione possibile e' dentro il bridge MuJoCo: un publisher esterno non puo'
// sovrascrivere un campo di un messaggio che pubblica qualcun altro.
//
// PERCHE' CONTRO IL TEMPO DI SIMULAZIONE e non l'orologio di parete: due esecuzioni dello stesso
// protocollo su macchine diverse (o con carico diverso) devono comandare le STESSE cose agli
// STESSI istanti fisici. Il tempo di simulazione e' anche l'asse di ground_truth.csv.
// `run_protocol.py` verifica a posteriori che il fattore di real-time sia ~1, perche' il
// controller integra invece sul suo orologio a 50 Hz: se MuJoCo rallenta i due divergono.
//
// Semantica del protocollo: ogni voce della timeline ha un istante `t` e uno stato; lo stato e'
// APPICCICOSO (vale finche' non lo cambia una voce successiva). Cosi' il file resta corto e
// leggibile, e aggiungere una manovra in mezzo non richiede di riscrivere tutto il resto.
//
#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace scripted
{
// Pubblicati dal loop fisico di main.cc a ogni mj_step.
inline std::atomic<double> g_sim_time{0.0};
// < 0 = nessun rilascio automatico della banda elastica (comportamento manuale, tasto 9).
inline std::atomic<double> g_band_release_at{-1.0};
// Alzato quando il protocollo ha superato `stop_at`: l'orchestratore lo usa come segnale di fine.
inline std::atomic<bool> g_finished{false};
}  // namespace scripted


class ScriptedJoystick : public unitree::common::UnitreeJoystick
{
public:
    struct Entry
    {
        double t = 0.0;
        std::string action;  // "", "fixstand", "rl"
        double vx = 0.0, vy = 0.0, wz = 0.0;
        double height = 0.98;
        bool ramps = false;   // questa voce CAMBIA la quota rispetto alla precedente
    };

    explicit ScriptedJoystick(const std::string & path) { load(path); }

    void update() override
    {
        const double t = scripted::g_sim_time.load();
        const double dt = (last_t_ < 0.0) ? 0.0 : std::max(0.0, t - last_t_);
        last_t_ = t;

        // tutto rilasciato di default: ogni ciclo riscrive lo stato completo
        back(0); start(0); LB(0); RB(0); A(0); B(0); X(0); Y(0);
        up(0); down(0); left(0); right(0);
        LT(0.0f); RT(0.0f);

        const Entry & e = active(t);
        const double since = t - e.t;   // < 0 solo prima della prima voce della timeline
        if (&e != last_) { last_ = &e; over_t_ = 0.0; }   // overhold: riparte a ogni voce

        // --- transizioni di stato: impulso, serve il FRONTE DI SALITA (DSL `.on_pressed`) ------
        if (!e.action.empty() && since >= 0.0 && since < kPulse)
        {
            if (e.action == "fixstand") { LT(1.0f); up(1); }   // Passive -> FixStand: LT + up
            else if (e.action == "rl")  { RB(1);    X(1);  }   // FixStand -> Squat:   RB + X
        }

        // --- quota: RT per scendere, B per salire, fino a raggiungere il target -----------------
        // Integratore IDENTICO a `base_height_command` nel controller (stesse rampe, stesso clamp).
        // L'anello si chiude sul MODELLO del target, non sull'altezza misurata: cosi' il comando
        // e' riproducibile per costruzione e non dipende da come la policy risponde -- che e'
        // esattamente la grandezza che stiamo misurando.
        if (target_ > e.height + kHeightEps)
        {
            RT(1.0f);
            target_ = std::max(e.height, target_ - kDescendRate * dt);
        }
        else if (target_ < e.height - kHeightEps)
        {
            B(1);
            target_ = std::min(e.height, target_ + kRiseRate * dt);
        }
        else if (overhold_ > 0.0 && since >= 0.0 && over_t_ < overhold_)
        {
            // OVERHOLD, solo alle quote di FINE CORSA. Il controller integra la rampa una volta per
            // passo di policy con step_dt nominale 0.02, ma il suo ciclo gira a ~46 Hz: sotto-integra
            // dell'8% e l'errore si accumula a ogni rampa (misurato: chiedendo 0.85/0.74/0.66/0.60 il
            // controller arrivava a 0.860/0.761/0.692/0.641). Agli estremi pero' `squat_target` viene
            // CLAMPATO: tenendo premuto oltre il necessario il controller finisce esattamente sul
            // limite, e da li' il modello e il controller tornano allineati. Serve percio' anche a
            // impedire che l'errore si accumuli attraverso le discese successive.
            // solo se questa voce ha davvero CHIESTO una rampa verso un estremo: altrimenti
            // ogni voce gia' a quota nominale (tutti i tratti di cammino in piedi) premerebbe B
            // per nulla, sporcando il flusso del joystick.
            const bool at_bound = (e.height <= deepest_ + 1e-6) || (e.height >= nominal_ - 1e-6);
            if (e.ramps && at_bound)
            {
                over_t_ += dt;
                if (e.height <= deepest_ + 1e-6) RT(1.0f);
                else                             B(1);
            }
        }
        target_ = std::clamp(target_, deepest_, nominal_);

        // --- levette -----------------------------------------------------------------------
        // Mappatura dal controller (observations.h):  obs = [ly, -lx, -rx]
        ly(static_cast<float>(e.vx));
        lx(static_cast<float>(-e.vy));
        rx(static_cast<float>(-e.wz));
        ry(0.0f);

        if (stop_at_ > 0.0 && t >= stop_at_) scripted::g_finished.store(true);
    }

    double stop_at() const { return stop_at_; }

private:
    static constexpr double kPulse = 0.30;        // durata dell'impulso di transizione [s]
    static constexpr double kDescendRate = 0.15;  // = descend_rate in observations.h
    static constexpr double kRiseRate = 0.15;     // = rise_rate   in observations.h
    static constexpr double kHeightEps = 1e-4;

    void load(const std::string & path)
    {
        YAML::Node cfg;
        try { cfg = YAML::LoadFile(path); }
        catch (const std::exception & ex) {
            std::cerr << "ScriptedJoystick: impossibile leggere " << path << ": " << ex.what() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        name_    = cfg["name"] ? cfg["name"].as<std::string>() : "unnamed";
        nominal_ = cfg["nominal_height"] ? cfg["nominal_height"].as<double>() : 0.98;
        deepest_ = cfg["deepest_height"] ? cfg["deepest_height"].as<double>() : 0.60;
        stop_at_ = cfg["stop_at"] ? cfg["stop_at"].as<double>() : -1.0;
        overhold_ = cfg["overhold"] ? cfg["overhold"].as<double>() : 0.0;
        target_  = nominal_;
        neutral_.height = nominal_;
        neutral_.t = -1e9;

        if (cfg["band_release_at"])
            scripted::g_band_release_at.store(cfg["band_release_at"].as<double>());

        // stato appiccicoso: ogni voce eredita dalla precedente cio' che non ridichiara
        Entry cur;
        cur.height = nominal_;
        for (const auto & n : cfg["timeline"])
        {
            Entry e = cur;
            e.action.clear();  // l'azione NON e' appiccicosa: e' un impulso
            e.t = n["t"].as<double>();
            if (n["do"])     e.action = n["do"].as<std::string>();
            if (n["vx"])     e.vx = n["vx"].as<double>();
            if (n["vy"])     e.vy = n["vy"].as<double>();
            if (n["wz"])     e.wz = n["wz"].as<double>();
            e.ramps = false;
            if (n["height"]) {
                const double newh = n["height"].as<double>();
                e.ramps = std::abs(newh - cur.height) > 1e-9;
                e.height = newh;
            }
            if (!entries_.empty() && e.t < entries_.back().t) {
                std::cerr << "ScriptedJoystick: timeline non ordinata a t=" << e.t << std::endl;
                std::exit(EXIT_FAILURE);
            }
            entries_.push_back(e);
            cur = e;
        }

        if (entries_.empty()) {
            std::cerr << "ScriptedJoystick: timeline vuota in " << path << std::endl;
            std::exit(EXIT_FAILURE);
        }

        // Avviso: il joystick FISICO azzera tutto sotto 0.15 (deadzone in physics_joystick.h,
        // messa li' per un difetto del pad). Il joystick scriptato NON la applica -- iniettare
        // valori esatti e' meglio e toglie una fonte di varianza -- ma un valore in quella fascia
        // non sarebbe riproducibile a mano, quindi va saputo.
        for (const auto & e : entries_)
            for (double v : {e.vx, e.vy, e.wz})
                if (std::abs(v) > 1e-9 && std::abs(v) < 0.15) {
                    std::cout << "ScriptedJoystick: ATTENZIONE, |comando|=" << std::abs(v)
                              << " a t=" << e.t << " e' sotto la deadzone 0.15 del pad fisico: "
                              << "questo istante non e' riproducibile a mano." << std::endl;
                    break;
                }

        std::cout << "ScriptedJoystick: protocollo '" << name_ << "', " << entries_.size()
                  << " voci, stop_at=" << stop_at_ << " s, banda rilasciata a "
                  << scripted::g_band_release_at.load() << " s, overhold=" << overhold_ << " s" << std::endl;
    }

    const Entry & active(double t) const
    {
        // Prima della prima voce non c'e' ancora protocollo: stato neutro. Senza questo ramo il
        // cursore resterebbe a 0 e l'impulso di transizione scatterebbe subito (|t - e.t| < pulse
        // e' vero anche per t MINORE di e.t), mandando il robot in FixStand all'istante 0.
        if (t < entries_.front().t) return neutral_;
        while (cursor_ + 1 < entries_.size() && entries_[cursor_ + 1].t <= t) ++cursor_;
        while (cursor_ > 0 && entries_[cursor_].t > t) --cursor_;  // dopo un reset del sim
        return entries_[cursor_];
    }

    std::vector<Entry> entries_;
    Entry neutral_;
    mutable size_t cursor_ = 0;
    const Entry * last_ = nullptr;
    double over_t_ = 0.0;
    std::string name_;
    double nominal_ = 0.98, deepest_ = 0.60, stop_at_ = -1.0, overhold_ = 0.0;
    double target_ = 0.98;
    double last_t_ = -1.0;
};
