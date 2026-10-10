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
#include <chrono>
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
    };

    explicit ScriptedJoystick(const std::string & path) { load(path); }

    void update() override
    {
        // Orologio del protocollo. Di default il tempo di SIMULAZIONE. Con `clock: wall_after_rl`
        // (replay di un esperimento reale) dal fronte di X dell'azione `rl` in poi il tempo avanza
        // con l'orologio di PARETE: il controller scandisce i passi di policy sull'orologio di parete
        // (sleep_until a 50 Hz), mentre la simulazione gira a rtf ~0.9995 -- su 270 s sarebbero
        // ~135 ms, 7 passi di policy, di deriva fra i comandi e i passi che li devono ricevere.
        double t = scripted::g_sim_time.load();
        if (wall_anchored_)
            t = anchor_t_ + std::chrono::duration<double>(std::chrono::steady_clock::now() - anchor_wall_).count();
        const double dt = (last_t_ < 0.0) ? 0.0 : std::max(0.0, t - last_t_);
        last_t_ = t;

        // Stato dei tasti calcolato PRIMA, poi ogni setter viene chiamato UNA volta sola in fondo.
        // LT e RT sono `Axis` con filtro EMA (x <- 0.97 x + 0.03 new): chiamare RT(0) e poi RT(1)
        // nello stesso ciclo significa due aggiornamenti, e il punto fisso diventa
        // 0.03/(1-0.97^2) = 0.508 -- esattamente sulla soglia 0.5 con cui il controller decide
        // R2.pressed. Il bit sfarfallava e la rampa di quota del controller andava a 1/3 della
        // velocita' nominale (misurato: 0.050 m/s invece di 0.15). Era anche la vera causa dello
        // "scarto del banco" attribuito a torto al ciclo del controller.
        bool bLT = false, bRT = false, bB = false, bUp = false, bRB = false, bX = false, bA = false;

        const Entry & e = active(t);
        const double since = t - e.t;   // < 0 solo prima della prima voce della timeline

        // --- transizioni di stato: impulso, serve il FRONTE DI SALITA (DSL `.on_pressed`) ------
        // Il DSL del controller vuole `LT + up.on_pressed`: il MODIFICATORE gia' premuto (LT e' un
        // Axis con EMA, supera 0.5 dopo ~23 ms) e il FRONTE del tasto. Quindi prima il modificatore
        // per tutto l'impulso, e il tasto solo dopo kModifierLead -- come fa una mano.
        if (!e.action.empty() && since >= 0.0 && since < kPulse)
        {
            const bool btn = since >= kModifierLead;
            if (e.action == "fixstand") { bLT = true; bUp = btn; }   // Passive -> FixStand: LT + up
            else if (e.action == "rl")  { bRB = true; bX = btn;  }   // FixStand -> Squat:   RB + X
            // ancora dell'orologio di parete: il fronte di X, cioe' l'ingresso in RL del controller
            if (wall_after_rl_ && !wall_anchored_ && e.action == "rl" && btn) {
                wall_anchored_ = true;
                anchor_t_ = e.t + kModifierLead;
                anchor_wall_ = std::chrono::steady_clock::now();
                t = anchor_t_;
            }
        }

        // --- quota: RT per scendere, B per salire, fino a raggiungere il target -----------------
        // Integratore IDENTICO a `base_height_command` nel controller (stesse rampe, stesso clamp).
        // L'anello si chiude sul MODELLO del target, non sull'altezza misurata: cosi' il comando
        // e' riproducibile per costruzione e non dipende da come la policy risponde -- che e'
        // esattamente la grandezza che stiamo misurando.
        //
        // FERMO del controller (`stop_height` nel protocollo, = cutoff_height + 1 cm): R2 da solo si
        // ferma li', per scendere oltre serve A + R2. Con la chiave il modello fa come una mano: RT
        // fino al fermo, poi A + RT. Senza la chiave (v1, v2, smoke, replay degli esperimenti reali
        // registrati prima del fermo) A resta premuto insieme a RT in ogni discesa: e' esattamente il
        // controller di prima, quindi quei protocolli comandano ancora le stesse quote.
        const bool was_moving = moving_, was_down = moving_down_;
        moving_ = false;
        if (target_ > e.height + kHeightEps)
        {
            bRT = true; moving_ = true; moving_down_ = true;
            const bool past_stop = !has_stop_ || target_ <= stop_ + kHeightEps;
            bA = past_stop;
            target_ = std::max(past_stop ? e.height : std::max(e.height, stop_), target_ - kDescendRate * dt);
        }
        else if (target_ < e.height - kHeightEps)
        {
            bB = true; moving_ = true; moving_down_ = false;
            target_ = std::min(e.height, target_ + kRiseRate * dt);
        }
        else
        {
            // OVERHOLD all'ARRIVO a un fine corsa: il tasto resta premuto `overhold` secondi in piu'.
            // Li' il controller CLAMPA `squat_target`, quindi finisce esattamente sul limite qualunque
            // fosse lo scarto accumulato (+-1 passo di policy per ogni inizio/fine rampa, perche' il
            // joystick gira sul tempo di simulazione e il controller sul suo orologio) e modello e
            // controller tornano allineati. Scatta sull'ARRIVO e non sulla voce della timeline, cosi'
            // funziona sia con timeline rade (standard_v2: una voce per quota) sia dense (replay di
            // un esperimento reale: una voce ogni 20 ms), e non preme mai nulla se la quota non si e'
            // mossa (niente B spuri durante il cammino in piedi).
            // Il fermo del controller e' un terzo fine corsa, ma solo arrivandoci DALL'ALTO: li' R2 da
            // solo clampa. Arrivandoci da sotto, R2 da solo non muove nulla e non riallinea niente.
            const bool low = e.height <= deepest_ + 1e-6, high = e.height >= nominal_ - 1e-6;
            const bool at_stop = has_stop_ && std::abs(e.height - stop_) < 1e-6;
            if (was_moving)
            {
                over_left_ = overhold_;
                over_key_ = low ? 'L' : high ? 'H' : (at_stop && was_down) ? 'S' : 0;
            }
            if (over_left_ > 0.0 && over_key_)
            {
                over_left_ -= dt;
                if (over_key_ == 'H') bB = true;
                else { bRT = true; bA = (over_key_ == 'L'); }   // sotto il fermo R2 da solo non scende
            }
            else over_left_ = 0.0;
        }
        target_ = std::clamp(target_, deepest_, nominal_);

        // --- scrittura dello stato: UNA chiamata per tasto/asse per ciclo ---------------------
        back(0); start(0); LB(0); Y(0); down(0); left(0); right(0);
        A(bA ? 1 : 0);
        LT(bLT ? 1.0f : 0.0f); RT(bRT ? 1.0f : 0.0f);
        up(bUp ? 1 : 0); RB(bRB ? 1 : 0); X(bX ? 1 : 0); B(bB ? 1 : 0);

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
    static constexpr double kPulse = 0.40;        // durata dell'impulso di transizione [s]
    static constexpr double kModifierLead = 0.15; // il tasto arriva dopo il modificatore [s]
    // Rampe del target di quota: DEVONO essere uguali a descend_rate / rise_rate nel controller
    // (deploy/include/isaaclab/envs/mdp/observations.h). Non sono costanti qui dentro ma chiavi del
    // protocollo (`descend_rate`, `rise_rate`), perche' quando il controller e' passato da 0.15 a
    // 0.05 m/s (commit 7e3bb51) il modello qui e' rimasto a 0.15 e le quote comandate sono
    // arrivate a un terzo (0.903 invece di 0.75). Se mancano nel YAML si usa 0.05 con un avviso.
    double kDescendRate = 0.05;
    double kRiseRate = 0.05;
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
        // `stick_smoothing: false` -> levette E grilletti (LT/RT) NON passano dal filtro EMA lato
        // simulatore. Serve al replay di un esperimento reale: sul robot il pacchetto del
        // radiocomando arriva al controller SENZA filtri, e l'unico filtro EMA e' quello del
        // controller (che c'e' anche in sim). Lasciare anche quello del simulatore raddoppiava il
        // ritardo: le levette di ~33 ms, e RT di ~23 ms -> ogni discesa partiva un passo dopo
        // (misurato con l'emulatore della catena completa). I comandi registrati sono gia' l'uscita
        // del filtro del controller: il replay lo pre-compensa sulle levette.
        if (cfg["stick_smoothing"] && !cfg["stick_smoothing"].as<bool>()) {
            lx.smooth = ly.smooth = rx.smooth = ry.smooth = 1.0f;
            LT.smooth = RT.smooth = 1.0f;
            stick_raw_ = true;
        }
        wall_after_rl_ = cfg["clock"] && cfg["clock"].as<std::string>() == "wall_after_rl";
        if (cfg["descend_rate"]) kDescendRate = cfg["descend_rate"].as<double>();
        if (cfg["rise_rate"])    kRiseRate    = cfg["rise_rate"].as<double>();
        if (!cfg["descend_rate"] || !cfg["rise_rate"])
            std::cout << "ScriptedJoystick: ATTENZIONE, descend_rate/rise_rate assenti dal protocollo: uso "
                      << kDescendRate << "/" << kRiseRate << " m/s. Devono coincidere con observations.h del controller."
                      << std::endl;
        // fermo del controller: deve valere cutoff_height + 1 cm del deploy.yaml della policy
        // (run_protocol.py lo verifica prima di partire). Assente = protocollo scritto prima del fermo.
        has_stop_ = static_cast<bool>(cfg["stop_height"]);
        if (has_stop_) {
            stop_ = cfg["stop_height"].as<double>();
            if (stop_ <= deepest_ || stop_ >= nominal_) {
                std::cerr << "ScriptedJoystick: stop_height " << stop_ << " fuori da (" << deepest_ << ", "
                          << nominal_ << ")" << std::endl;
                std::exit(EXIT_FAILURE);
            }
        }
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
            if (n["height"]) e.height = n["height"].as<double>();
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
        if (!stick_raw_)
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
                  << scripted::g_band_release_at.load() << " s, overhold=" << overhold_ << " s, rampe "
                  << kDescendRate << "/" << kRiseRate << " m/s"
                  << (has_stop_ ? ", fermo a " + std::to_string(stop_) + " m (RT fino al fermo, poi A + RT)"
                                : std::string(", A premuto con RT (protocollo senza fermo)"))
                  << (wall_after_rl_ ? ", orologio di parete dopo RL" : "")
                  << (stick_raw_ ? ", levette senza filtro" : "") << std::endl;
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
    bool wall_after_rl_ = false, wall_anchored_ = false;
    double anchor_t_ = 0.0;
    std::chrono::steady_clock::time_point anchor_wall_;
    bool moving_ = false, moving_down_ = false;
    double over_left_ = 0.0;
    char over_key_ = 0;   // overhold in corso: 'L' fondo (A + RT), 'S' fermo (RT), 'H' in piedi (B)
    bool has_stop_ = false;
    double stop_ = 0.0;
    bool stick_raw_ = false;
    std::string name_;
    double nominal_ = 0.98, deepest_ = 0.60, stop_at_ = -1.0, overhold_ = 0.0;
    double target_ = 0.98;
    double last_t_ = -1.0;
};
