/* Klon der AldesConnect Box auf ESP32-S31 — Bausteinverwaltung.
 *
 * Port von firmware/src/cb_cdc_driver.c (Klassentreiber, Frame-Reassemblierung,
 * TX-Einmalversuch), netcontrol.c (Befehle), main.c (STATUS-Zeile) und eines
 * Teils von pico.Pilot (45-s-Sperre, Kennungsvergabe) auf esp_tinyusb 2.2.1.
 *
 * Der Klasse-Treiber laeuft im TinyUSB-Task; alles, was Netz/Entities angeht
 * (Anzeige, Freigabe, Senden), laeuft im ESPHome-Main-Loop. Geteilter Zustand
 * ist deshalb mit einem rekursiven Mutex geschuetzt. Es gibt genau einen
 * Schreibkanal: SEND mit frischer Freigabe, ein Versuch, kein Replay.
 */
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#include "aldes_gate.h"
#include "aldes_report21.h"

namespace esphome {
namespace aldes_cb {

/* Default-Marke fuer "kein Sollwert/Messwert" (Home Assistant: unavailable). */
constexpr float ALDES_NAN = NAN;

/* ------------------------------------------------------------- Baustein */

class AldesCB;
class AldesCBServices;

/* ------------------------------------------------------------ Ziele (Sinks)
 *
 * Alle Zustandszeilen entstehen im TinyUSB-Task oder im Frame-Parser; ESPHome-
 * Entitaeten (text_sensor, sensor, select, ...) duerfen aber nur aus dem
 * Main-Loop angesprochen werden. Ein Sink merkt sich den letzten Wert und
 * schiebt ihn erst beim flush() am Ende des Main-Loop-Durchlaufs weiter. */
class TextSink {
 public:
  explicit TextSink(AldesCB *parent) : parent_(parent) {}
  void publish_state(const std::string &msg) {
    line_ = msg;
    pending_ = true;
  }
  void add_on_state_callback(std::function<void(const std::string &)> &&cb) { callbacks_.push_back(std::move(cb)); }
  void flush();
  size_t count() const { return callbacks_.size(); }

 protected:
  AldesCB *parent_{nullptr};
  std::string line_;
  bool pending_{false};
  std::vector<std::function<void(const std::string &)>> callbacks_;
};

class FloatSink {
 public:
  explicit FloatSink(AldesCB *parent) : parent_(parent) {}
  void publish_state(float v) {
    value_ = v;
    pending_ = true;
  }
  void add_on_state_callback(std::function<void(float)> &&cb) { callbacks_.push_back(std::move(cb)); }
  void flush();
  size_t count() const { return callbacks_.size(); }

 protected:
  AldesCB *parent_{nullptr};
  float value_{ALDES_NAN};
  bool pending_{false};
  std::vector<std::function<void(float)>> callbacks_;
};

/* Auswahlliste: Index = Rohwert (0..n-1). `control()` kommt aus Home
 * Assistant, `flush()` schiebt beides am Ende des Main-Loop-Durchlaufs raus.
 * Die Texte stehen in options_ (Reihenfolge = Rohwert, wie pico.AIR_MODES). */
class IndexSink {
 public:
  explicit IndexSink(AldesCB *parent, size_t options = 0) : parent_(parent), options_(options) {}
  void set_options(const std::vector<std::string> &names) {
    names_ = names;
    if (!names_.empty()) options_ = names_.size();
  }
  void publish_state(float index) {
    value_ = index;
    pending_ = true;
  }
  void control(size_t index);
  /* Aus der HA-Oberflaeche: Text -> Rohwert; unbekannter Text wird verworfen. */
  void control_name(const std::string &name);
  void add_on_state_callback(std::function<void(float)> &&cb) { callbacks_.push_back(std::move(cb)); }
  void add_on_control_callback(std::function<void(size_t)> &&cb) { on_control_.push_back(std::move(cb)); }
  void flush();
  size_t options() const { return options_; }

 protected:
  AldesCB *parent_{nullptr};
  size_t options_;
  std::vector<std::string> names_;
  float value_{ALDES_NAN};
  bool pending_{false};
  std::vector<std::function<void(float)>> callbacks_;
  std::vector<std::function<void(size_t)>> on_control_;
  std::vector<size_t> pending_controls_;
};

/* Sollwert-Zahl (16–30 °C, 0,5-°C-Raster) — das Bedienelement der HA-Box; der
 * ausgewaehlte Wert geht mit dem Knopf "Send setpoints". */
class NumberSink {
 public:
  explicit NumberSink(AldesCB *parent) : parent_(parent) {}
  void control(float v) {
    state_ = v;
    has_ = true;
  }
  bool has() const { return has_; }
  float state() const { return state_; }

 protected:
  AldesCB *parent_{nullptr};
  float state_{20.0f};
  bool has_{false};
};

class ButtonSink {
 public:
  explicit ButtonSink(AldesCB *parent) : parent_(parent) {}
  void press() {
    if (on_press_) on_press_();
  }
  void add_on_press_callback(std::function<void()> &&cb) { on_press_ = std::move(cb); }

 protected:
  AldesCB *parent_{nullptr};
  std::function<void()> on_press_;
};

/* ------------------------------------------------------------- Baustein */

#ifdef USE_SELECT
/* Selekt, dessen HA-Aufruf im Sink landet statt in einer Textsuche. ESPHome
 * ruft control(size_t) aus dem SelectCall; jede andere Basisklasse wuerde hier
 * zuerst den Text bilden und wieder einordnen. */
class ControlSelect : public select::Select {
 public:
  void set_sink_index_callback(std::function<void(size_t)> &&cb) { sink_cb_ = std::move(cb); }

  /* Rohwert -> Option; unbekannt (NAN/ausserhalb) leert den Zustand. */
  void publish_index(float index) {
    if (std::isnan(index) || index < 0 || (size_t) index >= this->size()) {
      this->publish_state(std::string());
      return;
    }
    this->publish_state(this->option_at((size_t) index));
  }

 protected:
  void control(size_t index) override {
    if (sink_cb_) sink_cb_(index);
  }

  std::function<void(size_t)> sink_cb_;
};
#endif

#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_NUMBER
/* Sollwert-Zahl, deren HA-Aufruf im Sink landet (kein Senden). */
class ControlNumber : public number::Number {
 public:
  void set_number_control_callback(std::function<void(float)> &&cb) { control_cb_ = std::move(cb); }

 protected:
  void control(float value) override {
    if (control_cb_) control_cb_(value);
  }

  std::function<void(float)> control_cb_;
};
#endif

#ifdef USE_BUTTON
/* Knopf, der beim Druecken den Sink des Bausteins anstoesst (press_action). */
class ControlButton : public button::Button {
 public:
  void set_press_callback(std::function<void()> &&cb) { press_cb_ = std::move(cb); }

 protected:
  void press_action() override {
    if (press_cb_) press_cb_();
  }

  std::function<void()> press_cb_;
};
#endif

class AldesCB : public Component {
 public:
  static constexpr uint8_t SLOT_SETPOINT_K1 = 0;
  static constexpr uint8_t SLOT_SETPOINT_K2 = 1;
  static constexpr uint8_t SLOT_SETPOINT_K3 = 2;
  static constexpr uint8_t SLOT_SETPOINT_K4 = 3;
  static constexpr uint8_t SLOT_TEMPERATURE_K1 = 4;
  static constexpr uint8_t SLOT_TEMPERATURE_K2 = 5;
  static constexpr uint8_t SLOT_TEMPERATURE_K3 = 6;
  static constexpr uint8_t SLOT_TEMPERATURE_K4 = 7;
  static constexpr uint8_t SLOT_AIR = 8;
  static constexpr uint8_t SLOT_ECS = 9;
  static constexpr uint8_t SLOT_CLOCK = 10;
  static constexpr uint8_t SLOT_COUNT = 11;

  static constexpr uint8_t AIR_MODE_COUNT = ALDES_AIR_MODE_COUNT;
  static constexpr uint8_t ECS_MODE_COUNT = ALDES_ECS_MODE_COUNT;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void on_shutdown() override;

  bool is_ready() const;
  void init_usb();
  void install_usb(); /* fuer USBON nach Stop/Timeout */

  /* --- Ausgaenge von Python (`to_code`) --- */
  void set_log_sink(TextSink *sink) { log_sink_ = sink; }
  TextSink *log_sink() const { return log_sink_; }
  void set_report_sensor(uint8_t slot, FloatSink *sink);
  void set_air_select(IndexSink *sel) { air_select_ = sel; }
  void set_ecs_select(IndexSink *sel) { ecs_select_ = sel; }
  void set_setpoint_number(uint8_t zone, NumberSink *number);
  void set_status_text(TextSink *t) { status_text_ = t; }
  void set_frame_text(TextSink *t) { frame_text_ = t; }
  void set_frame_event_text(TextSink *t) { frame_event_text_ = t; }
  void set_send_setpoints_button(ButtonSink *b) { btn_send_setpoints_ = b; }
  void set_send_air_button(ButtonSink *b) { btn_send_air_ = b; }
  void set_send_ecs_button(ButtonSink *b) { btn_send_ecs_ = b; }
  void set_vacation_clear_button(ButtonSink *b) { btn_vacation_clear_ = b; }
  void set_usb_on_button(ButtonSink *b) { btn_usb_on_ = b; }
  /* API-Dienste anmelden (aldes_cb.send/.arm/.stop/.usb_on). */
  void enable_services() { services_enabled_ = true; }
  void set_arm_button(ButtonSink *b) { btn_arm_ = b; }
  void set_stop_button(ButtonSink *b) { btn_stop_ = b; }

  /* --- Zeitfenster fuer 0x21-Werte (reader.snapshot(), Pico: 25 s) --- */
  void set_report_window_seconds(uint32_t seconds) { report_window_s_ = seconds; }
  /* Zustandszeile-Kadenz (main.c: 10 s). */
  void set_status_period_ms(uint32_t ms) { status_period_ms_ = ms; }

  /* --- Knopf- und Auswahl-Aktionen (immer im Main-Loop) --- */
  void send_setpoints();
  void send_air();
  void send_ecs();
  void vacation_clear();
  void arm_from_ui();
  void stop_from_ui();
  void usb_on_from_ui();

  /* --- Rohbefehle (Host-Test und Diagnose) --- */
  std::string exec_workbench_line(const std::string &line);
  std::string status_line();

  void air_selected(size_t index) { air_choice_ = index; air_choice_set_ = true; }
  void ecs_selected(size_t index) { ecs_choice_ = index; ecs_choice_set_ = true; }

  /* Zeile ins Pico-Format-Log (und in den Log-TextSensor). */
  void log_line_(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
  /* Frame senden: kind 0 = Sollwert (zone 1..4), 1 = Air, 2 = ECS,
   * 3 = Urlaub raeumen. Nur ueber den Main-Loop aufrufen. */
  void request_send_(uint8_t kind, float value, uint8_t zone);

 protected:
  void publish_(uint32_t now);
  float report_value_(uint8_t slot, const aldes_report21_t &r) const;
  bool send_allowed_(std::string *why);
  void submit_frame_(const uint8_t *frame, size_t len, const char *action);
  time_t pac_clock_to_epoch_(const aldes_report21_t &r) const;

  TextSink *log_sink_{nullptr};
  std::array<FloatSink *, SLOT_COUNT> report_sinks_{};
  IndexSink *air_select_{nullptr};
  IndexSink *ecs_select_{nullptr};
  std::array<NumberSink *, 4> setpoint_numbers_{};
  TextSink *status_text_{nullptr};
  TextSink *frame_text_{nullptr};
  TextSink *frame_event_text_{nullptr};
  ButtonSink *btn_send_setpoints_{nullptr};
  ButtonSink *btn_send_air_{nullptr};
  ButtonSink *btn_send_ecs_{nullptr};
  ButtonSink *btn_vacation_clear_{nullptr};
  ButtonSink *btn_usb_on_{nullptr};
  ButtonSink *btn_arm_{nullptr};
  ButtonSink *btn_stop_{nullptr};

  /* Was der Benutzer in Home Assistant ausgewaehlt hat (nicht der PAC-Stand). */
  size_t air_choice_{0};
  bool air_choice_set_{false};
  size_t ecs_choice_{0};
  bool ecs_choice_set_{false};
  bool published_good_{false};
  bool services_enabled_{false};
  AldesCBServices *services_{nullptr};

  friend class AldesCBServices;

  uint32_t report_window_s_{25};
  uint32_t status_period_ms_{10000};
  uint32_t last_status_ms_{0};
  bool first_status_{true};
};

/* ------------------------------------------------------- HA-Dienste
 *
 * Home Assistant ruft aldes_cb.send / .arm / .stop / .usb_on; ESPHome-API-
 * Dienste werden ueber register_service() aus CustomAPIDevice angemeldet.
 * Dieses kleine Objekt haelt die Dienste, damit der Baustein selbst kein
 * API-Kind wird (und ohne API weiter uebersetzt). */
class AldesCBServices {
 public:
  explicit AldesCBServices(AldesCB *parent) : parent_(parent) {}
  void setup();

  /* Argumente wie im YAML beschrieben. */
  void on_send(std::string action, float value, int32_t zone);
  void on_arm();
  void on_stop();
  void on_usb_on();

 protected:
  AldesCB *parent_{nullptr};
};

}  // namespace aldes_cb
}  // namespace esphome
