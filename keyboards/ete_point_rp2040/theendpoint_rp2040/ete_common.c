// ete_common.c  (完成形 / I2C負荷最適化 + 反応遅延チューニング)
// ------------------------------------------------------------
// 目的:
//  - CPM計算 → Core2へ I2C送信（既定: 200ms）
//  - レイヤー変更 → I2C と / または RAW HID で送信
//  - ソレノイド → I2C と / または RAW HID で送信
//
// 改善点:
//  1) ソレノイドI2C送信を「即時」から「キュー送信」に変更（詰まり回避）
//  2) I2C timeout を短縮（既定: 2ms）
//  3) CPM送信周期を 200ms に（体感なめらか）
//  4) Mod-Tap / Layer-Tap のTapキーに展開して判定安定化
// ------------------------------------------------------------

#include "ete_common.h"
#include "timer.h"
#include "lib_combine/ETE/ETE.h"

#ifdef ETE_ENABLE_RAW
#    include "raw_hid.h"
#endif

#ifdef ETE_ENABLE_I2C
#    include "i2c_master.h"
#endif

#include "print.h"  // uprintf（不要なら外してOK）
#include "math.h"
#include "pointing_device.h"

#include "eeconfig.h"



// scroll mode API
bool ete_get_scroll_hold(void);
uint8_t ete_get_scroll_speed(void);
static uint8_t cursor_speed = 80;   // 100% = デフォルト

// ================================
// 設定
// ================================

// ---- I2C (Core2) ----
// QMK の i2c_transmit() は「8bit アドレス（7bit<<1）」流儀が多いので合わせる
#ifdef ETE_ENABLE_I2C
#    define ETE_I2C_ADDR_METER_8BIT  (0x0B << 1)   // TypingMeter
#    define ETE_I2C_ADDR_SOL_8BIT    (0x20 << 1)   // Solenoid

#    define CMD_CPM            0x01
#    define CMD_LAYER          0x02
#    define CMD_SOL_L          0x80
#    define CMD_SOL_S          0x81
#    define CMD_MISSILE        0x0D

// I2C timeout（ms）: 10ms は長いので短縮
#    ifndef ETE_I2C_TIMEOUT_MS
#        define ETE_I2C_TIMEOUT_MS 2
#    endif
#endif

// ---- CPM window / send period ----
#ifndef ETE_CPM_WINDOW_MS
#    define ETE_CPM_WINDOW_MS  1000
#endif

#ifndef ETE_SEND_PERIOD_MS
// 500ms→200ms（体感なめらか、負荷はキュー化で相殺）
#    define ETE_SEND_PERIOD_MS 200
#endif

// ---- RAW HID ----
// デフォルトは「レイヤー(0x02)だけ送る」想定
#ifndef ETE_RAW_SEND_KEYEVENT
#    define ETE_RAW_SEND_KEYEVENT 0
#endif

// ---- Solenoid queue ----
// 連打時に「抜け」を減らすためキュー化（1scanあたり最大1送信）
#ifndef ETE_SOL_QUEUE_SIZE
#    define ETE_SOL_QUEUE_SIZE 8
#endif

// ================================
// 内部状態
// ================================

static uint32_t window_start32 = 0;
static uint32_t window_count32 = 0;
static uint16_t current_cpm16  = 0;
static uint32_t last_send32    = 0;

static uint8_t  last_layer8    = 0xFF;

static bool swap_lr = false;

static bool scroll_hold = false;     // ① 押している間だけON
static uint8_t scroll_speed = 80;   // ② 速度(%) 100=等倍

static uint8_t touch_guard = 0;//指を置いた瞬間の誤爆防止

static uint8_t inertia_strength = 80; // 0〜100（デフォルト80）

static bool inertia_enabled = true;

// ソレノイド送信キュー
#ifdef ETE_ENABLE_SOLENOID
static uint8_t  sol_q[ETE_SOL_QUEUE_SIZE];
static uint8_t  sol_head = 0;
static uint8_t  sol_tail = 0;
#endif

// ================================
// ヘルパ
// ================================
static inline uint8_t ete_get_active_layer(void) {
    return (uint8_t)get_highest_layer(layer_state);
}

// Mod-Tap / Layer-Tap の Tap キーへ展開（強打判定などを壊さない）
static inline uint16_t ete_resolve_tap_keycode(uint16_t keycode) {

#if defined(IS_QK_MOD_TAP)
    if (IS_QK_MOD_TAP(keycode)) {
        return QK_MOD_TAP_GET_TAP_KEYCODE(keycode);
    }
#endif

#if defined(IS_QK_LAYER_TAP)
    if (IS_QK_LAYER_TAP(keycode)) {
        return QK_LAYER_TAP_GET_TAP_KEYCODE(keycode);
    }
#endif

    return keycode;
}

#ifdef ETE_ENABLE_RAW
static inline void ete_raw_send_layer(uint8_t layer) {
    uint8_t data[32] = {0};
    data[0] = 0x02;
    data[1] = layer;
    raw_hid_send(data, sizeof(data));
}

#    if ETE_RAW_SEND_KEYEVENT
static inline void ete_raw_send_keyevent(uint16_t keycode, bool pressed, uint8_t layer) {
    uint8_t data[32] = {0};
    data[0] = 0x01;
    data[1] = pressed ? 1 : 0;
    data[2] = keycode & 0xFF;
    data[3] = (keycode >> 8) & 0xFF;
    data[4] = layer;
    raw_hid_send(data, sizeof(data));
}
#    endif
#endif

#ifdef ETE_ENABLE_I2C
static inline void ete_i2c_send_layer(uint8_t layer) {
    uint8_t buf[2] = {CMD_LAYER, layer};
    i2c_transmit(ETE_I2C_ADDR_METER_8BIT, buf, sizeof(buf), ETE_I2C_TIMEOUT_MS);
}

static inline void ete_i2c_send_cpm(uint16_t cpm) {
    // Typingbridge 側が LSB/MSB だったとしても、Core2 側実装に合わせる必要あり。
    // 今の Core2 側が「MSB→LSB」想定ならこれでOK。（元コード踏襲）
    uint8_t buf[3];
    buf[0] = CMD_CPM;
    buf[1] = (cpm >> 8) & 0xFF;
    buf[2] = cpm & 0xFF;
    i2c_transmit(ETE_I2C_ADDR_METER_8BIT, buf, sizeof(buf), ETE_I2C_TIMEOUT_MS);
}

static inline void ete_i2c_send_cmd1(uint8_t cmd) {
    // Core2 側が 1バイトコマンド(0x10/0x11) を受ける想定
    i2c_transmit(ETE_I2C_ADDR_SOL_8BIT, &cmd, 1, ETE_I2C_TIMEOUT_MS);
}
#endif

#ifdef ETE_ENABLE_SOLENOID
static inline bool ete_solq_is_empty(void) {
    return sol_head == sol_tail;
}
static inline bool ete_solq_is_full(void) {
    return (uint8_t)(sol_head + 1) % ETE_SOL_QUEUE_SIZE == sol_tail;
}
static inline void ete_solq_push(uint8_t cmd) {
    // full のときは「古いのを捨てて新しいのを入れる」方が体感よい
    if (ete_solq_is_full()) {
        sol_tail = (uint8_t)(sol_tail + 1) % ETE_SOL_QUEUE_SIZE;
    }
    sol_q[sol_head] = cmd;
    sol_head = (uint8_t)(sol_head + 1) % ETE_SOL_QUEUE_SIZE;
}
static inline bool ete_solq_pop(uint8_t *out_cmd) {
    if (ete_solq_is_empty()) return false;
    *out_cmd = sol_q[sol_tail];
    sol_tail = (uint8_t)(sol_tail + 1) % ETE_SOL_QUEUE_SIZE;
    return true;
}
#endif

void ete_config_init(void) {
    uint32_t raw = 0;

    raw = ete_conf_set_scroll_speed(raw, 80);
    raw = ete_conf_set_cursor_speed(raw, 80);
    raw = ete_conf_set_inertia(raw, 80);
    raw = ete_conf_set_swap_lr(raw, false);

    eeconfig_update_kb(raw);

    uprintf("[INIT DEFAULT] raw=0x%08lX\n", (unsigned long)raw);
}


// ================================
// 公開API
// ================================

void ete_init(void) {
    if (!eeconfig_is_enabled()) {
        eeconfig_init();
        ete_config_init();
    }

    uint32_t raw = eeconfig_read_kb();
    bool need_save = false;

    // --- swap ---
    swap_lr = ete_conf_get_swap_lr(raw);

    // --- scroll ---
    uint8_t spd = ete_conf_get_scroll_speed(raw);
    if (spd >= 30 && spd <= 127) {
        scroll_speed = spd;
    } else {
        scroll_speed = 80;
        raw = ete_conf_set_scroll_speed(raw, scroll_speed);
        need_save = true;
    }

    // --- cursor ---
    uint8_t cs = ete_conf_get_cursor_speed(raw);
    if (cs >= 30 && cs <= 127) {
        cursor_speed = cs;
    } else {
        cursor_speed = 80;
        raw = ete_conf_set_cursor_speed(raw, cursor_speed);
        need_save = true;
    }

    // --- inertia ---
    uint8_t in = ete_conf_get_inertia(raw);

    uprintf("[INERTIA LOAD] raw=0x%08lX mask=0x%08lX val=%u\n",
        (unsigned long)raw,
        (unsigned long)(raw & ETE_CONF_INERTIA_MASK),
        in);

    if (in <= 127 && in > 0) {
        inertia_strength = in;
    } else {
        inertia_strength = 80;
        raw = ete_conf_set_inertia(raw, inertia_strength);
        need_save = true;
    }

    // --- 必要なら修復保存 ---
    if (need_save) {
        eeconfig_update_kb(raw);
        uprintf("[EEPROM FIXED] raw=0x%08lX\n", (unsigned long)raw);
    }

    window_start32 = timer_read32();
    last_send32    = timer_read32();
    window_count32 = 0;
    current_cpm16  = 0;
    last_layer8    = 0xFF;

#ifdef ETE_ENABLE_SOLENOID
    sol_head = sol_tail = 0;
#endif

#ifdef ETE_ENABLE_I2C
    i2c_init();
    wait_ms(10);
#endif

    ete_on_layer(ete_get_active_layer());

uprintf("[ETE] init raw=0x%08lX swap=%u scroll=%u cursor=%u inertia=%u master=%u\n",
    (unsigned long)raw,
    swap_lr,
    scroll_speed,
    cursor_speed,
    inertia_strength,
    is_keyboard_master());
}

void ete_solenoid_light(void) {
#ifdef ETE_ENABLE_SOLENOID
    // 即I2Cしない。キューに積む（詰まり回避）
    ete_solq_push(CMD_SOL_L);
#endif

#ifdef ETE_ENABLE_RAW
    // RAW HID でも必要なら送れる（PC側で使う時だけ）
    uint8_t data[32] = {0};
    data[0] = CMD_SOL_L;
    raw_hid_send(data, sizeof(data));
#endif
}

void ete_solenoid_strong(void) {
#ifdef ETE_ENABLE_SOLENOID
    ete_solq_push(CMD_SOL_S);
#endif

#ifdef ETE_ENABLE_RAW
    uint8_t data[32] = {0};
    data[0] = CMD_SOL_S;
    raw_hid_send(data, sizeof(data));
#endif
}

// ---- キーイベント（process_record_user() から呼ぶ）----
void ete_on_key(uint16_t keycode, keyrecord_t *record) {

    // ---- CPM 用カウント ----
    if (record->event.pressed) {
        window_count32++;
    }

#if defined(ETE_ENABLE_RAW) && (ETE_RAW_SEND_KEYEVENT == 1)
    const uint8_t layer = ete_get_active_layer();
    ete_raw_send_keyevent(keycode, record->event.pressed, layer);
#endif

#ifdef ETE_ENABLE_SOLENOID
if (record->event.pressed) {

    if (keycode == KC_ENT || keycode == KC_ENTER) {
        ete_solq_push(CMD_MISSILE);   // ★ ミサイル
    } else {
        ete_solq_push(CMD_SOL_L);     // ★ 通常
    }
}
#endif
}

// ---- レイヤー変更（layer_state_set_user() から呼ぶ）----
void ete_on_layer(uint8_t layer) {
    if (layer == last_layer8) return;
    last_layer8 = layer;

#ifdef ETE_ENABLE_I2C
    ete_i2c_send_layer(layer);
#endif
#ifdef ETE_ENABLE_RAW
    ete_raw_send_layer(layer);
#endif

    uprintf("[ETE] layer=%u\n", layer);
}

// ---- 定期処理（matrix_scan_user() から呼ぶ）----
void ete_tick(void) {
    const uint32_t now = timer_read32();

    // 1秒ごとにCPM更新
    if (timer_elapsed32(window_start32) >= ETE_CPM_WINDOW_MS) {
        uint32_t cpm = window_count32 * 60UL;
        if (cpm > 2000) cpm = 2000;
        current_cpm16  = (uint16_t)cpm;

        window_count32 = 0;
        window_start32 = now;
    }

#ifdef ETE_ENABLE_I2C
    // ★ まずソレノイド（体感優先）を 1scanにつき最大1発だけ送る
    //    これで連打してもI2Cが詰まりにくい
#    ifdef ETE_ENABLE_SOLENOID
    uint8_t solcmd;
    if (ete_solq_pop(&solcmd)) {
        ete_i2c_send_cmd1(solcmd);
    }
#    endif

    // 200msごとにCore2へ送信（I2C）
    if (timer_elapsed32(last_send32) >= ETE_SEND_PERIOD_MS) {
        last_send32 = now;
        ete_i2c_send_cpm(current_cpm16);
    }
#endif
}

// 便利：外から現在CPMを読む（任意）
uint16_t ete_get_cpm(void) {
    return current_cpm16;
}



//-----//
//トラックパッドチューニング（呼び出し確認 + 安定化）
//----//
report_mouse_t ete_pointing_tune(report_mouse_t report)
{
    // ==============================
    // 永続変数（全部ここに置く）
    // ==============================

    static float scroll_accum_v = 0;
    static float scroll_accum_h = 0;
    static uint8_t scroll_release_guard = 0;
    

 // ==============================
// 精密トラックボール最終版
// ==============================


int vx = report.x;
int vy = report.y;

float dead = 2.0f;


// ===== ① HOLDスクロールモード：XYをVHへ変換 =====
if (ete_get_scroll_hold()) {
    report.h = vx;
    report.v = vy;

    report.x = 0;
    report.y = 0;
    report.buttons = 0;   // 追加

    return report;
}

// なだらかデッドゾーン
if (fabs(vx) < dead) vx = 0;
if (fabs(vy) < dead) vy = 0;

int speed = abs(vx) + abs(vy);

// ===== velocityモデル =====
static float vel_x = 0;
static float vel_y = 0;


float cursor_scale = (float)cursor_speed / 127.0f;

// 入力
float input_x = vx * cursor_scale;
float input_y = vy * cursor_scale;


// ===== 入力中 =====
if (speed > 0) {
    float inertia = inertia_enabled
    ? (float)inertia_strength / 100.0f
    : 0.0f;

    inertia = inertia * 1.4f;


    // ★低速時は慣性を弱める
    if (speed < 3) {
        inertia *= 0.3f;
    }
    else if (speed < 8) {
        inertia *= 0.6f;
    }
    float accel;

    if (speed < 3) {
        accel = 0.08f;   // ←初速
    }
    else if (speed < 10) {
        accel = 0.25f;
    }
    else {
        accel = 0.35f;
    }

    // ★ inertiaで伸びる
    accel += inertia * 0.15f;

    vel_x = vel_x * (1.0f - accel) + input_x * accel;
    vel_y = vel_y * (1.0f - accel) + input_y * accel;
}
else {
    // ★ 速度ベース慣性
    float speed_f = sqrtf(vx*vx + vy*vy);

    float inertia = inertia_enabled
        ? (float)inertia_strength / 100.0f
        : 0.0f;

    inertia = inertia * 1.5f;

    float dynamic_decay;

    if (speed_f < 3) {
        dynamic_decay = 0.75f + inertia * 0.1f;
    }
    else if (speed_f < 10) {
        dynamic_decay = 0.85f + inertia * 0.15f;
    }
    else {
        dynamic_decay = 0.92f + inertia * 0.10f;
    }
    
    if (!inertia_enabled) {
    vel_x = input_x;
    vel_y = input_y;
    }
    
    // 微振動キラー
    if (speed < 2 && fabs(vel_x) < 0.3f) vel_x = 0;
    if (speed < 2 && fabs(vel_y) < 0.3f) vel_y = 0;

    vel_x *= dynamic_decay;
    vel_y *= dynamic_decay;

    if (fabs(vel_x) < 0.05f) vel_x = 0;
    if (fabs(vel_y) < 0.05f) vel_y = 0;
}

// 出力
report.x = (int)vel_x;
report.y = (int)vel_y;

    // ==============================
    // ヌルヌル型 精密スクロール
    // ==============================

    int raw_scroll_speed = abs(report.v) + abs(report.h);

    // ---- 終端検出 ----
    if (raw_scroll_speed <= 1) {
        scroll_release_guard++;
    } else {
        scroll_release_guard = 0;
    }

    if (scroll_release_guard >= 1) {
        if (fabs(scroll_accum_v) < 0.6f) scroll_accum_v = 0;
        if (fabs(scroll_accum_h) < 0.6f) scroll_accum_h = 0;
    }

    int sv = report.v;
    int sh = report.h;

    // ==============================
    // ソフトデッドゾーン（神設定）
    // ==============================
    float deads = 0;
    
    float fsv = (float)sv;
    float fsh = (float)sh;

    // Y
    if (fabs(fsv) < deads) {
        fsv = fsv * 0.8f;   // ←完全に殺さない
    } else {
        fsv = (fsv > 0) ? (fsv - deads) : (fsv + deads);
    }

    // X
    if (fabs(fsh) < deads) {
        fsh = fsh * 0.8f;
    } else {
        fsh = (fsh > 0) ? (fsh - deads) : (fsh + deads);
    }

    // ★超低速ブースト（ここがキモ）
    if (fabs(fsv) > 0 && fabs(fsv) < 1.5f) {
        fsv *= 1.8f;
    }
    if (fabs(fsh) > 0 && fabs(fsh) < 1.5f) {
        fsh *= 1.8f;
    }

    // 戻す
    sv = (int)fsv;
    sh = (int)fsh;

    int scroll_speed = abs(sv) + abs(sh);


    // ==============================
    // 超なめらか連続カーブ（初動改善版）
    // ==============================

    float s = (float)scroll_speed;

    // 0〜40くらいを想定して正規化
    float t = s / 40.0f;
    if (t > 1.0f) t = 1.0f;

    // ease-outカーブ（初動を極小に）
    float eased = t * t * t;  // ←ここ重要（指数2乗）

    // スケール生成
    float scroll_scale = 0.004f + eased * 0.35f;

    // 上限
    if (scroll_scale > 0.7f) scroll_scale = 0.7f;

    float out_sv = sv * scroll_scale;
    float out_sh = sh * scroll_scale;

    // スクロール速度反映（先にやる！）
    float sp = (float)ete_get_scroll_speed() / 100.0f;
    out_sv *= sp;
    out_sh *= sp;


    static float scroll_vel_v = 0;
    static float scroll_vel_h = 0;

   float inertia = inertia_enabled
    ? (float)inertia_strength / 100.0f
    : 0.0f;
    inertia = inertia * 1.2f;

    if (!inertia_enabled) {
    scroll_vel_v = out_sv;
    scroll_vel_h = out_sh;
}

    // 減衰係数（スクロール用）
    float decay;

    float vel_power = fabs(scroll_vel_v) + fabs(scroll_vel_h);

    if (vel_power < 0.5f) {
    decay = 0.65f + inertia * 0.10f;
    }
    else if (vel_power < 3.0f) {
        decay = 0.90f + inertia * 0.07f;
    }
    else {
        decay = 0.97f + inertia * 0.03f;
    }

    //符号ブレ防止
    // ★入力がある場合
    if (fabs(out_sv) > 0 || fabs(out_sh) > 0) {
        float accel;

        if (vel_power < 0.5f) {
            accel = 0.18f;                  // ← 初動ブースト
            accel += inertia * 0.05f;       // ← 慣性弱め
        }
        else if (vel_power < 2.0f) {
            accel = 0.35f;
            accel += inertia * 0.25f;
        }
        else {
            accel = 0.6f;
            accel += inertia * 0.2f;
        }

        scroll_vel_v = scroll_vel_v * (1.0f - accel) + out_sv * accel;
        scroll_vel_h = scroll_vel_h * (1.0f - accel) + out_sh * accel;
    }
    else {
        // ★入力なし → 慣性だけ
        scroll_vel_v *= decay;
        scroll_vel_h *= decay;
    }


    // 停止処理
    if (fabs(scroll_vel_v) < 0.2f) scroll_vel_v = 0;
    if (fabs(scroll_vel_h) < 0.2f) scroll_vel_h = 0;

    // ★最大速度制限・フレームレート調整
    if (scroll_vel_v > 2.5f) scroll_vel_v = 2.5f;
    if (scroll_vel_v < -2.5f) scroll_vel_v = -2.5f;

    if (scroll_vel_h > 2.5f) scroll_vel_h = 2.5f;
    if (scroll_vel_h < -2.5f) scroll_vel_h = -2.5f;

    // 出力
    static float scroll_rem_v = 0;
    static float scroll_rem_h = 0;

    scroll_rem_v += scroll_vel_v;
    scroll_rem_h += scroll_vel_h;

    int out_v = (int)scroll_rem_v;
    int out_h = (int)scroll_rem_h;

    scroll_rem_v -= out_v;
    scroll_rem_h -= out_h;

    report.v = out_v;
    report.h = out_h;
    
    // ★ここ追加！！！！
    if (report.buttons &&
        (fabs(scroll_vel_v) > 0.1f || fabs(scroll_vel_h) > 0.1f)) {

        scroll_vel_v = 0;
        scroll_vel_h = 0;
        scroll_rem_v = 0;
        scroll_rem_h = 0;
    }


    // ==============================
    // 中速域に気持ちよさを作る
    // ==============================

        if (ete_get_scroll_hold()) {
        report.buttons = 0;
    }

    //低速操作時のクリック誤検知防止
    int move_mag = abs(report.x) + abs(report.y);

    // ★ 低速時クリック禁止
    if (move_mag > 0 && move_mag < 3) {//3～4で調整
        report.buttons = 0;
    }

    //指を置いた瞬間の誤検知防止
    if (speed > 0) {
        touch_guard = 3;  // 数フレームガード
    }

    if (touch_guard > 0) {
        report.buttons = 0;
        touch_guard--;
    }

    
    return report;
}


report_mouse_t pointing_device_task_user(report_mouse_t report)
{
    return ete_pointing_tune(report);
}



bool ete_get_swap_state(void) {
    return swap_lr;
}


void ete_set_scroll_hold(bool on) {
    // masterに集約したい場合（おすすめ）

        scroll_hold = on;
}

bool ete_get_scroll_hold(void) {
    return scroll_hold;
}

uint8_t ete_get_scroll_speed(void) {
    return scroll_speed;
}

void ete_scroll_speed_inc(void) {
    if (scroll_speed < 200) {
        scroll_speed += 5;
    }
}

void ete_scroll_speed_dec(void) {
    if (scroll_speed > 30) {
        scroll_speed -= 5;
    }
}


void ete_cursor_speed_inc(void) {
    if (cursor_speed < 200) {
        cursor_speed += 5;
    }
}

void ete_cursor_speed_dec(void) {
    if (cursor_speed > 50) {
        cursor_speed -= 5;
    }
}

void keyboard_post_init_user(void)
{
    uint32_t raw = eeconfig_read_kb();

    scroll_speed     = ete_conf_get_scroll_speed(raw);
    cursor_speed     = ete_conf_get_cursor_speed(raw);
    swap_lr          = ete_conf_get_swap_lr(raw);
    inertia_strength = ete_conf_get_inertia(raw);

    uprintf("[LOAD] raw=0x%08lX scroll=%u cursor=%u inertia=%u swap=%u\n",
        (unsigned long)raw,
        scroll_speed,
        cursor_speed,
        inertia_strength,
        swap_lr);
}

static inline uint8_t to_percent(uint8_t v) {
    return (v * 100) / 127;
}

void ete_settings_save(void) {
    uint32_t raw = eeconfig_read_kb();

    raw = ete_conf_set_scroll_speed(raw, scroll_speed);
    raw = ete_conf_set_cursor_speed(raw, cursor_speed);
    raw = ete_conf_set_swap_lr(raw, swap_lr);
    raw = ete_conf_set_inertia(raw, inertia_strength);

    eeconfig_update_kb(raw);
    uprintf("[SAVE] scroll=%u cursor=%u inertia=%u (%s)\n",
        to_percent(scroll_speed),
        to_percent(cursor_speed),
        to_percent(inertia_strength),
        inertia_enabled ? "inertia_ON" : "inertia_OFF"
    );
}

void ete_toggle_lr(void) {
    swap_lr = !swap_lr;

    if (!is_keyboard_master()) return;

    uint32_t raw = eeconfig_read_kb();
    raw = ete_conf_set_swap_lr(raw, swap_lr);
    eeconfig_update_kb(raw);

    uprintf("[SWAP SAVE] raw=0x%08lX swap=%u\n",
            (unsigned long)raw, swap_lr);
}


uint8_t ete_get_inertia(void) {
    return inertia_strength;
}

void ete_inertia_inc(void) {
    if (inertia_strength <= 95) {
        inertia_strength += 5;
    } else {
        inertia_strength = 100;
    }
    uprintf("[INERTIA] %u\n", inertia_strength);
}


void ete_inertia_dec(void) {
    if (inertia_strength >= 5) {
        inertia_strength -= 5;
    } else {
        inertia_strength = 0;
    }
    uprintf("[INERTIA] %u\n", inertia_strength);
}

void ete_toggle_inertia(void) {

    inertia_enabled = !inertia_enabled;

    if (inertia_enabled) {
        // EEPROMから復元
        uint32_t raw = eeconfig_read_kb();
        inertia_strength = ete_conf_get_inertia(raw);

        uprintf("[INERTIA ON] %u\n", inertia_strength);
    } else {
        uprintf("[INERTIA OFF]\n");
    }
}