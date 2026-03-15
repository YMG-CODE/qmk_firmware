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
static uint8_t cursor_speed = 100;   // 100% = デフォルト

#define ETE_EECONF_SWAPLR_MASK      0x00000001UL
#define ETE_EECONF_SCROLLSPD_SHIFT  1
#define ETE_EECONF_SCROLLSPD_MASK   0x000001FEUL  // bits 8..1
#define ETE_EECONF_CURSORSPD_SHIFT 9
#define ETE_EECONF_CURSORSPD_MASK  0x0003FE00UL


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
static uint8_t scroll_speed = 100;   // ② 速度(%) 100=等倍

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

// ================================
// 公開API
// ================================

void ete_init(void) {

    uint32_t val = eeconfig_read_kb();
    swap_lr = (val & 0x01);

    uint32_t spd = (val & ETE_EECONF_SCROLLSPD_MASK) >> ETE_EECONF_SCROLLSPD_SHIFT;
    if (spd >= 30 && spd <= 200) {
        scroll_speed = (uint8_t)spd;
    } else {
        scroll_speed = 100; // デフォルト
    }

    uint32_t cs = (val & ETE_EECONF_CURSORSPD_MASK) >> ETE_EECONF_CURSORSPD_SHIFT;
    if (cs >= 50 && cs <= 200) {
    cursor_speed = (uint8_t)cs;
    } else {
     cursor_speed = 100;    
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

    // ※ I2C速度アップ（環境依存なので“あれば”使う）
    // QMKの環境によって関数/マクロが違うので、ここはコンパイル安全にしている
    // もしあなたの環境に i2c_set_speed があるなら有効化してOK
    // i2c_set_speed(I2C_SPEED_FAST); // 400kHz
#endif

    // 起動直後に「現在レイヤー」を一度送っておく（Core2の初期表示安定）
    ete_on_layer(ete_get_active_layer());

    uprintf("[ETE] init done.\n");
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

static float smooth_x = 0;
static float smooth_y = 0;

int vx = report.x;
int vy = report.y;

float dead = 2.0f;


// ===== ① HOLDスクロールモード：XYをVHへ変換 =====
if (ete_get_scroll_hold()) {
    // 2点タッチ等で report.v/h が既に入ってる場合があるので「加算」にするのが安全
    report.h += vx;
    report.v += vy;

    // カーソルは完全停止
    report.x = 0;
    report.y = 0;

    // ★クリック無効
    report.buttons = 0;

    // ★ ローカルも止める（これが重要）
    vx = 0;
    vy = 0;
}

// なだらかデッドゾーン
if (fabs(vx) < dead) vx = 0;
if (fabs(vy) < dead) vy = 0;

int speed = abs(vx) + abs(vy);

// 連続カーブ
float s = speed / 32.0f;
float scale = 0.18f + (s * s * 0.75f);

// 微小速度ブースト（scaleを先に補正する）
if (speed > 0 && speed < 5) {
    scale += 0.05f * (5 - speed);
}

if (scale > 1.6f) scale = 1.6f;


float cursor_scale = (float)cursor_speed / 100.0f;
float target_x = vx * scale * cursor_scale;
float target_y = vy * scale * cursor_scale;

// ===== 速度依存スムージング（重要） =====
float alpha;

if (speed < 3) {
    alpha = 0.85f;   // ★ 超精密域：ほぼダイレクト
}
else if (speed < 8) {
    alpha = 0.60f;   // 精密操作域
}
else if (speed < 18) {
    alpha = 0.40f;   // 中速
}
else {
    alpha = 0.25f;   // 高速
}

smooth_x = smooth_x * (1.0f - alpha) + target_x * alpha;
smooth_y = smooth_y * (1.0f - alpha) + target_y * alpha;

// ===== ソフト停止（最後にやる） =====
if (speed == 0) {
    smooth_x *= 0.85f;
    smooth_y *= 0.85f;

    if (fabs(smooth_x) < 0.25f) smooth_x = 0;
    if (fabs(smooth_y) < 0.25f) smooth_y = 0;
}

report.x = (int)smooth_x;
report.y = (int)smooth_y;

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

    if (abs(sv) < 2) sv = 0;
    if (abs(sh) < 2) sh = 0;

    int scroll_speed = abs(sv) + abs(sh);

    // ==============================
    // 連続ヌルヌルカーブ
    // ==============================

    float scroll_scale;

    // 低速：超精密
    if (scroll_speed < 6) {
        scroll_scale = 0.025f;
    }
    // 中速：一番気持ちいいゾーン
    else if (scroll_speed < 18) {
        float t = (scroll_speed - 6) / 12.0f;
        scroll_scale = 0.025f + t * 0.22f;
    }
    // 高速：伸びるが暴れない
    else {
        float t = (scroll_speed - 18) / 40.0f;
        scroll_scale = 0.22f + (t * 0.25f);
    }

    if (scroll_scale > 0.7f) scroll_scale = 0.7f;

    float out_sv = sv * scroll_scale;
    float out_sh = sh * scroll_scale;

    float sp = (float)ete_get_scroll_speed() / 100.0f;
    out_sv *= sp;
    out_sh *= sp;

    // ==============================
    // 中速域に気持ちよさを作る
    // ==============================
    if (scroll_speed > 6 && scroll_speed < 18) {
        out_sv *= 1.2f;
        out_sh *= 1.2f;
    }

    // ==============================
    // サブノッチ蓄積
    // ==============================
    scroll_accum_v += out_sv;
    scroll_accum_h += out_sh;

    int final_v = (int)scroll_accum_v;
    int final_h = (int)scroll_accum_h;

    scroll_accum_v -= final_v;
    scroll_accum_h -= final_h;

    //スクロールが出ているフレームではクリックを無効化。   
    if (report.v != 0 || report.h != 0) {
    report.buttons = 0;
    }

    report.v = final_v;
    report.h = final_h;
    return report;
}



report_mouse_t pointing_device_task_user(report_mouse_t report)
{
    return ete_pointing_tune(report);
}



bool ete_get_swap_state(void) {
    return swap_lr;
}

void ete_toggle_lr(void) {
    swap_lr = !swap_lr;

    uint32_t val = eeconfig_read_kb();
    if (swap_lr) {
        val |= 0x01;
    } else {
        val &= ~0x01;
    }
    eeconfig_update_kb(val);
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
    if (!is_keyboard_master()) return;
    if (scroll_speed < 200) scroll_speed += 5;   // 好みで刻み変更
}

void ete_scroll_speed_dec(void) {
    if (!is_keyboard_master()) return;
    if (scroll_speed > 30) scroll_speed -= 5;    // 下限
}

void ete_scroll_settings_save(void) {
    if (!is_keyboard_master()) return;

    uint32_t val = eeconfig_read_kb();

    // swap_lr(bit0)は保持、scroll_speed(bits8..1)だけ上書き
    val &= ~ETE_EECONF_SCROLLSPD_MASK;
    val |= ((uint32_t)scroll_speed << ETE_EECONF_SCROLLSPD_SHIFT) & ETE_EECONF_SCROLLSPD_MASK;

    eeconfig_update_kb(val);
}

void ete_cursor_speed_inc(void) {
    if (!is_keyboard_master()) return;
    if (cursor_speed < 200) cursor_speed += 5;
}

void ete_cursor_speed_dec(void) {
    if (!is_keyboard_master()) return;
    if (cursor_speed > 50) cursor_speed -= 5;
}

uint8_t ete_get_cursor_speed(void) {
    return cursor_speed;
}

void ete_cursor_settings_save(void) {
    uint32_t val = eeconfig_read_kb();

    val &= ~ETE_EECONF_CURSORSPD_MASK;
    val |= ((uint32_t)cursor_speed << ETE_EECONF_CURSORSPD_SHIFT)
           & ETE_EECONF_CURSORSPD_MASK;

    eeconfig_update_kb(val);
}

void ete_settings_save(void) {
    if (!is_keyboard_master()) return;

    uint32_t val = eeconfig_read_kb();

    // scroll speed
    val &= ~ETE_EECONF_SCROLLSPD_MASK;
    val |= ((uint32_t)scroll_speed << ETE_EECONF_SCROLLSPD_SHIFT)
           & ETE_EECONF_SCROLLSPD_MASK;

    // cursor speed
    val &= ~ETE_EECONF_CURSORSPD_MASK;
    val |= ((uint32_t)cursor_speed << ETE_EECONF_CURSORSPD_SHIFT)
           & ETE_EECONF_CURSORSPD_MASK;
    
    uprintf("SAVE scroll=%d cursor=%d\n", scroll_speed, cursor_speed);

    eeconfig_update_kb(val);
}