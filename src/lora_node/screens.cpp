// v2 OLED screens — see screens.h and docs/SCREENS.md
#include "screens.h"
#include <stdio.h>
#include <string.h>

namespace {

const uint8_t* const F_BIG = u8g2_font_logisoso16_tr;   // FISH ON
const uint8_t* const F_NAME = u8g2_font_helvB12_tr;     // hole name on the alert screen
const uint8_t* const F_MED = u8g2_font_6x10_tr;         // normal text
const uint8_t* const F_BOLD = u8g2_font_6x12_tr;        // headers (u8g2 has no bold 6x10; 6x12 reads stronger)
const uint8_t* const F_SMALL = u8g2_font_5x8_tr;        // details, footer

void center(u8g2_t* u, int y, const char* s) {
  const int w = u8g2_GetStrWidth(u, s);
  u8g2_DrawStr(u, (128 - w) / 2 < 0 ? 0 : (128 - w) / 2, y, s);
}
void right(u8g2_t* u, int y, const char* s) { u8g2_DrawStr(u, 128 - u8g2_GetStrWidth(u, s), y, s); }

// Fits s into max_w pixels with the current font (cuts the end).
void fit(u8g2_t* u, char* s, int max_w) {
  size_t n = strlen(s);
  while (n > 1 && u8g2_GetStrWidth(u, s) > max_w) s[--n] = 0;
}

void holeName(const ScrHole& h, char* out, size_t n) {
  if (h.name[0]) snprintf(out, n, "%s", h.name);
  else snprintf(out, n, "Hole %u", h.id);
}

// 7x7 state symbol at (x, y = top): filled = OK, hollow = offline, X = fault, half = low battery, filled+blink = fish
void symbol(u8g2_t* u, int x, int y, uint8_t st, bool blink, int sz = 7) {
  switch (st) {
    case SH_OK: u8g2_DrawBox(u, x, y, sz, sz); break;
    case SH_FISH: if (blink) u8g2_DrawBox(u, x - 1, y - 1, sz + 2, sz + 2); else u8g2_DrawFrame(u, x - 1, y - 1, sz + 2, sz + 2); break;
    case SH_OFFLINE: u8g2_DrawFrame(u, x, y, sz, sz); break;
    case SH_FAULT: u8g2_DrawFrame(u, x, y, sz, sz); u8g2_DrawLine(u, x, y, x + sz - 1, y + sz - 1); u8g2_DrawLine(u, x + sz - 1, y, x, y + sz - 1); break;
    case SH_LOWBAT: u8g2_DrawFrame(u, x, y, sz, sz); u8g2_DrawBox(u, x, y + sz / 2, sz, sz - sz / 2); break;
  }
}

// page dots bottom right: which of the 4 pages is shown
void pageDots(u8g2_t* u, uint8_t page) {
  for (uint8_t i = 0; i < PG_COUNT; i++) {
    const int x = 128 - (PG_COUNT - i) * 6, y = 59;
    if (i == page) u8g2_DrawBox(u, x, y, 4, 4); else u8g2_DrawFrame(u, x, y, 4, 4);
  }
}

void mmss(char* out, size_t n, uint32_t s) {
  if (s >= 3600) snprintf(out, n, "%luh%02lu", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60));
  else snprintf(out, n, "%lu:%02lu", (unsigned long)(s / 60), (unsigned long)(s % 60));
}

const char* trName(uint8_t t) { return t == 1 ? "LoRa only" : t == 2 ? "ESP-NOW only" : "Auto"; }

// ---- alert: whole screen, blinking, readable from across the room ----
void drawAlert(u8g2_t* u, const ScreenModel& m, bool blink) {
  int first = -1, n = 0;
  uint32_t newest = 0xFFFFFFFF;
  for (uint8_t i = 0; i < m.n_holes; i++)
    if (m.holes[i].state == SH_FISH) { n++; if (m.holes[i].since_s < newest) { newest = m.holes[i].since_s; first = i; } }
  if (blink) { u8g2_DrawBox(u, 0, 0, 128, 64); u8g2_SetDrawColor(u, 0); }
  u8g2_SetFont(u, F_BIG); center(u, 18, "FISH ON");
  const ScrHole& h = m.holes[first];
  char name[24]; holeName(h, name, sizeof(name));
  u8g2_SetFont(u, F_NAME); fit(u, name, 126); center(u, 38, name);
  char line[48], t[12]; mmss(t, sizeof(t), h.since_s);
  u8g2_SetFont(u, F_MED);
  if (h.hub == h.id) snprintf(line, sizeof(line), "Hub %u hole  %s", h.hub, t);
  else snprintf(line, sizeof(line), "Pocket %u  %s", h.hub, t);
  center(u, 50, line);
  u8g2_SetFont(u, F_SMALL);
  if (n > 1) snprintf(line, sizeof(line), "+%d more  Button: silence", n - 1);
  else snprintf(line, sizeof(line), "Button: silence");
  center(u, 62, line);
  u8g2_SetDrawColor(u, 1);
}

// header: left text bold, right small text, line under it
void header(u8g2_t* u, const char* left, const char* rt) {
  u8g2_SetFont(u, F_BOLD); u8g2_DrawStr(u, 0, 10, left);
  if (rt) { u8g2_SetFont(u, F_SMALL); right(u, 9, rt); }
  u8g2_DrawHLine(u, 0, 13, 128);
}

void summary(const ScreenModel& m, char* out, size_t n, uint8_t hub_filter) {
  int tot = 0, off = 0, bad = 0, low = 0;
  for (uint8_t i = 0; i < m.n_holes; i++) {
    const ScrHole& h = m.holes[i];
    if (hub_filter && h.hub != hub_filter) continue;
    tot++;
    if (h.state == SH_OFFLINE) off++; else if (h.state == SH_FAULT) bad++; else if (h.state == SH_LOWBAT) low++;
  }
  if (m.setup_s) snprintf(out, n, "SETUP %lu min", (unsigned long)((m.setup_s + 59) / 60));
  else if (!off && !bad && !low) snprintf(out, n, "%d holes OK", tot);
  else if (off) snprintf(out, n, "%d/%d online", tot - off, tot);
  else if (bad) snprintf(out, n, "%d sensor fault", bad);
  else snprintf(out, n, "%d low battery", low);
}

void footer(u8g2_t* u, const ScreenModel& m, uint8_t page) {
  u8g2_SetFont(u, F_SMALL);
  char line[40];
  if (m.silenced) { char t[12]; mmss(t, sizeof(t), m.silence_s); snprintf(line, sizeof(line), "SILENCED %s", t); }
  else if (m.chalet) snprintf(line, sizeof(line), "%s", m.sta ? m.url : m.ssid);
  else if (m.hotspot) snprintf(line, sizeof(line), "Hotspot on %lu min", (unsigned long)m.hotspot_min);
  else snprintf(line, sizeof(line), "Hold 3s: hotspot");
  fit(u, line, 104);
  u8g2_DrawStr(u, 0, 63, line);
  pageDots(u, page);
}

// ---- home, chalet: one row per pocket (two columns above 4 pockets) ----
void homeChalet(u8g2_t* u, const ScreenModel& m, bool blink) {
  char s[32], ch[16];
  summary(m, s, sizeof(s), 0);
  snprintf(ch, sizeof(ch), "ch%u%s", m.lora_ch, m.transport == 2 ? " ESPNOW" : "");
  header(u, s, ch);
  // pockets = distinct hub IDs in hole order
  uint8_t pk[10]; uint8_t np = 0;
  for (uint8_t i = 0; i < m.n_holes && np < 10; i++) {
    bool seen = false;
    for (uint8_t k = 0; k < np; k++) seen |= pk[k] == m.holes[i].hub;
    if (!seen) pk[np++] = m.holes[i].hub;
  }
  if (np == 0) { u8g2_SetFont(u, F_MED); center(u, 36, m.setup_s ? "Waiting for hubs..." : "No hub heard"); return; }
  const bool two = np > 4;
  const int rows = two ? (np + 1) / 2 : np;
  const int rh = rows <= 4 ? 11 : 8;
  const int sz = rows <= 4 ? 7 : 6;
  for (uint8_t p = 0; p < np; p++) {
    const int col = two ? p / rows : 0, row = two ? p % rows : p;
    const int x0 = col * 64, y0 = 16 + row * rh;
    u8g2_SetFont(u, F_SMALL);
    snprintf(s, sizeof(s), "H%u", pk[p]);
    u8g2_DrawStr(u, x0, y0 + sz, s);
    int x = x0 + (two ? 17 : 14), off = 0, n = 0;
    for (uint8_t i = 0; i < m.n_holes; i++) {
      if (m.holes[i].hub != pk[p]) continue;
      n++;
      if (m.holes[i].state == SH_OFFLINE) off++;
      if (x <= x0 + (two ? 56 : 80)) symbol(u, x, y0 + 1, m.holes[i].state, blink, sz);
      x += sz + 3;
    }
    if (!two) {
      // link of that pocket, at the right
      const char* lk = "";
      for (uint8_t k = 0; k < m.n_hubs; k++) if (m.hubs[k].id == pk[p]) lk = m.hubs[k].lora_ok ? "" : (m.hubs[k].eb_ok ? "ESP-NOW" : "LOST");
      if (lk[0]) snprintf(s, sizeof(s), "%s", lk);
      else if (off) snprintf(s, sizeof(s), "%d off", off);
      else snprintf(s, sizeof(s), "OK");
      right(u, y0 + 7, s);
    }
  }
}

// ---- home, hub: its pocket, and whether it reaches the master ----
void homeHub(u8g2_t* u, const ScreenModel& m, bool blink) {
  char s[32], ch[16];
  snprintf(s, sizeof(s), "HUB %u", m.self_id);
  snprintf(ch, sizeof(ch), "ch%u", m.lora_ch);
  header(u, s, ch);
  u8g2_SetFont(u, F_MED);
  if (m.setup_s && !m.master_heard) snprintf(s, sizeof(s), "Waiting for chalet");
  else if (!m.master_heard) snprintf(s, sizeof(s), m.on_backup ? "Chalet via ESP-NOW" : "No chalet signal");
  else if (m.via_echo) snprintf(s, sizeof(s), "Chalet via relay");
  else snprintf(s, sizeof(s), "Chalet OK %ddBm", m.beacon_rssi);
  u8g2_DrawStr(u, 0, 24, s);
  int y = 26;
  for (uint8_t i = 0; i < m.n_holes && y < 50; i++) {
    const ScrHole& h = m.holes[i];
    if (h.hub != m.self_id) continue;
    symbol(u, 0, y + 2, h.state, blink);
    char nm[24]; holeName(h, nm, sizeof(nm));
    u8g2_SetFont(u, F_SMALL);
    char b[8]; if (h.batt <= 100) snprintf(b, sizeof(b), "%u%%", h.batt); else snprintf(b, sizeof(b), "-");
    fit(u, nm, 80);
    u8g2_DrawStr(u, 11, y + 8, nm);
    right(u, y + 8, h.state == SH_OFFLINE ? "off" : b);
    y += 9;
  }
}

// ---- holes list, 4 per page ----
void holesPage(u8g2_t* u, const ScreenModel& m, uint8_t sub, bool blink) {
  const uint8_t pages = screenHolesPages(m);
  if (sub >= pages) sub = 0;
  char s[32];
  snprintf(s, sizeof(s), "Holes");
  char r[12]; snprintf(r, sizeof(r), "%u/%u", sub + 1, pages);
  header(u, s, r);
  int y = 16;
  for (uint8_t k = 0; k < 4; k++) {
    const int i = sub * 4 + k;
    if (i >= m.n_holes) break;
    const ScrHole& h = m.holes[i];
    symbol(u, 0, y + 1, h.state, blink);
    char nm[24]; holeName(h, nm, sizeof(nm));
    u8g2_SetFont(u, F_MED);
    fit(u, nm, 70);
    u8g2_DrawStr(u, 10, y + 8, nm);
    u8g2_SetFont(u, F_SMALL);
    char d[20];
    if (h.state == SH_OFFLINE) snprintf(d, sizeof(d), "offline");
    else if (h.fish >= 0 && h.batt <= 100) snprintf(d, sizeof(d), "%df %u%%", h.fish, h.batt);
    else if (h.batt <= 100) snprintf(d, sizeof(d), "%u%%", h.batt);
    else snprintf(d, sizeof(d), "-");
    right(u, y + 8, d);
    y += 11;
  }
}

// ---- network ----
void networkPage(u8g2_t* u, const ScreenModel& m) {
  header(u, "Network", m.setup_s ? "setup" : "running");
  char s[40];
  u8g2_SetFont(u, F_SMALL);
  if (m.setup_s) snprintf(s, sizeof(s), "Fallbacks arm in %lu min", (unsigned long)((m.setup_s + 59) / 60));
  else snprintf(s, sizeof(s), "Link: %s", trName(m.transport));
  u8g2_DrawStr(u, 0, 22, s);
  char busy[8]; if (m.busy_pct <= 100) snprintf(busy, sizeof(busy), "%u%%", m.busy_pct); else snprintf(busy, sizeof(busy), "-");
  snprintf(s, sizeof(s), "LoRa ch%u %u.%u MHz %s %s", m.lora_ch, m.lora_mhz_x10 / 10, m.lora_mhz_x10 % 10, m.ch_auto ? "Auto" : "fixed", m.chalet ? busy : "");
  u8g2_DrawStr(u, 0, 31, s);
  if (m.chalet) {
    int x = 0, y = 41;
    for (uint8_t k = 0; k < m.n_hubs; k++) {
      const ScrHubLink& h = m.hubs[k];
      if (h.lora_ok) snprintf(s, sizeof(s), "H%u %d", h.id, h.lora_rssi);
      else if (h.eb_ok) snprintf(s, sizeof(s), "H%u E%u", h.id, h.eb_hops);
      else snprintf(s, sizeof(s), "H%u --", h.id);
      u8g2_DrawStr(u, x, y, s);
      x += 42; if (x > 100) { x = 0; y += 9; }
      if (y > 58) break;
    }
  } else {
    snprintf(s, sizeof(s), "ESP-NOW backup: %s", m.on_backup ? "IN USE" : "standby");
    u8g2_DrawStr(u, 0, 41, s);
    snprintf(s, sizeof(s), "Up %luh%02lu", (unsigned long)(m.uptime_s / 3600), (unsigned long)(m.uptime_s / 60 % 60));
    u8g2_DrawStr(u, 0, 50, s);
  }
  pageDots(u, PG_NETWORK);
}

// ---- connect ----
void connectPage(u8g2_t* u, const ScreenModel& m) {
  header(u, "Connect phone", nullptr);
  u8g2_SetFont(u, F_MED);
  char s[48];
  if (!m.chalet && !m.hotspot) {
    u8g2_DrawStr(u, 0, 28, "Hotspot is off.");
    u8g2_DrawStr(u, 0, 42, "Hold button 3 s");
    pageDots(u, PG_CONNECT);
    return;
  }
  if (m.sta) {
    u8g2_DrawStr(u, 0, 26, "On the cabin Wi-Fi:");
    snprintf(s, sizeof(s), "%s", m.url); fit(u, s, 128); u8g2_DrawStr(u, 0, 38, s);
    u8g2_SetFont(u, F_SMALL);
    snprintf(s, sizeof(s), "or Wi-Fi %s", m.ssid); fit(u, s, 128); u8g2_DrawStr(u, 0, 50, s);
  } else {
    snprintf(s, sizeof(s), "%s", m.ssid); fit(u, s, 128); u8g2_DrawStr(u, 0, 26, s);
    snprintf(s, sizeof(s), "pass %s", m.pass); u8g2_DrawStr(u, 0, 38, s);
    snprintf(s, sizeof(s), "%s", m.url); fit(u, s, 128); u8g2_DrawStr(u, 0, 50, s);
  }
  pageDots(u, PG_CONNECT);
}

// rows used by the chalet home (pockets, two columns above 4)
uint8_t homeRows(const ScreenModel& m) {
  if (!m.chalet) return 0;
  uint8_t pk[10]; uint8_t np = 0;
  for (uint8_t i = 0; i < m.n_holes && np < 10; i++) {
    bool seen = false;
    for (uint8_t k = 0; k < np; k++) seen |= pk[k] == m.holes[i].hub;
    if (!seen) pk[np++] = m.holes[i].hub;
  }
  return np > 4 ? static_cast<uint8_t>((np + 1) / 2) : np;
}

}  // namespace

uint8_t screenHolesPages(const ScreenModel& m) { return m.n_holes ? static_cast<uint8_t>((m.n_holes + 3) / 4) : 1; }

void screenDraw(u8g2_t* u, const ScreenModel& m, uint8_t page, uint8_t sub, bool blink) {
  u8g2_ClearBuffer(u);
  u8g2_SetDrawColor(u, 1);
  u8g2_SetFontMode(u, 1);
  bool fish = false;
  for (uint8_t i = 0; i < m.n_holes; i++) fish |= m.holes[i].state == SH_FISH;
  if (fish && !m.silenced) { drawAlert(u, m, blink); u8g2_SendBuffer(u); return; }
  switch (page) {
    case PG_HOLES: holesPage(u, m, sub, blink); pageDots(u, page); break;
    case PG_NETWORK: networkPage(u, m); break;
    case PG_CONNECT: connectPage(u, m); break;
    default:
      if (m.chalet) homeChalet(u, m, blink); else homeHub(u, m, blink);
      if (homeRows(m) <= 4) footer(u, m, PG_HOME); else pageDots(u, PG_HOME);   // 5 pocket rows: no room for the footer
      break;
  }
  u8g2_SendBuffer(u);
}
