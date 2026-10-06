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
const uint8_t* const F_TINY = u8g2_font_tom_thumb_4x6_tr; // sonar labels

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
    const int x = 128 - (PG_COUNT - i) * 5, y = 60;
    if (i == page) u8g2_DrawBox(u, x, y, 3, 3); else u8g2_DrawPixel(u, x + 1, y + 1);
  }
}

// depth text in the user's unit
void depthStr(char* out, size_t n, uint16_t cm, bool feet) {
  if (feet) { const unsigned ft10 = (cm * 10u + 15u) / 30u; snprintf(out, n, "%u.%uft", ft10 / 10, ft10 % 10); }   // 1 ft = 30.48 cm
  else snprintf(out, n, "%u.%um", cm / 100, cm % 100 / 10);
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
  const char* act = screenPageAction(m, page);
  const bool scroll = m.kb && ((page == PG_HOLES && screenHolesPages(m) > 1) || (page == PG_SONAR && screenSonarPages(m) > 1));
  if (scroll) snprintf(line, sizeof(line), "^v: more holes");
  else if (act[0]) snprintf(line, sizeof(line), "%s %s", m.kb ? "OK:" : "2x:", act);
  else if (m.silenced) { char t[12]; mmss(t, sizeof(t), m.silence_s); snprintf(line, sizeof(line), "SILENCED %s", t); }
  else if (m.chalet && m.sta) { const char* ip = strncmp(m.url, "http://", 7) == 0 ? m.url + 7 : m.url; snprintf(line, sizeof(line), "%s", ip); }
  else if (m.chalet) snprintf(line, sizeof(line), "%s", (m.sta_ssid[0] && (m.uptime_s / 3) % 2) ? "No cabin Wi-Fi" : m.ssid);   // alternates every 3 s
  else if (m.hotspot) snprintf(line, sizeof(line), "Hotspot on %lu min", (unsigned long)m.hotspot_min);
  else snprintf(line, sizeof(line), "Hold 3s: hotspot");
  fit(u, line, 128 - PG_COUNT * 5 - 2);
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
  int y = 15;
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
    y += 10;
  }
}

// ---- sonar at a glance: one depth column per sonar hole, common depth scale ----
//   dithered = bottom, bar = fish (wider = stronger), short tick at the left = bait
void sonarPage(u8g2_t* u, const ScreenModel& m, uint8_t sub, bool blink) {
  int idx[48], n = 0;
  for (uint8_t i = 0; i < m.n_holes; i++) if (m.holes[i].son.valid) idx[n++] = i;
  const int per = 9, pages = n ? (n + per - 1) / per : 1;
  if (sub >= pages) sub = 0;
  uint16_t range = 300;
  for (int k = 0; k < n; k++) if (m.holes[idx[k]].son.bottom_cm + 30 > range) range = m.holes[idx[k]].son.bottom_cm + 30;
  char r[16], d[12]; depthStr(d, sizeof(d), range, m.feet); snprintf(r, sizeof(r), "0-%s", d);
  char h[24]; if (pages > 1) snprintf(h, sizeof(h), "Sonar %d/%d", sub + 1, pages); else snprintf(h, sizeof(h), "Sonar");
  header(u, h, r);
  if (!n) { u8g2_SetFont(u, F_MED); center(u, 38, "No sonar hole"); return; }
  const int y0 = 15, y1 = 55, H = y1 - y0;
  for (int c = 0; c < per && sub * per + c < n; c++) {
    const ScrHole& ho = m.holes[idx[sub * per + c]];
    const ScrSonar& so = ho.son;
    const int x = c * 14 + 1, w = 12;
    u8g2_DrawVLine(u, x + w, y0, H);                                   // column separator
    if (ho.state == SH_FISH && blink) u8g2_DrawFrame(u, x - 1, y0 - 1, w + 2, H + 2);
    if (so.bottom_cm) {
      const int yb = y0 + so.bottom_cm * H / range;
      u8g2_DrawHLine(u, x, yb, w);
      for (int y = yb + 1; y <= y1; y++) for (int xx = x; xx < x + w; xx++) if (((xx + y) & 1) == 0) u8g2_DrawPixel(u, xx, y);
    }
    for (uint8_t k = 0; k < so.n; k++) {
      const int y = y0 + so.t[k].depth_cm * H / range;
      if (so.t[k].bait) { u8g2_DrawHLine(u, x, y, 3); continue; }
      const int bw = so.t[k].level >= 3 ? 10 : so.t[k].level == 2 ? 7 : 4;
      u8g2_DrawBox(u, x + (w - bw) / 2, y - 1, bw, 2);
    }
    char lab[8];
    if (ho.name[0]) snprintf(lab, sizeof(lab), "%.3s", ho.name); else snprintf(lab, sizeof(lab), "%u", ho.id);
    u8g2_SetFont(u, F_TINY);
    u8g2_DrawStr(u, x + (w - u8g2_GetStrWidth(u, lab)) / 2, 63, lab);
  }
}

// ---- FOCUS: classic mono fish finder, newest ping at the right ----
void focusPage(u8g2_t* u, const ScreenModel& m) {
  const ScrHole* fh = nullptr;
  for (uint8_t i = 0; i < m.n_holes; i++) if (m.holes[i].id == m.focus_node) fh = &m.holes[i];
  char h[24], r[12];
  if (!fh || m.n_cols == 0) {
    header(u, "Focus", nullptr);
    u8g2_SetFont(u, F_MED); center(u, 34, fh ? "Waiting for data" : "No focus hole");
    u8g2_SetFont(u, F_SMALL); center(u, 50, m.kb ? "Up/down: choose hole" : "Double press: choose");
    return;
  }
  char nm[20]; holeName(*fh, nm, sizeof(nm));
  snprintf(h, sizeof(h), "%s", nm);
  const ScrPingCol& last = m.cols[m.n_cols - 1];
  depthStr(r, sizeof(r), last.bottom_cm, m.feet);
  u8g2_SetFont(u, F_BOLD); fit(u, h, 80); u8g2_DrawStr(u, 0, 10, h);
  u8g2_SetFont(u, F_SMALL); right(u, 9, r);
  u8g2_DrawHLine(u, 0, 12, 128);
  uint16_t range = 300;   // deepest bottom in view + 20 %, so the ground shows
  for (uint8_t c = 0; c < m.n_cols; c++) if (m.cols[c].bottom_cm * 6u / 5u > range) range = static_cast<uint16_t>(m.cols[c].bottom_cm * 6u / 5u);
  const int y0 = 14, y1 = 63, H = y1 - y0, W = 118;
  const int x0 = W - m.n_cols;
  for (uint8_t c = 0; c < m.n_cols; c++) {
    const ScrPingCol& p = m.cols[c];
    const int x = x0 + c;
    if (p.bottom_cm) {
      const int yb = y0 + p.bottom_cm * H / range;
      u8g2_DrawPixel(u, x, yb); u8g2_DrawPixel(u, x, yb + 1);
      for (int y = yb + 2; y <= y1; y++) if (((x + y) & 1) == 0 && ((y - yb) < 6 || (y & 1))) u8g2_DrawPixel(u, x, y);
    }
    for (uint8_t k = 0; k < p.n; k++) {
      const int y = y0 + p.d[k] * H / range;
      if (p.lv[k] >= 2) u8g2_DrawBox(u, x, y - 1, 1, p.lv[k] >= 3 ? 3 : 2); else u8g2_DrawPixel(u, x, y);
    }
  }
  if (m.bait_cm) { const int yb = y0 + m.bait_cm * H / range; for (int x = 0; x < W; x += 6) u8g2_DrawPixel(u, x, yb); }   // bait depth: dotted
  // depth scale at the right
  u8g2_SetFont(u, F_TINY);
  char d[10];
  depthStr(d, sizeof(d), range / 2, m.feet); for (char* q = d; *q; q++) if (*q == '.') { *q = 0; break; }
  u8g2_DrawStr(u, W + 1, y0 + H / 2 + 3, d);
  depthStr(d, sizeof(d), range, m.feet); for (char* q = d; *q; q++) if (*q == '.') { *q = 0; break; }
  u8g2_DrawStr(u, W + 1, y1, d);
}

// ---- test / simulation ----
void testPage(u8g2_t* u, const ScreenModel& m) {
  int son = 0, hall = 0, any = 0;
  for (uint8_t i = 0; i < m.n_holes; i++) { son += (m.holes[i].sim & 1) != 0; hall += (m.holes[i].sim & 2) != 0; any += m.holes[i].sim != 0; }
  header(u, "Test", any ? "SIM ON" : "sim off");
  u8g2_SetFont(u, F_MED);
  char s[40];
  snprintf(s, sizeof(s), "Sonar sim: %d hole%s", son, son == 1 ? "" : "s"); u8g2_DrawStr(u, 0, 25, s);
  snprintf(s, sizeof(s), "Hall sim:  %d hole%s", hall, hall == 1 ? "" : "s"); u8g2_DrawStr(u, 0, 37, s);
  snprintf(s, sizeof(s), "Radio test: %s", m.radio_test ? (m.radio_test_name ? m.radio_test_name : "on") : "off"); u8g2_DrawStr(u, 0, 49, s);
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
  char s[64];
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
    snprintf(s, sizeof(s), "%s", m.ssid); fit(u, s, 128); u8g2_DrawStr(u, 0, 25, s);
    snprintf(s, sizeof(s), "pass %s", m.pass); u8g2_DrawStr(u, 0, 36, s);
    snprintf(s, sizeof(s), "%s", m.url); fit(u, s, 128); u8g2_DrawStr(u, 0, 47, s);
  }
  // why there is no cabin address (chalet), or how to change the Wi-Fi
  u8g2_SetFont(u, F_SMALL);
  if (m.chalet && !m.sta) {
    if (m.sta_ssid[0]) snprintf(s, sizeof(s), "%s: %s", m.sta_ssid, m.sta_state);
    else snprintf(s, sizeof(s), "No cabin Wi-Fi set");
    fit(u, s, 128); u8g2_DrawStr(u, 0, 57, s);
  } else if (m.chalet && m.kb) {
    u8g2_DrawStr(u, 0, 63, "OK: Wi-Fi options");
  }
  pageDots(u, PG_CONNECT);
}

// ---- options: list / text entry / yes-no (content decided by the firmware) ----
void optionsPage(u8g2_t* u, const ScreenModel& m, bool blink) {
  const ScrOptions& o = m.opt;
  char s[72];
  if (o.mode == SO_TEXT) {
    header(u, o.title, nullptr);
    u8g2_SetFont(u, F_SMALL);
    snprintf(s, sizeof(s), "%s", o.line); fit(u, s, 128); u8g2_DrawStr(u, 0, 23, s);
    u8g2_DrawFrame(u, 0, 26, 128, 15);
    u8g2_SetFont(u, F_MED);
    // the end of the text that fits, cursor at the end
    const char* t = o.text;
    while (*t && u8g2_GetStrWidth(u, t) > 116) t++;
    u8g2_DrawStr(u, 3, 37, t);
    if (blink) u8g2_DrawHLine(u, 3 + u8g2_GetStrWidth(u, t) + 1, 38, 5);
    u8g2_SetFont(u, F_SMALL);
    snprintf(s, sizeof(s), "%s", o.hint); fit(u, s, 128); u8g2_DrawStr(u, 0, 52, s);
    u8g2_DrawStr(u, 0, 62, "Esc: back   Del: erase");
    return;
  }
  if (o.mode == SO_CONFIRM) {
    header(u, o.title, nullptr);
    u8g2_SetFont(u, F_MED);
    snprintf(s, sizeof(s), "%s", o.line); fit(u, s, 128); center(u, 34, s);
    u8g2_SetFont(u, F_SMALL);
    center(u, 54, "OK = yes     Esc = no");
    return;
  }
  char r[12]; snprintf(r, sizeof(r), "%u/%u", o.n ? o.sel + 1 : 0, o.n);
  header(u, o.title, r);
  const int VIS = 4, y0 = 14, rh = 10;   // 4 rows 14..54, footer below
  int top = o.sel >= VIS ? o.sel - VIS + 1 : 0;
  for (int k = 0; k < VIS && top + k < o.n; k++) {
    const ScrRow& row = o.rows[top + k];
    const int y = y0 + k * rh;
    const bool sel = top + k == o.sel;
    if (sel) { u8g2_DrawBox(u, 0, y, 128, rh); u8g2_SetDrawColor(u, 0); }
    u8g2_SetFont(u, F_MED);
    char v[24]; snprintf(v, sizeof(v), "%s%s", row.value, row.sub ? " >" : "");
    const int vw = u8g2_GetStrWidth(u, v);
    snprintf(s, sizeof(s), "%s", row.label); fit(u, s, 128 - vw - 6);
    u8g2_DrawStr(u, 2, y + 8, s);
    u8g2_DrawStr(u, 126 - vw, y + 8, v);
    u8g2_SetDrawColor(u, 1);
  }
  u8g2_SetFont(u, F_SMALL);
  snprintf(s, sizeof(s), "%s", o.hint); fit(u, s, 128 - PG_COUNT * 5 - 2);
  u8g2_DrawStr(u, 0, 63, s);
  pageDots(u, PG_OPTIONS);
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
uint8_t screenSonarPages(const ScreenModel& m) {
  int n = 0; for (uint8_t i = 0; i < m.n_holes; i++) n += m.holes[i].son.valid;
  return n ? static_cast<uint8_t>((n + 8) / 9) : 1;
}
const char* screenPageAction(const ScreenModel& m, uint8_t page) {
  switch (page) {
    case PG_HOLES: return screenHolesPages(m) > 1 ? "next 4 holes" : "";
    case PG_SONAR: return "";   // no footer room; paging by double press when > 9 holes
    case PG_FOCUS: return m.chalet ? "next hole" : "";
    case PG_TEST: {
      if (!m.chalet) return "";
      bool any = false; for (uint8_t i = 0; i < m.n_holes; i++) any |= m.holes[i].sim != 0;
      return any ? "sim all OFF" : "sim all ON";
    }
    default: return "";
  }
}

void screenDraw(u8g2_t* u, const ScreenModel& m, uint8_t page, uint8_t sub, bool blink) {
  u8g2_ClearBuffer(u);
  u8g2_SetDrawColor(u, 1);
  u8g2_SetFontMode(u, 1);
  bool fish = false;
  for (uint8_t i = 0; i < m.n_holes; i++) fish |= m.holes[i].state == SH_FISH;
  if (fish && !m.silenced) { drawAlert(u, m, blink); u8g2_SendBuffer(u); return; }
  switch (page) {
    case PG_HOLES: holesPage(u, m, sub, blink); footer(u, m, page); break;
    case PG_SONAR: sonarPage(u, m, sub, blink); break;
    case PG_FOCUS: focusPage(u, m); break;
    case PG_NETWORK: networkPage(u, m); break;
    case PG_TEST: testPage(u, m); footer(u, m, page); break;
    case PG_CONNECT: connectPage(u, m); break;
    case PG_OPTIONS: optionsPage(u, m, blink); break;
    default:
      if (m.chalet) homeChalet(u, m, blink); else homeHub(u, m, blink);
      if (homeRows(m) <= 4) footer(u, m, PG_HOME); else pageDots(u, PG_HOME);   // 5 pocket rows: no room for the footer
      break;
  }
  u8g2_SendBuffer(u);
}
