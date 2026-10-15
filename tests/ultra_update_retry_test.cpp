// auto_retry (components/dig2analog_ultra/update_retry.h) with the real bootloader logic: the component's
// UpdateSession frames go through the board firmware's bootloader and its flash / board model
// (bootloader/test/test_bl.c, as the firmware's sender_tests use it), and AutoRetry decides after each session, as the
// update entity does (fw_update.cpp session_done_ / CHECKING): blind on a one-way line, from the receivers' versions on
// a line of Diff-Power receivers. The app side of a repeated session (a board on the release stays lit) runs on the
// firmware's whole-board model: tests/ultra_telemetry_contract_test.cpp, sc_update_repeat.
//
// Needs the board firmware (ULTRA_FIRMWARE). Build and run: python tests/run_ultra_update_retry_test.py

#define BL_TEST_NO_MAIN
#include "test_bl.c"

#include "update_retry.h"
#include "update_session.h"

#include <cstdio>
#include <functional>
#include <string>

using namespace esphome::dig2analog_ultra;

static const uint32_t FRAME_US = 384u * 1350u / 1000u + 300u;   // the hub's worst bit (700 + 650 ns) + reset

/// The board's app decides on an ANNOUNCE itself (src/core/upd_app.c): it enters the bootloader only for an image
/// for its hardware that isn't the one it runs (the model of test_bl.c reboots on every ANNOUNCE).
static int app_would_take(const image_t *im) {
  const uint8_t *run = &fl[UPD_APP_DESC - FL_BASE];
  const int ok = upd_desc_check(im->desc, upd_crc32_words(0xFFFFFFFFu, im->desc, 15u), BL_HW_ID, BL_HW_REV,
                                BL_VERSION) == 0;
  return ok && upd_get32(run + UPD_D_FW_VERSION) != upd_get32(im->desc + UPD_D_FW_VERSION);
}

/// One session from the component's sender to the board; the board loses frames [drop_from, drop_to). Returns the
/// frames sent; *boots: the board's (re)boots during it.
static uint32_t send_session(const image_t *im, uint8_t session, uint32_t drop_from, uint32_t drop_to,
                             uint32_t *boots = nullptr) {
  static uint8_t img[UPD_DESC_BYTES + UPD_APP_BODY_MAX];
  UpdateSession us;
  UpdateSession::Options o;
  uint8_t f[UPD_FRAME_BYTES];
  uint32_t pause = 0, n = 0;
  const uint32_t boots0 = B.boots;
  memcpy(img, im->desc, UPD_DESC_BYTES);
  memcpy(img + UPD_DESC_BYTES, im->body, im->len);
  CHECK(us.begin(img, UPD_DESC_BYTES + im->len, session, o, FRAME_US), "begin");
  B.app_takes = app_would_take(im);
  while (us.next(f, static_cast<uint32_t>(B.now_us), &pause)) {
    loss_ppm = n >= drop_from && n < drop_to ? 1000000u : 0u;
    host_frame_bits(f, UPD_FRAME_BYTES, pause + 50u);
    n++;
  }
  loss_ppm = 0;
  if (boots != nullptr)
    *boots = B.boots - boots0;
  return n;
}

/// What the hub's telemetry makes of the board (a receiver): its app answers with its IDENTITY (the version it runs,
/// its bootloader's hardware), its bootloader doesn't answer.
static RxUpdate board_state(const image_t *target) {
  if (!B.in_app)
    return RXU_NO_ANSWER;
  ReceiverData d;
  d.have_identity = d.have_status = true;
  d.fw_word = upd_get32(&fl[UPD_APP_DESC - FL_BASE + UPD_D_FW_VERSION]);
  d.hw_id = BL_HW_ID;
  d.bl_version = BL_VERSION;
  d.hw = static_cast<uint8_t>((BL_HW_ID << 4) | BL_HW_REV);
  const UpdateTarget t{upd_get32(target->desc + UPD_D_FW_VERSION), target->desc[UPD_D_HW_ID],
                       static_cast<uint16_t>(upd_get16(target->desc + UPD_D_HW_REV_MASK)), target->desc[UPD_D_MIN_BL]};
  return receiver_update_state(d, true, t);
}

struct Install {
  int sessions{0};
  std::vector<std::string> texts;   // the status texts AutoRetry gave, in order
  uint32_t boots_last{0};           // the board's boots during the last session
};

/// An install as the update entity runs it: a session, the hub's quiet second / resync / PING burst (the result),
/// then AutoRetry: the session again (RETRY), the board's state every 500 ms (CHECK, verify), or the end. lose(k,
/// from, to): the frames session k loses on the board.
static Install install(const image_t *im, uint8_t retries, bool verify, AutoRetry &r,
                       const std::function<void(int, uint32_t &, uint32_t &)> &lose) {
  Install out;
  r.begin(retries, verify);
  const uint32_t version = upd_get32(im->desc + UPD_D_FW_VERSION);
  for (;;) {
    uint32_t from = 0, to = 0;
    lose(out.sessions, from, to);
    send_session(im, static_cast<uint8_t>(0x51 + 2 * out.sessions), from, to, &out.boots_last);
    out.sessions++;
    host_after();
    AutoRetry::Step s = r.session_done(ms());
    if (s == AutoRetry::CHECK)
      out.texts.push_back(r.checking_text(1));
    while (s == AutoRetry::CHECK) {
      host_idle(500000u);
      const RxUpdate st = board_state(im);
      s = r.check(ms(), &st, 1);
    }
    if (s == AutoRetry::FINISH) {
      out.texts.push_back(verify ? r.result_text(version) : "done");
      return out;
    }
    out.texts.push_back(r.retry_text(version));
  }
}

static const auto NO_LOSS = [](int, uint32_t &, uint32_t &) {};
/// session 0 loses everything after the lead-in (the board erased its app: it waits in its bootloader)
static const auto LOSE_FIRST = [](int k, uint32_t &from, uint32_t &to) {
  if (k == 0) {
    from = 700;
    to = 0xFFFFFFFFu;
  }
};
static const auto LOSE_ALL = [](int, uint32_t &from, uint32_t &to) {
  from = 700;
  to = 0xFFFFFFFFu;
};

static void fresh_board() {
  board_reset_model();
  make_image(&im_old, 8192, V(0, 6, 8, 0xFF), 1, REV_OK, 1, 301);
  make_image(&im_new, 8192, V(0, 6, 9, 0xFF), 1, REV_OK, 1, 302);
  factory(&im_old);
  board_power_on();
}

/// One-way line (blind): a board the first session left in its bootloader installs from the first retry; every
/// retry is sent (the hub can't see), and the board that runs the release doesn't take the last one (no reboot).
static void test_blind() {
  std::printf("blind (a one-way line), auto_retry 2\n");
  fresh_board();
  AutoRetry r;
  // the first session alone: the board erased its app and waits in its bootloader
  send_session(&im_new, 0x41, 700, 0xFFFFFFFFu);
  host_after();
  host_idle(4000000u);
  CHECK(!B.in_app && BS.mode == BL_ACT_RECOVERY, "frames lost after the lead-in: the bootloader waits (recovery)");
  // the install: session 1 lost as above, then the retries
  fresh_board();
  const Install in = install(&im_new, 2, false, r, LOSE_FIRST);
  CHECK(in.sessions == 3 && r.used() == 2 && running(&im_new), "3 sessions (1 + auto_retry 2), the release runs (%d)",
        in.sessions);
  CHECK(in.boots_last == 0 && B.in_app, "the last retry: the board on the release doesn't take it (%u boots)",
        static_cast<unsigned>(in.boots_last));
  CHECK(in.texts.size() == 3 && in.texts[0] == "retry 1 of 2 for boards that may have missed the update" &&
            in.texts[1] == "retry 2 of 2 for boards that may have missed the update",
        "texts: '%s', '%s'", in.texts.size() > 1 ? in.texts[0].c_str() : "", in.texts.size() > 1 ? in.texts[1].c_str() : "");
  // auto_retry 0: one session, as before
  fresh_board();
  const Install off = install(&im_new, 0, false, r, LOSE_FIRST);
  CHECK(off.sessions == 1 && !B.in_app, "auto_retry 0: one session (the board stays in its bootloader)");
  // a clean line: still every retry (blind), nothing reboots after the first
  fresh_board();
  const Install clean = install(&im_new, 2, false, r, NO_LOSS);
  CHECK(clean.sessions == 3 && clean.boots_last == 0 && running(&im_new), "a clean line: 3 sessions, installed once");
  CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/// A line of Diff-Power receivers (verify): a session again only while the receiver doesn't run the release.
static void test_verify() {
  std::printf("verify (receivers answer), auto_retry 2\n");
  AutoRetry r;
  fresh_board();
  const Install clean = install(&im_new, 2, true, r, NO_LOSS);
  CHECK(clean.sessions == 1 && running(&im_new) && r.installed() && r.used() == 0,
        "installed at once: no retry (%d sessions)", clean.sessions);
  CHECK(clean.texts.size() == 2 && clean.texts[0] == "installed; checking the 1 receiver" &&
            clean.texts[1] == "the receiver on 0.6.9",
        "texts: '%s' / '%s'", clean.texts[0].c_str(), clean.texts.back().c_str());

  fresh_board();
  const Install missed = install(&im_new, 2, true, r, LOSE_FIRST);
  CHECK(missed.sessions == 2 && running(&im_new) && r.installed() && r.used() == 1,
        "the first session missed: one retry installs it, no more (%d sessions)", missed.sessions);
  CHECK(missed.texts.size() == 4 && missed.texts[1] == "retry 1 of 2: receiver 1 doesn't answer" &&
            missed.texts[2] == "retry 1 of 2: installed; checking the 1 receiver" &&
            missed.texts[3] == "the receiver on 0.6.9",
        "texts: '%s' / '%s' / '%s'", missed.texts[1].c_str(), missed.texts[2].c_str(), missed.texts[3].c_str());

  fresh_board();
  const Install never = install(&im_new, 2, true, r, LOSE_ALL);
  CHECK(never.sessions == 3 && !B.in_app && !r.installed() && r.pending() == 1,
        "missed every time: 3 sessions, then it waits in its bootloader (%d)", never.sessions);
  CHECK(never.texts.back() == "0 of 1 receivers on 0.6.9 after 2 retries; receiver 1 doesn't answer (it waits in its "
                              "bootloader, red 1 Hz, for the next install)",
        "text: '%s'", never.texts.back().c_str());
  // the next install (the user's) reaches it in its bootloader
  const Install next = install(&im_new, 2, true, r, NO_LOSS);
  CHECK(next.sessions == 1 && running(&im_new) && r.installed(), "the next install takes it");

  // an image this board can't take (other hardware): no retry, it stays on its version
  fresh_board();
  make_image(&im_other, 8192, V(0, 6, 9, 0xFF), 2, REV_OK, 1, 303);
  const Install other = install(&im_other, 2, true, r, NO_LOSS);
  CHECK(other.sessions == 1 && running(&im_old) && r.cant() == 1 && !r.installed() &&
            other.texts.back().find("receiver 1 can't take it (hardware, bootloader") != std::string::npos,
        "other hardware: no retry, '%s'", other.texts.back().c_str());
  // a newer version on the board (no FORCE): it rejects the older release, no retry
  fresh_board();
  make_image(&im_other, 8192, V(0, 6, 7, 0xFF), 1, REV_OK, 1, 304);
  const Install older = install(&im_other, 2, true, r, NO_LOSS);
  CHECK(older.sessions == 1 && running(&im_old) && r.cant() == 1, "an older release: rejected, no retry");
  CHECK(fl_violations == 0 && keep_areas_intact(), "flash rules");
}

/// The policy's own rules: several receivers, the check time, the texts.
static void test_policy() {
  std::printf("the policy: several receivers, the check time, the texts\n");
  AutoRetry r;
  r.begin(2, true);
  CHECK(r.session_done(1000) == AutoRetry::CHECK, "verify: check first");
  RxUpdate s[4] = {RXU_ON_VERSION, RXU_OLD, RXU_NO_ANSWER, RXU_CANT};
  CHECK(r.check(1000 + AutoRetry::CHECK_MS - 1, s, 4) == AutoRetry::CHECK, "within the check time: wait");
  CHECK(r.check(1000 + AutoRetry::CHECK_MS, s, 4) == AutoRetry::RETRY && r.used() == 1, "then retry");
  CHECK(r.retry_text(0x000609FF) == "retry 1 of 2: receiver 3 doesn't answer, receiver 2 not on 0.6.9 yet",
        "'%s'", r.retry_text(0x000609FF).c_str());
  CHECK(r.session_done(50000) == AutoRetry::CHECK, "the retry's session: check again");
  s[1] = s[2] = RXU_ON_VERSION;
  CHECK(r.check(50001, s, 4) == AutoRetry::FINISH && r.installed(), "on the release (one can't): done at once");
  CHECK(r.result_text(0x000609FF) == "3 of 4 receivers on 0.6.9; receiver 4 can't take it (hardware, bootloader "
                                     "or a newer version)",
        "'%s'", r.result_text(0x000609FF).c_str());
  s[0] = s[1] = s[2] = s[3] = RXU_ON_VERSION;
  CHECK(r.check(50002, s, 4) == AutoRetry::FINISH && r.result_text(0x000609FF) == "all 4 receivers on 0.6.9",
        "'%s'", r.result_text(0x000609FF).c_str());
  r.begin(0, true);
  CHECK(r.session_done(0) == AutoRetry::FINISH, "auto_retry 0 (and FORCE / recover): no check, no retry");
  r.begin(1, false);
  CHECK(r.session_done(0) == AutoRetry::RETRY && r.prefix() == "retry 1 of 1: " &&
            r.session_done(0) == AutoRetry::FINISH,
        "blind: exactly the retries");
  // receiver_update_state against a receiver's pages
  ReceiverData d;
  const UpdateTarget t{0x000609FF, 1, 1u << 3, 2};
  CHECK(receiver_update_state(d, false, t) == RXU_NO_ANSWER && receiver_update_state(d, true, t) == RXU_OLD,
        "no answer / not read yet");
  d.have_identity = d.have_status = true;
  d.hw_id = 1;
  d.hw = 0x13;
  d.bl_version = 2;
  d.fw_word = 0x000608FF;
  CHECK(receiver_update_state(d, true, t) == RXU_OLD, "an older version: pending");
  d.fw_word = 0x000609FF;
  CHECK(receiver_update_state(d, true, t) == RXU_ON_VERSION, "the release");
  d.fw_word = 0x000700FF;
  CHECK(receiver_update_state(d, true, t) == RXU_CANT, "a newer one: can't (no FORCE)");
  d.fw_word = 0x000608FF;
  d.bl_version = 1;
  CHECK(receiver_update_state(d, true, t) == RXU_CANT, "an older bootloader: can't");
  d.bl_version = 2;
  d.hw = 0x12;
  CHECK(receiver_update_state(d, true, t) == RXU_CANT, "another board revision: can't");
  d.hw = 0x13;
  d.hw_id = 2;
  CHECK(receiver_update_state(d, true, t) == RXU_CANT, "other hardware: can't");
}

int main() {
  std::printf("ultra update retry test (auto_retry with the bootloader's logic)\n");
  test_policy();
  test_blind();
  test_verify();
  std::printf("\n%d checks, %d failed\n", checks, fails);
  return fails ? 1 : 0;
}
