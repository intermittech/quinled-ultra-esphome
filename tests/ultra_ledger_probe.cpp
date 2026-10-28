// Optional target object probe: compile -Os -mlongcalls -fno-exceptions -fno-rtti with xtensa-esp32-elf-g++.
#include "../components/ultra_line/ultra_ledger.h"
// Constants belonging only to the frozen round-10 differential reference.
namespace ledger_r10 { static constexpr uint32_t LS_STEP_WH_PER_01H = 100, LS_STEP_WH = 1000; }
#include "fixtures/ultra_ledger_r10.h"
namespace current = esphome::ultra_line;
extern "C" {
__attribute__((noinline)) void ledger_time_before(ledger_r10::LifetimeLedger *l, ledger_r10::LedgerEntry *e, uint64_t ms) {
  l->add_time(*e, true, ms);
}
__attribute__((noinline)) void ledger_time_after(current::LifetimeLedger *l, current::LedgerEntry *e, uint64_t ms) {
  l->add_time(*e, true, ms);
}
char layout_ledger_before[sizeof(ledger_r10::LifetimeLedger)] = {};
char layout_ledger_after[sizeof(current::LifetimeLedger)] = {};
char layout_keeper_before[sizeof(ledger_r10::LedgerKeeper)] = {};
char layout_keeper_after[sizeof(current::LedgerKeeper)] = {};
char layout_entry_before[sizeof(ledger_r10::LedgerEntry)] = {};
char layout_entry_after[sizeof(current::LedgerEntry)] = {};
char layout_position_before[sizeof(ledger_r10::LedgerPosition)] = {};
char layout_position_after[sizeof(current::LedgerPosition)] = {};
char layout_hook[sizeof(current::LedgerLitFlush)] = {};
#ifdef __GLIBCXX__
char layout_hook_allocation[sizeof(std::_Sp_counted_ptr_inplace<current::LedgerLitFlush, std::allocator<void>,
                                                            __gnu_cxx::__default_lock_policy>)] = {};
char layout_hook_registry_slot[sizeof(std::weak_ptr<current::LedgerLitFlush>)] = {};
#endif
}
