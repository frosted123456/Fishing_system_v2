// Minimal Unity-compatible shim so the same test files run with plain g++ (make -C test).
// With PlatformIO (`pio test -e native`) the real Unity framework is used instead and this
// file is ignored (it is only on the include path of the Makefile build).
#ifndef ICEMESH_UNITY_SHIM_H
#define ICEMESH_UNITY_SHIM_H
#include <cstdio>
#include <cstring>
#include <cstdint>

namespace unity_shim {
struct State { int tests; int failures; int cur_failed; const char* cur_name; };
inline State& st() { static State s = {0, 0, 0, ""}; return s; }
inline void fail(const char* file, int line, const char* msg) {
  if (!st().cur_failed) st().failures++;
  st().cur_failed = 1;
  std::printf("%s:%d:%s:FAIL: %s\n", file, line, st().cur_name, msg);
}
}  // namespace unity_shim

void setUp(void);
void tearDown(void);

#define UNITY_BEGIN() (unity_shim::st().tests = 0, unity_shim::st().failures = 0, 0)
#define UNITY_END() \
  (std::printf("\n-----------------------\n%d Tests %d Failures 0 Ignored\n%s\n", \
               unity_shim::st().tests, unity_shim::st().failures,              \
               unity_shim::st().failures ? "FAIL" : "OK"),                     \
   unity_shim::st().failures)
#define RUN_TEST(fn)                                                     \
  do {                                                                   \
    unity_shim::st().tests++;                                            \
    unity_shim::st().cur_failed = 0;                                     \
    unity_shim::st().cur_name = #fn;                                     \
    setUp();                                                             \
    fn();                                                                \
    tearDown();                                                          \
    if (!unity_shim::st().cur_failed) std::printf("%s:PASS\n", #fn);     \
  } while (0)

#define UNITY_SHIM_CHECK(cond, msg) \
  do { if (!(cond)) { unity_shim::fail(__FILE__, __LINE__, msg); return; } } while (0)

#define TEST_ASSERT_TRUE(c)            UNITY_SHIM_CHECK((c), "expected TRUE: " #c)
#define TEST_ASSERT_FALSE(c)           UNITY_SHIM_CHECK(!(c), "expected FALSE: " #c)
#define TEST_ASSERT(c)                 TEST_ASSERT_TRUE(c)
#define TEST_ASSERT_NULL(p)            UNITY_SHIM_CHECK((p) == nullptr, "expected NULL: " #p)
#define TEST_ASSERT_NOT_NULL(p)        UNITY_SHIM_CHECK((p) != nullptr, "expected not NULL: " #p)
#define TEST_ASSERT_EQUAL(e, a)        UNITY_SHIM_CHECK((long long)(e) == (long long)(a), "expected " #e " == " #a)
#define TEST_ASSERT_EQUAL_INT(e, a)    TEST_ASSERT_EQUAL(e, a)
#define TEST_ASSERT_EQUAL_UINT8(e, a)  TEST_ASSERT_EQUAL((uint8_t)(e), (uint8_t)(a))
#define TEST_ASSERT_EQUAL_UINT16(e, a) TEST_ASSERT_EQUAL((uint16_t)(e), (uint16_t)(a))
#define TEST_ASSERT_EQUAL_UINT32(e, a) TEST_ASSERT_EQUAL((uint32_t)(e), (uint32_t)(a))
#define TEST_ASSERT_EQUAL_PTR(e, a)    UNITY_SHIM_CHECK((const void*)(e) == (const void*)(a), "expected same pointer " #e " " #a)
#define TEST_ASSERT_EQUAL_MEMORY(e, a, n) UNITY_SHIM_CHECK(std::memcmp((e), (a), (n)) == 0, "memory differs " #e " " #a)
#define TEST_ASSERT_UINT32_WITHIN(d, e, a) \
  UNITY_SHIM_CHECK(((long long)(a) >= (long long)(e) - (long long)(d)) && ((long long)(a) <= (long long)(e) + (long long)(d)), "not within " #d " of " #e ": " #a)
#define TEST_ASSERT_LESS_OR_EQUAL(limit, a) UNITY_SHIM_CHECK((long long)(a) <= (long long)(limit), "expected " #a " <= " #limit)
#define TEST_ASSERT_GREATER_OR_EQUAL(limit, a) UNITY_SHIM_CHECK((long long)(a) >= (long long)(limit), "expected " #a " >= " #limit)

#endif
