// Proves the harness itself works (shim under make, real Unity under pio test).
#include <unity.h>
#include <IceMesh.h>   // umbrella must compile on its own

void setUp(void) {}
void tearDown(void) {}

static void test_harness_runs(void) { TEST_ASSERT_EQUAL(4, 2 + 2); }

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_harness_runs);
  return UNITY_END();
}
