#include <sanitizer/lsan_interface.h>

// Python, Cling, and DDS own process-lifetime allocation graphs that are not the
// subject of this gate. Disable tracking before Python starts; the generated test
// transform opens a balanced tracking window on its own worker thread.
__attribute__((constructor)) static void disable_lsan_before_python()
{
  __lsan_disable();
}
