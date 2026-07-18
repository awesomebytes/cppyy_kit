#include <climits>
#include <cstdlib>
#include <cstring>
#include <string>

__attribute__((noinline)) void trigger_asan()
{
  char * bytes = new char[8];
  volatile int past_end = 8;
  bytes[past_end] = 'x';
  delete[] bytes;
}

__attribute__((noinline)) int trigger_ubsan()
{
  volatile int maximum = INT_MAX;
  volatile int one = 1;
  return maximum + one;
}

__attribute__((noinline)) void trigger_lsan()
{
  void * leaked = std::malloc(1024);
  std::memset(leaked, 0x5a, 1024);
  asm volatile("" : : "r"(leaked) : "memory");
}

int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 64;
  }
  const std::string mode(argv[1]);
  if (mode == "asan") {
    trigger_asan();
    return 0;
  }
  if (mode == "ubsan") {
    return trigger_ubsan() == 0;
  }
  if (mode == "lsan") {
    trigger_lsan();
    return 0;
  }
  return 64;
}
