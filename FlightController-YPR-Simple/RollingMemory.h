#ifndef Memory_h
#define Memory_h

// Rolling average strcture
typedef struct {
  double memory[40] = {0};
  double sum = 0;
  uint8_t index = 0;
  double samples = 20.0;
  uint8_t averaged_samples = 0;
}RollingMemory;

#endif
