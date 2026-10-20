#ifndef BUZZER_H
#define BUZZER_H

#include "stdint.h"

void buzzer_init(void);
void buzzer_beep(uint8_t volume, uint8_t duration_ms);

#endif /* BUZZER_H */