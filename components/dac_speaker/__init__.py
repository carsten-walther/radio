"""dac_speaker - a speaker platform for the ESP32's internal DAC.

ESPHome removed internal-DAC support from the i2s_audio speaker ("Internal DAC
is no longer supported", a hard error since the move to the IDF 5 I2S driver,
which itself dropped the built-in DAC mode). The silicon did not go anywhere:
IDF 5 replaced that mode with a dedicated DMA driver, driver/dac_continuous.h.
This component wraps that driver in the ordinary Speaker interface, so the
speaker media_player can feed it like any other output.

Exists for boards whose amplifier is hard-wired to a DAC pin - here the CYD's
on-board amp on GPIO26 - where "use an external I2S DAC instead" would mean
abandoning the built-in speaker entirely.
"""

CODEOWNERS = ["@carsten-walther"]
