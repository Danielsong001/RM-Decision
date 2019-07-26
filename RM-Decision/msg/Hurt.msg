uint8 armor_type
# 0-3bits: the attacked armor id:
#   0x00: 0 front
#   0x01: 1 left
#   0x02: 2 behind
#   0x03: 3 right
#   others reserved*/
uint8 hurt_type
# 4-7bits: blood volume change type
#   0x00: armor attacked
#   0x01: module offline
#   0x02: bullet over speed
#   0x03: bullet over frequency */
