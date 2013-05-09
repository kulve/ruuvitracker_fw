require('led')

local failed=0

print("Starting Production test on RuuviTracker RevB1\n")
print("Press any key to start\n")

while uart.read(0,1,0) == "" do
   ruuvi.delay_ms(100)
end

print("Turning GREEN and RED leds on\n")
green_led:on()
red_led:on()
print("Check leds!\n")


--Test LSM303DLHC
--Linear acceleration address is 0011001b = 0x19
--Magnetic interface address is 11110b (0x1E)
print("Test LSM303DLHC, reg 0x20")
local byte = i2c.read_from(0, 0x19, 0x20, 1)
XEN = 0  -- Axis enable bits
YEN = 1
ZEN = 2
if     bit.isset(byte, XEN)
   and bit.isset(byte, YEN)
   and bit.isset(byte, ZEN) then
   print("PASS")
else
   print("FAIL")
   print(string.format("%02X", byte))
   failed=failed+1
end

print("Test BQ24190")
PG_STAT = 2 -- Power good
THERM_STAT = 1 -- 1=Thermal requlation
byte = i2c.read_from(0, 0x6B, 0x08, 1)
if bit.isset(byte, PG_STAT) and not bit.isset(byte, THERM_STAT) then
   print("PASS")
else
   print("FAIL")
   print(string.format("%02X", byte))
   failed=failed+1
end

print("Test EEPROM")
byte = i2c.read_from(0, 0x50, 0x00, 1)
if byte==0xFF then
   print("PASS")
else
   print("FAIL")
   print(string.format("%02X", byte))
   failed=failed+1
end

print("Test GSM")
gsm.set_power_state(gsm.POWER_ON)
ruuvi.delay_ms(10e3) -- Wait 10s
if gsm.state() > gsm.STATE_OFF and gsm.flag_is_set(gsm.SIM_INSERTED) then
   gsm.set_power_state(gsm.POWER_OFF)
   print("PASS")
else
   gsm.set_power_state(gsm.POWER_OFF)
   print("FAIL")
   failed=failed+1
end

if failed == 0 then
   print("Success\n\nDEVICE PASSED")
   red_led:off()
else
   print("Failed tests ", failed)
   green_led:off()
end
