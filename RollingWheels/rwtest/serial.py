import wiringpi2 as wiringpi
wiringpi.wiringPiSetup()
serial = wiringpi.serialOpen('/dev/ttyAMA0',115200)
wiringpi.serialPuts(serial,'hello world!')
