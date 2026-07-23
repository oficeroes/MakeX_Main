import novapi
import time
from mbuild import gamepad
from mbuild.led_matrix import led_matrix_class

__led = led_matrix_class('PORT2', 'INDEX1')
__led.show('rdy')
time.sleep(0.5)

while True:
    Lx = gamepad.get_joystick('Lx')
    Ly = gamepad.get_joystick('Ly')
    Rx = gamepad.get_joystick('Rx')

    aLx = abs(Lx)
    aLy = abs(Ly)
    aRx = abs(Rx)

    best = max(aLx, aLy, aRx)

    if best < 3:
        __led.show('----')
    elif best == aLy:
        __led.show('y%d' % int(Ly))
    elif best == aLx:
        __led.show('x%d' % int(Lx))
    else:
        __led.show('r%d' % int(Rx))

    time.sleep(0.05)
