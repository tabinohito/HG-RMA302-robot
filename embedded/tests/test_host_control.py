import unittest
import test_power_integration as integrated
PROGRAM = integrated.PROGRAM

SCENARIO = r'''
static void expect_mode_leds(uint32_t state, uint32_t mode) {
    assert(led_changes);
    for (unsigned i=0; i<30; ++i) assert(led_pixels[i]==(i<24 ? state : mode));
}

static void command_text(const char *text) {
    char command[128]; snprintf(command,sizeof(command),"%s",text);
    console(command,now_us/1000);
}
static void scenario(uint32_t ms) {
    switch(ms) {
    case 250: assert(!control.enabled && !host.managed && !gpio_get(2)); break;
    case 300: command_text("claim 123"); assert(host.managed && !gpio_get(2)); break;
    case 350: command_text("host 123 1 READY"); break;
    case 400: switch_down=true; break;
    case 450: assert(control.enabled && gpio_get(2)); expect_mode_leds(0x08000800,0x08000c00); break;
    case 500: switch_down=false; command_text("host 123 2 DRIVING joystick"); break;
    case 550: expect_mode_leds(0x0c000000,0x060c0000); break;
    case 600: command_text("estop"); assert(host.estop && !control.enabled && !gpio_get(2)); break;
    case 650: switch_down=true; break;
    case 700:
        assert(!gpio_get(2)); expect_leds(0x000c0000);
        command_text("clear 123"); assert(host.estop);
        command_text("host 123 3 DRIVING"); assert(host.state==CART_STOPPED);
        break;
    case 750: switch_down=false; break;
    case 800: command_text("clear 123"); assert(!host.estop && !gpio_get(2)); break;
    case 850: switch_down=true; break;
    case 900: assert(gpio_get(2)); command_text("host 123 4 DRIVING"); break;
    case 950: switch_down=false; break;
    case 1649: assert(gpio_get(2)); break;
    case 1700:
        assert(!gpio_get(2) && host.estop && !strcmp(host.stop_reason,"host_timeout"));
        command_text("host 123 4 DRIVING"); assert(host.seen_at==900);
        command_text("host -1 5 DRIVING"); assert(host.seen_at==900);
        command_text("host 123 5 UNKNOWN"); assert(host.seen_at==900);
        command_text("host 123 5 READY invalid"); assert(host.seen_at==900);
        command_text("host 123 5 READY tank extra"); assert(host.seen_at==900);
        command_text("host 123 5 DRIVING"); assert(host.estop && !gpio_get(2));
        command_text("clear 125"); assert(host.estop);
        break;
    case 1800: command_text("clear 123"); assert(!host.estop && !gpio_get(2)); switch_down=true; break;
    case 1850: assert(gpio_get(2)); break;
    case 1900: switch_down=false; mock_usb_connected=false; break;
    case 1950:
        assert(!gpio_get(2) && host.estop && !strcmp(host.stop_reason,"usb_disconnect"));
        mock_usb_connected=true;
        command_text("claim 999"); assert(host.session==123);
        break;
    case 2500:
        command_text("claim 999"); assert(host.session==999 && host.estop && !gpio_get(2));
        command_text("host 123 6 READY"); assert(host.sequence==0);
        command_text("host 999 1 READY");
        command_text("clear 999"); assert(!host.estop && !gpio_get(2));
        command_text("on"); assert(!gpio_get(2)); // No remote-on command.
        break;
    case 2600: command_text("release 999"); assert(!host.managed && !gpio_get(2)); break;
    case 2700: switch_down=true; break;
    case 2750:
        assert(gpio_get(2)); command_text("estop"); assert(!gpio_get(2) && host.estop);
        assert(buzz_count==0); // All host and physical actions stay muted.
        break;
    case 2800: longjmp(finished,1);
    }
}
'''

class HostIntegrationTests(unittest.TestCase):
    def test_production_loop_ssr_leds_latched_stop_disconnect_timeout_and_session(self):
        start=PROGRAM.index('static void scenario(')
        end=PROGRAM.index('static void advance_us(',start)
        program=(PROGRAM[:start]+SCENARIO+PROGRAM[end:]).replace('now_us == 4950000','now_us == 2800000')
        integrated.IntegratedPowerTests().run_controller_flow(False,program=program)

if __name__=='__main__': unittest.main()
